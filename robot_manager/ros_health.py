#!/usr/bin/env python3
"""Short lived ROS probe, isolated from the management socket server."""
import json
import socket
import subprocess
import sys
import time
from video_probe import video_frames
from stop_evidence import cold_stopped
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from arm_control.action import ExecuteArmGoal
from rclpy.action import ActionClient


def probe():
    if sys.argv[1] in ('config', 'preflight'):
        import yaml
        profile = json.loads(sys.argv[2])
        if profile['mode'] == 'hardware':
            with open(profile['baseline']) as stream:
                if yaml.safe_load(stream).get('commissioned') is not True:
                    return 1
        if profile['video']:
            with open(profile['video_config']) as stream:
                data = yaml.safe_load(stream)
                if not isinstance(data, dict) or not ('streams' in data or 'direct_sources' in data):
                    return 1
        if sys.argv[1] == 'config':
            return 0
        rclpy.init()
        node = Node('robot_manager_preflight')
        deadline = time.monotonic()+1
        while time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        names = node.get_node_names()
        node.destroy_node()
        rclpy.shutdown()
        return 1 if any(n in names for n in ('controller_manager','robot_runtime')) else 0
    rclpy.init()
    node = Node('robot_manager_probe')
    latest = {}
    def state(msg):
        try:
            latest.update(value=json.loads(msg.data), at=time.monotonic())
        except ValueError:
            pass
    node.create_subscription(String, '/robot_runtime/state', state, 1)
    if sys.argv[1] == 'halt':
        client = node.create_client(Trigger, '/robot_runtime/halt')
        deadline = time.monotonic()+27
        if not client.wait_for_service(timeout_sec=12):
            print(json.dumps({'halt_result':'service unavailable'}))
            return 1
        # A cold software-only stack has never configured its hardware. Prove
        # that with read-only controller-manager responses instead of issuing
        # needless halt mutations (which can themselves have unknown outcomes).
        readers = [node.create_client(kind, '/controller_manager/'+name) for kind,name in
                   [(ListControllers,'list_controllers'),(ListHardwareComponents,'list_hardware_components')]]
        reads = [None,None]
        proof_until = min(deadline,time.monotonic()+4)
        while time.monotonic()<proof_until:
            rclpy.spin_once(node,timeout_sec=.05)
            for i,reader in enumerate(readers):
                if reads[i] is None and reader.service_is_ready():
                    reads[i]=reader.call_async(ListControllers.Request() if i==0 else ListHardwareComponents.Request())
            if all(f is not None and f.done() for f in reads):
                controllers=reads[0].result().controller
                hardware=reads[1].result().component
                if any(h.state.id!=1 for h in hardware):
                    break  # Configured hardware always follows explicit halt.
                runtime=latest.get('value',{})
                if cold_stopped([c.state for c in controllers],
                                {h.name:h.state.id for h in hardware}, runtime,
                                time.monotonic()-latest.get('at',0)):
                    return 0
                reads=[None,None]
        for future in reads:
            if future is not None and not future.done():future.cancel()
        reason = 'no confirmed stopped state'
        futures = []
        # Halt is idempotent and only revokes motion. A transient guard/monitor
        # observation may reject the first halt; never retry enable or resume.
        for attempt in range(2):
            remaining = deadline-time.monotonic()
            if remaining <= 0: break
            future = client.call_async(Trigger.Request())
            futures.append(future)
            rclpy.spin_until_future_complete(node, future, timeout_sec=min(10, remaining))
            if not future.done():
                reason = 'halt response timeout; outcome unknown'
                continue
            if not any(f.done() and f.result().success for f in futures):
                reason = future.result().message
                if 'ownership reconciliation in progress' in reason:
                    # The runtime advertises its service before owner auditing
                    # completes. Wait for observed completion, not a fixed sleep.
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=.05)
                        runtime = latest.get('value', {})
                        if (runtime.get('diagnostics', {}).get('readable') is True and
                                time.monotonic()-latest.get('at',0)<.5):
                            break
                continue
            observe_until = min(deadline, time.monotonic()+3)
            while time.monotonic() < observe_until:
                rclpy.spin_once(node, timeout_sec=.05)
                runtime = latest.get('value', {})
                groups = runtime.get('groups', {})
                if (time.monotonic()-latest.get('at',0)<.5 and
                        runtime.get('state') in ('Stopped','Halted') and
                        all(s != 'Running' for s in groups.values())):
                    return 0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            runtime = latest.get('value', {})
            if (any(f.done() and f.result().success for f in futures) and
                    time.monotonic()-latest.get('at',0)<.5 and
                    runtime.get('state') in ('Stopped','Halted') and
                    all(s != 'Running' for s in runtime.get('groups',{}).values())):
                return 0
        print(json.dumps({'halt_result':reason}))
        return 1
    clients = [node.create_client(kind, '/controller_manager/'+name) for kind,name in [(ListControllers,'list_controllers'),(ListHardwareComponents,'list_hardware_components')]]
    pending = [None,None]
    action = ActionClient(node, ExecuteArmGoal, '/arm_control/execute_goal')
    deadline = time.monotonic()+7
    core = False
    hardware = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        for i,c in enumerate(clients):
            if pending[i] is None and c.service_is_ready():
                pending[i] = c.call_async(ListControllers.Request() if i == 0 else ListHardwareComponents.Request())
        if all(f is not None and f.done() for f in pending) and latest:
            cs = pending[0].result().controller
            hs = pending[1].result().component
            hardware = {h.name: h.state.id for h in hs}
            required = {'joint_state_broadcaster','arm_position_controller','flipper_csp_controller','flipper_csv_forward_controller','gripper_controller','wheel_controller'}
            core = (required <= {c.name for c in cs if c.state in ('inactive','active')}
                    and {h.name for h in hs} == {'RobotCanopenSystem','CanDriverSystem'}
                    and time.monotonic()-latest['at'] < .5 and action.server_is_ready()
                    and latest['value'].get('diagnostics',{}).get('readable') is True)
            if core: break
    modules = dict(core=dict(ok=core))
    # This monitor handshake never takes control or resets the operator session.
    bridge = False
    bridge_state = None
    try:
        with socket.create_connection(('127.0.0.1',int(sys.argv[2])),timeout=.6) as conn:
            conn.settimeout(.6)
            f = conn.makefile('rb')
            hello = json.loads(f.readline(65536))
            caps = json.loads(f.readline(65536))
            conn.sendall(b'{"type":"heartbeat","protocol_version":1,"seq":987654}\n')
            end = time.monotonic()+1
            while time.monotonic()<end:
                msg = json.loads(f.readline(65536))
                if msg.get('type')=='heartbeat_ack' and msg.get('seq')==987654:
                    bridge = (hello.get('type')=='hello' and
                              {'sync_request','readonly_monitor','monotonic_watchdog','connection_control_owner','control_v2','legacy_readonly'} <= set(caps.get('supports',[])))
                    conn.sendall(b'{"type":"sync_request","protocol_version":1,"seq":987655}\n')
                    while time.monotonic()<end:
                        snapshot = json.loads(f.readline(65536))
                        if snapshot.get('type')=='system_snapshot' and snapshot.get('seq')==987655:
                            bridge_state = snapshot
                            break
                    break
    except (OSError, ValueError): pass
    modules['bridge'] = dict(ok=bridge, robot_id=hello.get('robot_id') if bridge else None)
    if sys.argv[3]=='true':
        video = False
        video_reason = 'video_manager_unavailable'
        try:
            with socket.create_connection(('127.0.0.1',18081), timeout=.6) as conn:
                conn.sendall(b'{"type":"list_cameras","seq":1}\n')
                msg = json.loads(conn.makefile('rb').readline(65536))
                cameras = msg.get('cameras',[])
                if msg.get('ok',False):
                    video,video_reason = video_frames(cameras)
        except (OSError,ValueError): pass
        modules['video'] = dict(ok=bool(video),reason=video_reason)
    rclpy.spin_once(node,timeout_sec=.05)
    runtime = latest.get('value') if time.monotonic()-latest.get('at',0) < 1 else None
    safety = dict(hardware_initialized=(all(v in (2,3) for v in hardware.values()) if hardware else None),
                  hardware_enabled=(all(v==3 for v in hardware.values()) if hardware else None),
                  hardware_components=hardware,
                  groups=(runtime or {}).get('groups'),
                  global_inhibit=(runtime or {}).get('diagnostics',{}).get('global_inhibit'),
                  emergency=(bridge_state or {}).get('emergency'))
    print(json.dumps(dict(modules=modules, runtime=runtime, safety=safety, observed_at=time.time())))
    node.destroy_node()
    rclpy.shutdown()
    return 0

def main():
    try:
        return probe()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    raise SystemExit(main())
