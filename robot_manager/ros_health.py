#!/usr/bin/env python3
"""Short lived ROS probe, isolated from the management socket server."""
import json
import socket
import subprocess
import sys
import time
from video_probe import video_frames
from stop_evidence import cold_stopped
from read_probe import ReadProbe
from group_selection import requirements, arm_services
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
        if profile['mode'] == 'hardware' and sys.argv[1] == 'config':
            with open(profile['baseline']) as stream:
                baseline = yaml.safe_load(stream)
                groups, _, _ = requirements(profile.get('requested_presets'))
                if baseline.get('commissioned') is not True:
                    if (baseline.get('schema_version') != 'robot24-parameter-baseline-v2' or
                            baseline.get('parameters_verified') is not True or
                            not groups <= set(baseline.get('commissioned_groups', []))):
                        print(json.dumps({'baseline_error': '现场 CANopen 身份/基线尚未完成登记，涉及组: ' +
                                          ','.join(sorted(groups - set(baseline.get('commissioned_groups', []))))}, ensure_ascii=False))
                        return 1
        if profile['video']:
            with open(profile['video_config']) as stream:
                data = yaml.safe_load(stream)
                if not isinstance(data, dict) or not ('streams' in data or 'direct_sources' in data):
                    return 1
        if sys.argv[1] == 'config':
            return 0
        if 'arm' in requirements(profile.get('requested_presets'))[0]:
            from pathlib import Path
            import os
            from ament_index_python.packages import get_package_prefix
            packages = ['arm_control', 'car_control', 'car_moveit_config', 'moveit_ros_move_group']
            if profile.get('enable_servo', False):
                packages.append('moveit_servo')
            try:
                prefixes = {package: Path(get_package_prefix(package)) for package in packages}
                executables = ['arm_goal_executor', 'wrist_roll_service_node', 'gripper_cmd_node']
                if profile.get('enable_servo', False):
                    executables.append('servo_readiness')
                for executable in executables:
                    path = prefixes['arm_control'] / 'lib/arm_control' / executable
                    if not path.is_file() or not os.access(path, os.X_OK):
                        raise RuntimeError('missing arm_control executable: ' + executable)
            except Exception as error:
                print(json.dumps({'dependency_error': str(error)}))
                return 1
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
        _, _, expected_backends = requirements(json.loads(sys.argv[2]) if len(sys.argv)>2 else None)
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
                                time.monotonic()-latest.get('at',0), expected_backends):
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
    reads = [ReadProbe(client, kind.Request) for client,kind in zip(clients,(ListControllers,ListHardwareComponents))]
    groups, required, backends = requirements(json.loads(sys.argv[4]) if len(sys.argv)>4 else None)
    action = ActionClient(node, ExecuteArmGoal, '/arm_control/execute_goal') if 'arm' in groups else None
    servo_required = len(sys.argv)>5 and sys.argv[5]=='true'
    service_names = arm_services(json.loads(sys.argv[4]) if len(sys.argv)>4 else None, servo_required)
    servo_services = [node.create_client(Trigger, name) for name in service_names]
    planner = None
    if 'arm' in groups:
        from moveit_msgs.srv import GetPlanningScene
        planner = node.create_client(GetPlanningScene, '/get_planning_scene')
    if service_names:
        required.add('arm_servo_controller')
    deadline = time.monotonic()+7
    core = False
    hardware = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
        values = [read.poll() for read in reads]
        if all(value is not None for value in values) and latest:
            cs = values[0].controller
            hs = values[1].component
            hardware = {h.name: h.state.id for h in hs}
            core = (required <= {c.name for c in cs if c.state in ('inactive','active')}
                    and backends <= {h.name for h in hs}
                    and time.monotonic()-latest['at'] < .5 and (action is None or action.server_is_ready())
                    and latest['value'].get('diagnostics',{}).get('readable') is True
                    and all(client.service_is_ready() for client in servo_services)
                    and (planner is None or planner.service_is_ready()))
            if core: break
            # Controllers may still be loading/switching after the first reply.
            # Retry only reads; never replay lifecycle or enable operations.
            for read in reads:
                read.refresh()
    missing_services=[name for name, client in zip(service_names, servo_services) if not client.service_is_ready()]
    if planner is not None and not planner.service_is_ready():missing_services.append('/get_planning_scene')
    modules = dict(core=dict(ok=core, missing_services=missing_services,
                            missing_controllers=sorted(required - {c.name for c in values[0].controller if c.state in ('inactive','active')}) if values[0] else sorted(required),
                            missing_backends=sorted(backends - set(hardware or {})),
                            runtime_readable=latest.get('value',{}).get('diagnostics',{}).get('readable') is True,
                            runtime_age_seconds=round(time.monotonic()-latest['at'],3) if latest else None,
                            arm_action_ready=action.server_is_ready() if action else None))
    # This monitor handshake never takes control or resets the operator session.
    bridge = False
    bridge_state = None
    for attempt in range(2):
        bridge = False
        bridge_state = None
        try:
            with socket.create_connection(('127.0.0.1',int(sys.argv[2])),timeout=.6) as conn:
                conn.settimeout(.6)
                try: conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                except OSError: pass
                with conn.makefile('rb') as f:
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
        if bridge and bridge_state is not None:
            break
        bridge = False
    modules['bridge'] = dict(ok=bridge, robot_id=hello.get('robot_id') if bridge else None)
    if sys.argv[3]=='true':
        video = False
        video_reason = 'video_manager_unavailable'
        try:
            with socket.create_connection(('127.0.0.1',18081), timeout=.6) as conn:
                conn.settimeout(.6)
                try: conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                except OSError: pass
                conn.sendall(b'{"type":"list_cameras","seq":1}\n')
                with conn.makefile('rb') as vf:
                    msg = json.loads(vf.readline(65536))
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
