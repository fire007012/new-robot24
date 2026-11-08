#!/usr/bin/env python3
"""Actual installed ROS mock end-to-end over management TCP. Outputs outside source."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from client import request

p=argparse.ArgumentParser()
p.add_argument('--setup',required=True)
p.add_argument('--output',required=True)
p.add_argument('--rounds',type=int,default=20)
p.add_argument('--optional-video',action='store_true')
p.add_argument('--restart-during-start',action='store_true')
p.add_argument('--exercise-running',action='store_true')
p.add_argument('--config-transactions',action='store_true')
a=p.parse_args()
root=Path(a.output).resolve()
root.mkdir(parents=True,exist_ok=False)
profiles=root/'profiles';profiles.mkdir()
profile=dict(profile_id='mock',config_revision='1',mode='mock',video=False,video_required=False,devices=[],startup_timeout=60,build_version='integration')
if a.optional_video:
    (profiles/'video.yaml').write_text('rtsp:\n  public_host: 127.0.0.1\n  publish_host: 127.0.0.1\n  port: 8554\ndirect_sources: []\nstreams: []\n')
    profile.update(video=True,video_config=str(profiles/'video.yaml'))
(profiles/'mock.json').write_text(json.dumps(profile))
token='local-integration-token-32-characters-minimum'
(root/'token').write_text(token)
config=dict(enable_config_transactions=a.config_transactions,robot_id='robot24',config_root=str(profiles),state_dir=str(root/'state'),token_file=str(root/'token'),setup=a.setup,build_version='integration',bind='127.0.0.1',port=19092,control_endpoint=dict(host='127.0.0.1',port=19090,protocol_version=2))
(root/'setup.bash').write_text('source '+a.setup+'\n')
(root/'release.json').write_text(json.dumps(dict(build_version='integration',management_protocol=1,control_protocol=2,files={'setup.bash':hashlib.sha256((root/'setup.bash').read_bytes()).hexdigest()})))
config['build_manifest']=str(root/'release.json')
(root/'manager.json').write_text(json.dumps(config))
env=dict(os.environ,ROS_DOMAIN_ID='94',ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(root/'ros-logs'),PYTHONDONTWRITEBYTECODE='1')
manager_path=Path(__file__).resolve().parents[1]/'manager.py'
log=(root/'manager.log').open('a')
def launch():
    proc=subprocess.Popen([sys.executable,str(manager_path),'--config',str(root/'manager.json')],env=env,stdout=log,stderr=log)
    deadline=time.monotonic()+10
    while time.monotonic()<deadline:
        try:call('hello');return proc
        except OSError:time.sleep(.1)
    raise AssertionError('manager unavailable')
def call(method,params=None,rid=None):
    result=request('127.0.0.1',19092,token,method,params,rid)
    if not result['ok']:raise AssertionError(result)
    return result['result']
def wait():
    deadline=time.monotonic()+100
    while time.monotonic()<deadline:
        op=call('operation')
        if op['state'] not in ('accepted','running'):return op
        time.sleep(.3)
    raise AssertionError('operation timeout')
selected_revision='1'
def start(rid=None):
    return call('start',dict(robot_id='robot24',profile_id='mock',config_revision=selected_revision),rid)
proc=launch()
results=[]
config_evidence=[]
try:
    if a.config_transactions:
        base=call('config.status')['source_revision']
        candidate=call('config.candidate',dict(base_revision=base,patches=[dict(profile_id='mock',field='startup_timeout',value=65)]))
        assert call('config.status')['source_revision']==base
        applied=call('config.apply',dict(base_revision=base,candidate_revision=candidate['candidate_revision']))
        deadline=time.monotonic()+30
        while True:
            op=call('config.operation',dict(operation_id=applied['operation_id']))
            if op['state'] not in ('accepted','running'):break
            assert time.monotonic()<deadline;time.sleep(.1)
        assert op['state']=='succeeded',op
        before_start=call('config.status')
        assert before_start['pending_restart'] and before_start['actual_runtime_revision'] is None,before_start
        selected_revision=call('profiles')[0]['config_revision']
        config_evidence.append(dict(stage='applied_stopped',status=before_start))
    for i in range(a.rounds):
        begin=time.monotonic()
        op=start('round-'+str(i))
        assert start('round-duplicate-'+str(i))['operation_id']==op['operation_id']
        # Disconnect/reconnect is inherent: each CLI request uses a new TCP socket.
        if a.restart_during_start and i==0:
            deadline=time.monotonic()+15
            while call('operation')['phase']!='health':
                assert time.monotonic()<deadline
                time.sleep(.05)
            assert call('operation')['state']=='running'
            proc.kill();proc.wait();proc=launch()
            assert call('operation')['error']['code']=='RECOVERY_REQUIRED'
            assert call('snapshot')['recovery_required']
            call('stop');recovered=wait();assert recovered['state']=='succeeded',recovered
            op=start('recovered-'+str(i))
        done=wait();assert done['state']=='succeeded',done
        snapshot=call('snapshot')
        assert snapshot['software_state']==('degraded' if a.optional_video else 'ready'),snapshot
        assert snapshot['modules']['bridge'].get('robot_id')=='robot24',snapshot
        if a.optional_video:assert not snapshot['modules']['video']['ok'],snapshot
        if a.config_transactions:
            status=call('config.status')
            assert status['actual_runtime_revision']==candidate['candidate_revision'] and not status['pending_restart'],status
            config_evidence.append(dict(stage='started_health_verified',status=status))
        runtime=snapshot['runtime']
        assert runtime['state'] not in ('Running','Armed'),runtime
        assert all(v not in ('Running','Armed') for v in runtime.get('groups',{}).values()),runtime
        if i==0 and a.exercise_running:
            # Explicit test-only authorization inside the hardcoded isolated
            # mock domain. The production manager has no such methods.
            services=[('/robot_runtime/init','std_srvs/srv/Trigger','{}')]
            services += [('/mock/conditions/'+condition,'std_srvs/srv/SetBool','{data: true}') for condition in ('arm_secured','support_trusted','payload_retained')]
            services += [('/robot_runtime/'+method,'std_srvs/srv/Trigger','{}') for method in ('enable','resume')]
            for service,kind,payload in services:
                rpc=subprocess.Popen(['/bin/bash','-c','source "$1" && shift && exec "$@"','mock-test',a.setup,'ros2','service','call',service,kind,payload],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
                try:out=rpc.communicate(timeout=30)[0].decode()
                except subprocess.TimeoutExpired:
                    os.killpg(rpc.pid,signal.SIGKILL);rpc.communicate();raise
                assert rpc.returncode==0 and 'success=True' in out,out
            deadline=time.monotonic()+20
            while call('snapshot').get('runtime',{}).get('state')!='Running':
                assert time.monotonic()<deadline
                time.sleep(.1)
        if i==0 and not a.restart_during_start:
            # Crash/restart daemon with live resources. No replay, no cached ready.
            proc.kill();proc.wait();proc=launch()
            assert call('snapshot')['recovery_required']
            result=request('127.0.0.1',19092,token,'start',dict(robot_id='robot24',profile_id='mock',config_revision=selected_revision))
            assert not result['ok'] and result['error']['code']=='RECOVERY_REQUIRED',result
        if i==1:
            state=json.loads((root/'state/state.json').read_text())
            bridge=next(r for r in state['resources'] if r['module'] in ('bridge','video_stack'))
            os.killpg(bridge['pid'],signal.SIGKILL)
            deadline=time.monotonic()+20
            while call('snapshot')['software_state']!='failed':
                assert time.monotonic()<deadline
                time.sleep(.3)
            assert call('hello')['protocol_version']==1
        call('stop');done=wait();assert done['state']=='succeeded',done
        state=json.loads((root/'state/state.json').read_text())
        assert not state['resources'],state
        with socket.socket() as sock:sock.bind(('0.0.0.0',19090))
        results.append(dict(round=i+1,seconds=round(time.monotonic()-begin,2),runtime=runtime['state'],explicit_running_test=bool(i==0 and a.exercise_running)))
        print(json.dumps(results[-1]),flush=True)
    if a.config_transactions:
        rolled=call('config.rollback',dict(base_revision=call('config.status')['source_revision'],target_revision=base))
        deadline=time.monotonic()+30
        while True:
            op=call('config.operation',dict(operation_id=rolled['operation_id']))
            if op['state'] not in ('accepted','running'):break
            assert time.monotonic()<deadline;time.sleep(.1)
        assert op['state']=='succeeded',op
        assert call('config.status')['source_revision']==base
        config_evidence.append(dict(stage='explicit_rollback_stopped',operation=op,status=call('config.status')))
    (root/'result.json').write_text(json.dumps(dict(passed=True,rounds=results,config_transactions=config_evidence),indent=2))
finally:
    try:
        snap=call('snapshot')
        if snap['software_state']!='stopped':call('stop');wait()
    except Exception as exc:print('cleanup:',exc,file=sys.stderr)
    proc.terminate();proc.wait(timeout=10)
    # Test-harness cleanup only: this script hardcodes mock and never issues
    # init/enable/resume. Do not use this bypass in the production daemon.
    from manager import Backend, identity
    state_path=root/'state/state.json'
    if state_path.exists():
        state=json.loads(state_path.read_text())
        backend=Backend(config,profiles,root/'state')
        for resource in reversed(state['resources']):
            if not a.exercise_running and resource.get('pid') and identity(resource['pid'])==resource.get('identity'):
                backend.stop(resource)
    log.close()
