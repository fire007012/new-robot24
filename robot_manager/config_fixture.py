#!/usr/bin/env python3
"""No ROS/device configuration fixture: NDJSON transcript, or loopback TCP server."""
import argparse
import json
from pathlib import Path
import tempfile
import time
from manager import Manager, Fault, Server

TOKEN='fixture-only-token-32-characters-minimum'


class NoDevice:
    def __init__(self): self.fail=False
    def validate(self, profile):
        if not 1<=profile.get('startup_timeout',90)<=180:raise Fault('CONFIG','invalid startup_timeout')
    def preflight(self, profile):return dict(passed=not self.fail,checks=[])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--serve',action='store_true')
    parser.add_argument('--port',type=int,default=19093)
    args=parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='robot24-config-fixture-') as folder:
        root=Path(folder);(root/'profiles').mkdir()
        (root/'profiles/mock.json').write_text(json.dumps(dict(profile_id='mock',config_revision='1',mode='mock',video=False,startup_timeout=90,build_version='fixture')))
        (root/'token').write_text(TOKEN)
        config=root/'manager.json'
        config.write_text(json.dumps(dict(enable_config_transactions=True,robot_id='robot24',config_root=str(root/'profiles'),state_dir=str(root/'state'),token_file=str(root/'token'),build_version='fixture',bind='127.0.0.1',port=args.port,control_endpoint=dict(host='127.0.0.1',port=0,protocol_version=2))))
        backend=NoDevice();manager=Manager(config,backend)
        counter=0
        def call(method,request_id=None,**params):
            nonlocal counter
            counter+=1
            request=dict(version=1,token=TOKEN,request_id=request_id or 'fixture-'+str(counter),method=method,params=params)
            print(json.dumps(dict(direction='request',frame=request)),flush=True)
            try:
                result=manager.dispatch(request)
                response=dict(version=1,request_id=request['request_id'],ok=True,result=result)
            except Fault as exc:response=dict(version=1,request_id=request['request_id'],ok=False,error=exc.data)
            print(json.dumps(dict(direction='response',frame=response)),flush=True)
            return response.get('result')
        def wait(op):
            deadline=time.monotonic()+3
            while manager.config_transactions.data['operations'][op['operation_id']]['state'] in ('accepted','running'):
                if time.monotonic()>deadline:raise RuntimeError('fixture timeout')
                time.sleep(.005)
            return call('config.operation',operation_id=op['operation_id'])
        try:
            if args.serve:
                with Server(manager) as server:
                    print(json.dumps(dict(fixture=True,host='127.0.0.1',port=server.server_address[1],token=TOKEN)),flush=True)
                    try:server.serve_forever()
                    except KeyboardInterrupt:pass
                return
            base=call('config.status')['source_revision']
            call('config.schema')
            call('config.source',profile_id='mock')
            candidate=call('config.candidate',base_revision=base,patches=[dict(profile_id='mock',field='startup_timeout',value=100)])
            call('config.preview',candidate_revision=candidate['candidate_revision'])
            call('config.status')  # Saving did not activate.
            call('config.apply',base_revision='0'*64,candidate_revision=candidate['candidate_revision'])
            applied=call('config.apply',request_id='apply-1',base_revision=base,candidate_revision=candidate['candidate_revision'])
            wait(applied)
            call('config.apply',request_id='apply-1',base_revision=base,candidate_revision=candidate['candidate_revision'])
            current=call('config.status')['source_revision']
            second=call('config.candidate',base_revision=current,patches=[dict(profile_id='mock',field='startup_timeout',value=110)])
            manager.config_transactions.hook=lambda phase: (_ for _ in ()).throw(RuntimeError('fixture apply failure')) if phase=='after_switch' else None
            wait(call('config.apply',base_revision=current,candidate_revision=second['candidate_revision']))
            manager.config_transactions.hook=lambda _:None
            wait(call('config.rollback',base_revision=current,target_revision=base))
            call('config.status');call('config.audit')
        finally:
            if manager.config_transactions.worker:manager.config_transactions.worker.join(3)
            manager.closed.set();manager.lock_file.close()

if __name__=='__main__':main()
