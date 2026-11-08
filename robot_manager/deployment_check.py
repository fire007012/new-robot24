#!/usr/bin/env python3
"""Read-only deployment verification. Never installs services or starts hardware."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import stat
from client import request


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',required=True)
    parser.add_argument('--probe',action='store_true',help='query existing management API on loopback/configured host')
    args=parser.parse_args()
    checks=[]
    def check(name,ok,detail=''):checks.append(dict(name=name,ok=bool(ok),detail=detail))
    config=json.loads(Path(args.config).read_text())
    token=Path(config['token_file'])
    check('token_permissions',stat.S_IMODE(token.stat().st_mode)==0o600)
    check('token_length',len(token.read_text().strip())>=32)
    check('setup',Path(config['setup']).is_file())
    check('state_directory',Path(config['state_dir']).is_dir() and os.access(config['state_dir'],os.W_OK))
    check('config_root',Path(config['config_root']).is_dir())
    check('control_protocol',config['control_endpoint'].get('protocol_version')==2)
    check('bind_policy',config.get('bind','127.0.0.1') in ('127.0.0.1','::1') or config.get('network_policy_confirmed') is True,
          'non-loopback requires explicit deployment network_policy_confirmed=true')
    check('bridge_bind_policy',config.get('bridge_bind','127.0.0.1') in ('127.0.0.1','::1') or config.get('network_policy_confirmed') is True)
    manifest_path=Path(config['build_manifest'])
    manifest=json.loads(manifest_path.read_text())
    check('manifest_version',manifest.get('build_version')==config['build_version'] and manifest.get('management_protocol')==1 and manifest.get('control_protocol')==2)
    base=manifest_path.parent.resolve()
    for rel,digest in manifest.get('files',{}).items():
        path=(base/rel).resolve()
        check('sha256:'+rel,path.is_relative_to(base) and path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==digest)
    check('manifest_not_empty',bool(manifest.get('files')))
    if args.probe:
        host=config.get('bind','127.0.0.1')
        if host=='0.0.0.0':host='127.0.0.1'
        for method in ('hello','snapshot'):
            response=request(host,config.get('port',9091),token.read_text().strip(),method)
            check('api:'+method,response.get('ok'))
    print(json.dumps(dict(passed=all(c['ok'] for c in checks),checks=checks),ensure_ascii=False,indent=2))
    return 0 if all(c['ok'] for c in checks) else 1

if __name__=='__main__':raise SystemExit(main())
