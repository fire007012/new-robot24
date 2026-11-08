#!/usr/bin/env python3
"""No ROS dependency. Example: client.py --token-file TOKEN start --profile mock."""
import argparse
import json
from pathlib import Path
import socket
import uuid


def request(host, port, token, method, params=None, request_id=None):
    payload = dict(version=1, token=token, request_id=request_id or str(uuid.uuid4()), method=method, params=params or {})
    with socket.create_connection((host, port), timeout=15) as conn:
        conn.sendall((json.dumps(payload)+'\n').encode())
        return json.loads(conn.makefile('rb').readline(65536))

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--host', default='127.0.0.1')
    p.add_argument('--port', type=int, default=9091)
    p.add_argument('--token-file', required=True)
    p.add_argument('method', choices=['hello','profiles','preflight','start','stop','snapshot','operation','logs'])
    p.add_argument('--robot', default='robot24')
    p.add_argument('--profile', default='mock')
    p.add_argument('--revision', default='1')
    p.add_argument('--request-id')
    p.add_argument('--operation-id')
    p.add_argument('--cursor', type=int, default=0)
    args = p.parse_args()
    params = dict(robot_id=args.robot, profile_id=args.profile, config_revision=args.revision) if args.method in ('start','preflight') else {}
    if args.operation_id: params['operation_id'] = args.operation_id
    if args.method == 'logs': params['cursor'] = args.cursor
    print(json.dumps(request(args.host,args.port,Path(args.token_file).read_text().strip(),args.method,params,args.request_id),ensure_ascii=False,indent=2))
