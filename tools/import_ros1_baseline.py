#!/usr/bin/env python3
"""Import user-confirmed ROS1 parameters; require actual readback for device identity.

No ROS/CAN I/O and no device parameter writes. Without --snapshot the output
remains uncommissioned. A snapshot is the JSON message from /robot_runtime/snapshot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[1]
IDENTITY={'1000:00','1018:01','1018:02','1018:03','1018:04','100A:00'}
PERSISTENT={'607C:00','2003:00','607D:01','607D:02'}
SOURCE='reference/ros1/Eyou_Canopen_Master/config/joints.yaml'


def build(snapshot=None):
    legacy=yaml.safe_load((ROOT/SOURCE).read_text())
    current=yaml.safe_load((ROOT/'robot_canopen_config/config/legacy_joints.yaml').read_text())
    if legacy!=current:raise ValueError('ROS2 CANopen parameters differ from confirmed ROS1 source; reconcile before import')
    baseline=yaml.safe_load((ROOT/'robot_hardware/config/commissioning.yaml').read_text())
    baseline['commissioned']=False
    baseline['parameter_reference']={
        'path':SOURCE, 'sha256':hashlib.sha256((ROOT/SOURCE).read_bytes()).hexdigest(),
        'confirmation':'user_confirmed_working_ros1_parameters',
        'identity_source':'actual_runtime_snapshot_required',
        'automatic_device_parameter_writes':False}
    joints={j['name']:j for j in legacy['joints']}
    if snapshot is not None and (not isinstance(snapshot,dict) or not isinstance(snapshot.get('axes'),dict)):
        raise ValueError('snapshot must contain axes from /robot_runtime/snapshot')
    for name,axis in baseline['axes'].items():
        expected=joints[name]['canopen']['node_id']
        if expected!=axis['node_id']:raise ValueError(name+': Node-ID differs from reference')
        axis['physical_label']='ROS1:'+name
        axis['direction_verified']=True
        axis['parameter_source']=SOURCE
        if snapshot is None:continue
        measured=snapshot['axes'].get(name,{})
        if measured.get('node_id')!=expected:raise ValueError(name+': snapshot Node-ID mismatch')
        identity=measured.get('identity',{})
        if set(identity)!=IDENTITY:raise ValueError(name+': incomplete measured identity')
        if any(type(identity[k]) is not int or not 0<=identity[k]<=0xffffffff for k in IDENTITY-{'100A:00'}):
            raise ValueError(name+': invalid identity values')
        firmware=identity['100A:00']
        if not isinstance(firmware,str) or not firmware.strip() or 'mock' in firmware.lower():
            raise ValueError(name+': missing or simulated firmware')
        if any(v==0x4d4f434b for v in identity.values()):raise ValueError('simulated identity cannot commission hardware')
        persistent=measured.get('persistent',{})
        if set(persistent)!=PERSISTENT or any(type(v) is not int or not 0<=v<=0xffffffff for v in persistent.values()):
            raise ValueError(name+': complete actual persistent readback required')
        axis['identity']=identity
        axis['persistent']=persistent
    if snapshot is not None:baseline['commissioned']=True
    return baseline


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    readback=p.add_mutually_exclusive_group()
    readback.add_argument('--snapshot',type=Path)
    readback.add_argument('--axis-readback',type=Path,nargs='+',help='six JSON exports from axis_commission (without --activate/--operation)')
    a=p.parse_args()
    snapshot=json.loads(a.snapshot.read_text()) if a.snapshot else None
    if a.axis_readback:
        snapshot={'axes':{}}
        for path in a.axis_readback:
            measured=json.loads(path.read_text())
            name=measured.get('joint')
            if name not in build()['axes'] or name in snapshot['axes']:
                raise ValueError('readbacks require unique known CANopen joints')
            snapshot['axes'][name]=measured
    # Also accept a saved JSON Trigger response, without treating a failed RPC as evidence.
    if isinstance(snapshot,dict) and 'success' in snapshot:
        if snapshot['success'] is not True:raise ValueError('snapshot service failed')
        snapshot=json.loads(snapshot['message'])
    result=build(snapshot)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:yaml.safe_dump(result,stream,allow_unicode=True,sort_keys=False)
    print(json.dumps({'output':str(a.output),'commissioned':result['commissioned'],
                      'device_io_performed':False,'identity_readback_required':snapshot is None},ensure_ascii=False))

if __name__=='__main__':main()
