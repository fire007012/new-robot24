#!/usr/bin/env python3
"""Stopped, offline deployment. Preserve state, token, emergency latch and ID journals.

Run the released copy while the management daemon is stopped. Does not start ROS.
Existing active bundles are migrated with a digest and an atomic pointer switch.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
from config_transactions import atomic
from deployment_paths import load_config
from manager import Manager


def deploy(config_path, setup, manifest, baseline, full_interfaces=None):
    config_path = Path(config_path).resolve(strict=True)
    config = load_config(config_path)
    release = json.loads(Path(manifest).read_text())
    if (release.get('management_protocol'), release.get('control_protocol')) != (1, 2):
        raise ValueError('release protocol mismatch')
    if not Path(setup).is_file():raise ValueError('setup missing')
    # flock rejects a live manager; recorded resources/operations reject unsafe migration.
    manager = Manager(config_path)
    try:
        if manager.active() or manager.recovery or manager.data['resources'] or manager.data['software_state'] != 'stopped':
            raise ValueError('stop/reconcile managed resources before deployment')
        tx = manager.config_transactions
        profiles = tx._read(tx.current()) if tx else {
            p.stem:json.loads(p.read_text()) for p in manager.root.glob('*.json')}
        before = tx.current() if tx else None
        baseline = Path(baseline).resolve(strict=True)
        digest = hashlib.sha256(baseline.read_bytes()).hexdigest()
        target = manager.root / ('commissioning-'+digest+'.yaml')
        if not target.exists():shutil.copyfile(baseline, target)
        ids = ['hardware'] + (['hardware-full'] if full_interfaces else [])
        for name in ids:
            value = json.loads((Path(__file__).parent/'config'/(name+'.json')).read_text())
            value.update(robot_id=config['robot_id'],baseline=target.name,build_version=release['build_version'])
            if name == 'hardware-full':
                a,b=full_interfaces
                value.update(can_interfaces={'canopen_interface':a,'private_interface':b},
                    devices=sorted({a,b}),group_devices={'base':[b],'gripper':[b],'flippers':[a],'arm':sorted({a,b})})
            value['config_revision'] = hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
            manager.backend.validate(value)
            profiles[name]=value
        # Match retained mock and other profiles to this release without changing their mode.
        for value in profiles.values():
            if value.get('build_version') != release['build_version']:
                value['build_version']=release['build_version']
                value['config_revision']=hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
            manager.backend.validate(value)
        revision = tx.bundle(profiles) if tx else None
        # File-based deployment revisions are per profile. Bundles rewrite all revisions.
        for name,value in profiles.items():atomic(manager.root/(name+'.json'), value)
        if tx:
            atomic(tx.pointer, {'revision':revision})
            tx.data['audit'].append({'action':'offline_can0_deployment','before_revision':before,'revision':revision})
            tx.save()
        raw = json.loads(config_path.read_text())
        raw.update(setup=os.path.relpath(Path(setup).resolve(),config_path.parent),
                   build_manifest=os.path.relpath(Path(manifest).resolve(),config_path.parent),
                   build_version=release['build_version'])
        for key in ('config_root','state_dir','token_file'):
            raw[key]=os.path.relpath(config[key], config_path.parent)
        atomic(config_path,raw)
        result={'profiles':ids,'bundle_revision':revision,'build_version':release['build_version'],
                'baseline_sha256':digest,'hardware_started':False}
        atomic(manager.state_dir/'can0-deployment.json',result)
        return result
    finally:
        manager.closed.set()
        manager.lock_file.close()


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('config','setup','manifest','baseline'):p.add_argument('--'+key,required=True,type=Path)
    p.add_argument('--full-interfaces',nargs=2,metavar=('CANOPEN','PRIVATE'))
    a=p.parse_args()
    print(json.dumps(deploy(a.config,a.setup,a.manifest,a.baseline,a.full_interfaces),ensure_ascii=False,indent=2))
