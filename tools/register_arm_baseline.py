#!/usr/bin/env python3
"""Register existing, operator-confirmed arm configuration without device I/O."""
import argparse
import copy
import hashlib
from pathlib import Path
import yaml

ARM = {'shoulder_yaw_joint': 6, 'shoulder_pitch_joint': 5}


def register(baseline, registration, bus):
    result=copy.deepcopy(baseline)
    if registration.get('registration_status')!='current_configuration_registered':
        raise ValueError('explicit current-configuration confirmation required')
    if registration.get('canopen_interface')!='can1':raise ValueError('CANopen must use can1')
    for joint,node in ARM.items():
        spec=registration['axes'][joint];entry=result['axes'][joint];configured=bus[joint]
        if any(v!=node for v in (spec['node_id'],entry['node_id'],configured['node_id'])):
            raise ValueError(joint+': Node-ID mismatch')
        if (spec['counts_per_rev']!=6619136 or spec['default_mode']!=configured['position_mode']
                or not spec['physical_label'] or spec['direction_verified'] is not True):
            raise ValueError(joint+': inconsistent operator configuration')
        # Preserve every known identity and persistent value, including unknowns.
        # This mode binds node/configuration; it does not invent device readbacks.
        entry.update(physical_label=spec['physical_label'],direction_verified=True)
        entry['parameter_registration']={
            'mode':'operator_confirmed_configuration','confirmed':True,
            'confirmed_by':'operator','date':registration['date'],
            'counts_per_rev':spec['counts_per_rev'],'position_mode':configured['position_mode'],
            'canopen_interface':'can1','identity_readback_required_before_registration':False}
    result['schema_version']='robot24-parameter-baseline-v2'
    result['parameters_verified']=True
    result['commissioned_groups']=sorted(set(result.get('commissioned_groups',[]))|{'arm','gripper'})
    result['commissioned_scope']='selected_groups_operator_confirmed_configuration'
    result.setdefault('group_registration',{}).setdefault('arm',{}).update(
        parameters_verified=True,registration_mode='operator_confirmed_configuration',
        identity_requirement='runtime_communication_checks; no prior identity snapshot required')
    result['can_interfaces']={'private_interface':'can0','canopen_interface':'can1'}
    if 'canopen_parameters' in result:
        result['canopen_parameters']['canopen']['interface']='can1'
    result.setdefault('parameter_reference',{}).update(
        identity_source='existing_values_preserved; missing_readback_not_fabricated',runtime_canopen_interface='can1')
    result['arm_registration']={
        'status':'registered','mode':'operator_confirmed_configuration','date':registration['date'],
        'axes':list(ARM),'device_io_performed':False,
        'identity_readback_required_before_registration':False,
        'device_identity_complete':all(v is not None for j in ARM for v in result['axes'][j]['identity'].values()),
        'source_sha256':registration['source_sha256']}
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('baseline','registration','bus','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();load=lambda path:yaml.safe_load(path.read_text())
    value=register(load(a.baseline),load(a.registration),load(a.bus))
    with a.output.open('x') as stream:yaml.safe_dump(value,stream,allow_unicode=True,sort_keys=False)
    print('Registered arm configuration baseline:',a.output)
    print('SHA256:',hashlib.sha256(a.output.read_bytes()).hexdigest())

if __name__=='__main__':main()
