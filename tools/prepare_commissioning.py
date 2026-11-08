#!/usr/bin/env python3
"""Create an unconfirmed field worksheet from source configuration, without device I/O."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
INPUTS = (
    'robot_hardware/config/commissioning.yaml',
    'robot_hardware/urdf/private.ros2_control.xacro',
    'robot_canopen_config/config/legacy_joints.yaml',
    'robot_safety/config/group_policy_v1.yaml',
)


def prepare(output):
    output = output.expanduser().resolve()
    if output == ROOT or ROOT in output.parents:
        raise ValueError('field records must be outside the source tree')
    policy = yaml.safe_load((ROOT / INPUTS[3]).read_text())
    baseline = yaml.safe_load((ROOT / INPUTS[0]).read_text())
    if baseline.get('commissioned') is not False:
        raise ValueError('source baseline must remain uncommissioned')
    canopen = yaml.safe_load((ROOT / INPUTS[2]).read_text())
    canopen_axes = {entry['name']: entry for entry in canopen['joints']}
    private_axes = {}
    for joint in ET.parse(ROOT / INPUTS[1]).getroot().findall('.//joint'):
        private_axes[joint.attrib['name']] = {
            p.attrib['name']: p.text for p in joint.findall('param')}
    axes = {}
    for name, ownership in policy['axes'].items():
        if ownership['backend'] == 'RobotCanopenSystem':
            configured = canopen_axes[name]
            expected = {
                'interface': canopen['canopen']['interface'], 'protocol': 'CANopen',
                'node_id': configured['canopen']['node_id'],
                'default_mode': configured['canopen']['default_mode'],
                'counts_per_rev': configured['counts_per_rev'],
            }
            if baseline['axes'][name]['node_id'] != expected['node_id']:
                raise ValueError(f'{name}: baseline Node-ID conflicts with configuration')
        else:
            configured = private_axes[name]
            expected = {
                'interface': 'canable1', 'protocol': configured['protocol'],
                'motor_id': int(configured['motor_id'], 0),
                'control_mode': configured['control_mode'],
                'direction_sign': float(configured.get('direction_sign', '1')),
            }
        axes[name] = {
            **ownership, 'configured_not_measured': expected,
            'actual': {key: None for key in (
                'model', 'serial', 'firmware', 'physical_label', 'wiring',
                'direction_evidence', 'original_parameters_path',
                'restore_method', 'readback_path', 'verified_by', 'verified_at')},
            'verified': False,
        }
    conditions = {}
    for rule in policy['associations']:
        conditions[rule['condition']] = {
            'required_source': rule['source'], 'initial': 'unknown',
            'integration_status': 'pending',
            **{key: None for key in (
                'sensor_or_controller', 'transport_and_signal',
                'trusted_measurement_rule', 'sample_rate_hz', 'validity_ms',
                'loss_or_stale_detection', 'measurement_error',
                'acceptance_evidence_path', 'verified_by', 'verified_at')},
            'negative_checks': {key: 'not_run_on_hardware' for key in (
                'missing', 'expired', 'wrong_source', 'unsatisfied')},
        }
    # Refuse reuse, including a pre-existing empty directory, to preserve field records.
    output.mkdir(parents=True, exist_ok=False)
    (output / 'raw').mkdir()
    (output / 'measurements').mkdir()
    shutil.copyfile(ROOT / INPUTS[0], output / 'commissioning.yaml')
    (output / 'inventory.yaml').write_text(yaml.safe_dump({
        'status': 'not_verified_on_hardware', 'axes': axes,
        'buses': {name: {'configured_bitrate': 1000000,
                        'actual_bitrate': None, 'unique_master_verified': False,
                        'interface_evidence_path': None}
                  for name in ('canable0', 'canable1')},
    }, sort_keys=False, allow_unicode=True))
    (output / 'safety_evidence.yaml').write_text(yaml.safe_dump({
        'status': 'worksheet_only_not_loaded_by_runtime', 'conditions': conditions,
        'field_thresholds': {key: None for key in (
            'feedback_deadlines_and_rationale', 'stopped_velocity_thresholds',
            'stop_observation_duration_ms', 'max_physical_stop_latency_ms',
            'brake_and_payload_retention_criteria', 'position_drift_limits',
            'measurement_instruments_and_error', 'clock_alignment_error_ms')},
    }, sort_keys=False, allow_unicode=True))
    manifest = {
        'created_utc': datetime.now(timezone.utc).isoformat(),
        'source_commit': subprocess.check_output(
            ['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
        'source_status': subprocess.check_output(
            ['git', '-C', str(ROOT), 'status', '--short'], text=True),
        'input_sha256': {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
                         for name in INPUTS},
        'observed_network_interfaces': sorted(p.name for p in Path('/sys/class/net').iterdir()),
        'device_io_performed': False,
    }
    (output / 'preparation.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    (output / 'README.md').write_text('''# 现场准备包（未实机核验）

本目录由 tools/prepare_commissioning.py 根据源码配置生成，不访问设备、不启动主站、不使能电机。
commissioning.yaml 仅覆盖六个 CANopen 轴，是运行时基线；inventory.yaml 登记全部 13 轴。
配置 ID、方向及波特率是期望值，actual 字段、verified 和 commissioned 必须经现场核验。
safety_evidence.yaml 是人工登记表，不会被运行时加载，填写它不会接入真实安全信号。

1. 先核对物理标签、接线、方向和唯一主站。保存 ip -details -statistics link show 的输出；
   比较两条 CAN 的实际接口、波特率和 inventory.yaml 中的源码期望值。
2. 按文档 28 逐个运行 single_axis.launch.py，另一个终端用 axis_commission JOINT，
   不传 --activate、--operation 或 --save，将原始 JSON 保存到 raw/JOINT.json。
   该流程配置主站并写易失通信配置，会发送 CAN 帧；现场确认后执行，每次只运行一个主站。
3. 将真实身份、固件、物理标签和方向填入 commissioning.yaml，并保存原始 607C:00、
   2003:00、607D:01/02、1010:01 读回；原始 uint32、有符号解释和恢复方法一起保留。
   私有 CAN 七轴按实际协议另行备份与读回，不能套用 CANopen 对象字典。
4. 真实姿态/制动、支撑、载荷信号的适配器须在控制管理器进程内写入共享 Domain，
   使用 policy 指定来源和单调时钟有效期限；当前只有 Mock 输入，没有真实适配器。
   缺失、断线、过期和不满足不能转换为 Satisfied，ROS 布尔开关不是可信证据。
5. 填写 safety_evidence.yaml 的采样频率、有效期、失效规则和物理阈值，按文档 32
   留存 measurements/ 下的外部测量及 F01–F13 记录。确认参数读回和现场核验后，
   再按文档 28 确认 commissioned；生成准备包不表示第 2 节完成。

反馈默认阈值参考文档 31，现场实测后另记调整依据；不要将默认值直接当成物理验收阈值。
''')
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='new directory outside the source tree; existing paths are refused')
    args = parser.parse_args()
    try:
        print(prepare(args.output))
    except (ValueError, FileExistsError) as exc:
        parser.exit(2, f'{exc}\n')


if __name__ == '__main__':
    main()
