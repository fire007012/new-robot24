#!/usr/bin/env python3
"""Convert the archived six-axis bus map into ros2_canopen configuration."""

import argparse
import math
from pathlib import Path

import yaml


AXES = {
    "left_front_arm_joint": (1, 9, 19857408),
    "right_front_arm_joint": (2, 9, 19857408),
    "left_rear_arm_joint": (3, 9, 19857408),
    "right_rear_arm_joint": (4, 9, 19857408),
    "shoulder_yaw_joint": (6, 7, 6619136),
    "shoulder_pitch_joint": (5, 7, 6619136),
}


def require(condition, detail):
    if not condition:
        raise ValueError(detail)


def mapping(pdos):
    return {
        int(number): {
            "enabled": not bool(int(pdo["cob_id"]) & 0x80000000),
            "cob_id": int(pdo["cob_id"]) & 0x7FFFFFFF,
            "transmission": pdo["transmission"],
            "mapping": [
                {"index": obj["index"], "sub_index": obj["sub_index"]}
                for obj in pdo["mapping"]
            ],
        }
        for number, pdo in pdos.items()
    }


def validate_legacy(source, joints):
    require(set(source) == {"options", "master", *AXES}, "unexpected bus nodes")
    require(source["master"]["node_id"] == 127, "master node ID changed")
    require(source["master"]["sync_period"] == 5000, "SYNC period changed")
    require(source["master"]["heartbeat_producer"] == 500, "master heartbeat changed")
    require(joints["canopen"]["interface"] == "canable0", "CAN interface changed")
    require(joints["canopen"]["master_node_id"] == 127, "joint master ID changed")
    require(joints["canopen"]["loop_hz"] == 200.0, "legacy loop rate changed")
    joint_entries = {entry["name"]: entry for entry in joints["joints"]}
    require(len(joint_entries) == len(joints["joints"]), "duplicate joint name")
    require(set(joint_entries) == set(AXES), "joint list changed")
    for name, (node_id, mode, counts_per_rev) in AXES.items():
        node = source[name]
        joint = joint_entries[name]
        require(joint["canopen"]["node_id"] == node_id, f"{name}: joint node ID changed")
        require(joint["canopen"]["default_mode"] == mode, f"{name}: default mode changed")
        require(joint["counts_per_rev"] == counts_per_rev, f"{name}: counts/rev changed")
        require(joint["ip_interpolation_period_ms"] == 5, f"{name}: IP period changed")
        require(node["node_id"] == node_id, f"{name}: node ID changed")
        require(node["heartbeat_producer"] == 500, f"{name}: heartbeat changed")
        require(node["dcf"] == "YiyouServo_V1.4.dcfgen.eds", f"{name}: EDS changed")
        require(node["sdo"] == [{"index": 0x6060, "sub_index": 0, "value": mode}],
                f"{name}: startup mode changed")
        rpdo = mapping(node["rpdo"])
        tpdo = mapping(node["tpdo"])
        require([item["index"] for item in node["rpdo"][1]["mapping"]] ==
                [0x6040, 0x607A, 0x6060], f"{name}: RPDO1 changed")
        require(node["rpdo"][2]["mapping"] ==
                [{"index": 0x60C1, "sub_index": 1, "bits": 32}], f"{name}: RPDO2 changed")
        require([item["index"] for item in node["tpdo"][1]["mapping"]] ==
                [0x6041, 0x6064, 0x6061], f"{name}: TPDO1 changed")
        require([item["index"] for item in node["tpdo"][2]["mapping"]] ==
                [0x606C, 0x6077], f"{name}: TPDO2 changed")
        require([tpdo[n]["enabled"] for n in range(1, 5)] ==
                [True, True, False, False], f"{name}: TPDO enable changed")
        require([rpdo[n]["enabled"] for n in range(1, 5)] ==
                [True, True, mode == 9, False], f"{name}: RPDO enable changed")
        require(rpdo[1]["cob_id"] == 0x200 + node_id, f"{name}: RPDO1 COB-ID changed")
        require(rpdo[2]["cob_id"] == 0x300 + node_id, f"{name}: RPDO2 COB-ID changed")
        require(tpdo[1]["cob_id"] == 0x180 + node_id, f"{name}: TPDO1 COB-ID changed")
        require(tpdo[2]["cob_id"] == 0x280 + node_id, f"{name}: TPDO2 COB-ID changed")


def build_bus(source, joints, dcf_path, axes=None):
    validate_legacy(source, joints)
    bus = {
        "options": {"dcf_path": str(dcf_path), "heartbeat_multiplier": 3.0, "retry_factor": 0},
        "master": {
            "node_id": 127,
            "driver": "ros2_canopen::MasterDriver",
            "package": "canopen_master_driver",
            "heartbeat_producer": 500,
            "sync_period": 5000,
            "sync_overflow": 0,
            "boot_timeout": 5000,
        },
    }
    selected = set(AXES if axes is None else axes)
    require(bool(selected) and selected <= set(AXES), 'invalid selected axes')
    for name, (_, mode, counts_per_rev) in AXES.items():
        if name not in selected: continue
        node = source[name]
        to_dev = counts_per_rev / (2.0 * math.pi)
        from_dev = 1.0 / to_dev
        bus[name] = {
            "node_id": node["node_id"],
            "dcf": node["dcf"],
            "driver": "ros2_canopen::Cia402Driver",
            "package": "canopen_402_driver",
            "period": 5,
            "sdo_timeout_ms": 100,
            "boot_timeout_ms": 5000,
            "non_transmit_timeout": 1000,
            "motion_safety": True,
            "managed_by_ros2_control": True,
            "perform_homing_on_init": False,
            "feedback_timeout_ms": 100,
            "ip_target_mirror": mode == 7,
            "heartbeat_producer": node["heartbeat_producer"],
            "scale_pos_to_dev": to_dev,
            "scale_pos_from_dev": from_dev,
            "scale_vel_to_dev": to_dev,
            "scale_vel_from_dev": from_dev,
            "position_mode": 7 if mode == 7 else 8,
            **({"velocity_mode": 9} if mode == 9 else {}),
            "rpdo": mapping(node["rpdo"]),
            "tpdo": mapping(node["tpdo"]),
            "sdo": [
                *node["sdo"],
                {"index": 0x60C2, "sub_index": 1, "value": 5},
            ],
        }
    return bus


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--joints", type=Path, required=True)
    parser.add_argument("--dcf-path", type=Path, required=True)
    parser.add_argument('--axes', nargs='+', choices=list(AXES))
    args = parser.parse_args()
    source = yaml.safe_load(args.source.read_text(encoding="utf-8"))
    joints = yaml.safe_load(args.joints.read_text(encoding="utf-8"))
    bus = build_bus(source, joints, args.dcf_path, args.axes)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(bus, sort_keys=False), encoding="utf-8")


if __name__ == "__main__":
    main()
