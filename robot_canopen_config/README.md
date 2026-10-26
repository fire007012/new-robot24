# robot_canopen_config

Six-axis CANopen configuration source for ROS2 Humble. This package archives the
ROS1 bus map, DCF, and both EDS variants, then generates a `ros2_canopen` `bus.yml`,
`master.dcf`, and per-node binary configuration with Lely `dcfgen`.

The archived `config/legacy_master.yaml` and `config/legacy_joints.yaml` are
source snapshots, not ROS2 runtime configurations. `scripts/generate_bus.py`
cross-checks their six node IDs, modes, counts/rev, and PDO mappings before
conversion. The generated `bus.yml` uses `dcf_path: .`, resolved against its installed
configuration directory, so the installed configuration can be relocated.
The build runs `dcfchk` and compares the common DCF sections against the archived
DCF. It also checks that each `UploadFile` is relative to the installed config
directory and that all six bin files exist.

## Build and inspect

Build in a ROS2 Humble workspace that contains the locked `ros2_canopen` source
and a working `dcfgen` executable:

```sh
colcon build --packages-up-to robot_canopen_config
colcon test --packages-select robot_canopen_config
```

Inspect the generated DCF, each node binary, and `bus.yml` under
`install/robot_canopen_config/share/robot_canopen_config/config/`. The
`urdf/canopen.ros2_control.xacro` macro is an integration fragment. It is used by the active ROS2 hardware description.

## Axis contract

| Joint | Node ID | Default mode | Position counts/rev | Command PDO |
| --- | ---: | --- | ---: | --- |
| left_front_arm_joint | 1 | CSV 9 | 19857408 | `0x60FF` |
| right_front_arm_joint | 2 | CSV 9 | 19857408 | `0x60FF` |
| left_rear_arm_joint | 3 | CSV 9 | 19857408 | `0x60FF` |
| right_rear_arm_joint | 4 | CSV 9 | 19857408 | `0x60FF` |
| shoulder_yaw_joint | 6 | IP 7 | 6619136 | `0x60C1:01` |
| shoulder_pitch_joint | 5 | IP 7 | 6619136 | `0x60C1:01` |

The four flippers also expose CSP 8 as a candidate position command mode; it
requires mode-switch testing before use. `scale_*_to_dev` is counts/rev divided
by `2*pi`, and `scale_*_from_dev` is its reciprocal. This assumes `0x606C` and
`0x60FF` are counts/s, matching the old driver. Verify that assumption,
direction, actual angle, and any device offset on each physical axis.

The startup SDO map writes `0x6060` and `0x60C2:01=5`. The latter matches the
old driver's interpolation-period write. It does not write zero, limits, or
persisted settings. Do not enable motion until the physical-device checks in
[the CANopen acceptance guide](../docs/28_CANopen软件交付与实机验收.md) are complete.
