# 2026-03-21 DCF/URDF 修复记录

## DCF/EDS 修复

- 文件: `config/YiyouServo_V1.4.dcfgen.eds`
- 删除不兼容段: `[DynamicChannels]`
- 修复 `UNSIGNED16` 上限越界:
  - `[2023]` `HighLimit: 65536 -> 65535`
  - `[2025]` `HighLimit: 65536 -> 65535`
- 同步产品码字段:
  - `[DeviceInfo] ProductNumber = 0x00221701`
  - `[1018sub2] DefaultValue = 0x00221701`

## 重新生成产物

- `config/master.dcf`
- `config/master.bin`
- `config/joint_1.bin`

## URDF 修复

- 文件: `urdf/robot.urdf`
- 从仅 `joint_1` 扩展为 `joint_1~joint_6`，以匹配
  `config/ros/controllers.yaml` 中 `arm_position_controller` / `arm_velocity_controller`
  的关节列表，修复控制器加载时报错：
  `Could not find joint 'joint_2' in URDF model.`
