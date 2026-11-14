#!/usr/bin/env python3
"""PP 电机：只读识别ID，指定速度运行，自动停车；用 --help 查看参数。"""
from motor_velocity_common import main

# 修改这里设置默认值；命令行参数优先。None表示自动识别。
DEFAULT_MOTOR_ID = None
DEFAULT_DEVICE = 'canable1'
DEFAULT_SPEED = 0.2  # rad/s；负数反转
DEFAULT_DURATION = 1.0   # 秒
DEFAULT_POSITION_SCALE = 9.587379924285257e-05
DEFAULT_VELOCITY_SCALE = 9.587379924285257e-05
DEFAULT_DIRECTION = 1    # 仅允许+1或-1

if __name__ == "__main__":
    raise SystemExit(main("PP", {
        "motor_id": DEFAULT_MOTOR_ID, "device": DEFAULT_DEVICE,
        "speed": DEFAULT_SPEED, "duration": DEFAULT_DURATION,
        "position_scale": DEFAULT_POSITION_SCALE, "velocity_scale": DEFAULT_VELOCITY_SCALE,
        "direction": DEFAULT_DIRECTION,
    }))
