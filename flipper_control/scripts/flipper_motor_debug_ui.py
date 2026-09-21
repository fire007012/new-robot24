#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import re
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Dict, Optional

import rospy
from controller_manager_msgs.srv import ListControllers
from control_msgs.msg import JointJog
from diagnostic_msgs.msg import DiagnosticArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from Eyou_ROS1_Master.msg import JointRuntimeStateArray
from flipper_control.msg import FlipperControlState
from flipper_control.srv import SetControlProfile, SetLinkageMode


DEFAULT_FLIPPER_NS = "/flipper_control"
DEFAULT_HYBRID_NS = "/hybrid_motor_hw_node"
DEFAULT_CANOPEN_NS = "/canopen_hw_node"
DEFAULT_JOINT_NAMES = [
    "left_front_arm_joint",
    "right_front_arm_joint",
    "left_rear_arm_joint",
    "right_rear_arm_joint",
]
PROFILE_OPTIONS = ["csp_position", "csp_jog", "csv_velocity"]
PROFILE_LABELS = {
    "csp_position": "csp_position（位置轨迹）",
    "csp_jog": "csp_jog（连续 jog）",
    "csv_velocity": "csv_velocity（CSV 速度）",
}
LINKAGE_OPTIONS = [
    "independent",
    "left_right_mirror",
    "front_rear_sync",
    "side_pair",
    "diagonal_pair",
]
LINKAGE_LABELS = {
    "independent": "独立",
    "left_right_mirror": "左右镜像",
    "front_rear_sync": "前后同步",
    "side_pair": "同侧联动",
    "diagonal_pair": "对角联动",
}
LIFECYCLE_SERVICES = (
    "init",
    "enable",
    "disable",
    "halt",
    "resume",
    "recover",
    "shutdown",
)
LIFECYCLE_SERVICE_LABELS = {
    "init": "初始化",
    "enable": "使能",
    "disable": "去使能",
    "halt": "停止",
    "resume": "恢复运行",
    "recover": "恢复",
    "shutdown": "关闭通信",
}
SERVICE_SUCCESS_STATES = {
    "init": "Armed",
    "enable": "Armed",
    "disable": "Standby",
    "halt": "Armed",
    "resume": "Running",
    "recover": "Standby",
    "shutdown": "Configured",
}
SERVICE_MESSAGE_STATES = (
    ("already running", "Running"),
    ("already enabled", "Armed"),
    ("already initialized", "Armed"),
    ("already halted", "Armed"),
    ("already disabled", "Standby"),
    ("initialized (armed)", "Armed"),
    ("enabled (armed)", "Armed"),
    ("disabled (standby)", "Standby"),
    ("recovered (standby)", "Standby"),
    ("communication stopped", "Configured"),
)
CANNOT_TRANSITION_RE = re.compile(r"cannot transition from ([A-Za-z]+) to", re.I)


def ensure_absolute_name(name: str) -> str:
    if not name:
        return ""
    normalized = name.strip()
    if not normalized:
        return ""
    if not normalized.startswith("/"):
        normalized = "/" + normalized
    if len(normalized) > 1:
        normalized = normalized.rstrip("/")
    return normalized


def parse_boolish(value: object) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def level_text(level: int) -> str:
    return {
        0: "正常",
        1: "警告",
        2: "错误",
        3: "过期",
    }.get(level, f"L{level}")


def canonical_lifecycle_name(value: str) -> str:
    normalized = re.sub(r"[^a-z]", "", value.strip().lower())
    return {
        "inactive": "Inactive",
        "configured": "Configured",
        "standby": "Standby",
        "armed": "Armed",
        "running": "Running",
        "faulted": "Faulted",
        "recovering": "Recovering",
        "shuttingdown": "ShuttingDown",
        "unknown": "Unknown",
    }.get(normalized, value.strip() or "Unknown")


def profile_display_name(value: str) -> str:
    return PROFILE_LABELS.get(value, value)


def profile_internal_name(value: str) -> str:
    for internal_name, display_name in PROFILE_LABELS.items():
        if value == display_name:
            return internal_name
    return value


def linkage_display_name(value: str) -> str:
    return LINKAGE_LABELS.get(value, value)


def linkage_internal_name(value: str) -> str:
    for internal_name, display_name in LINKAGE_LABELS.items():
        if value == display_name:
            return internal_name
    return value


def lifecycle_display_name(value: str) -> str:
    normalized = canonical_lifecycle_name(value)
    return {
        "Inactive": "未激活",
        "Configured": "已配置",
        "Standby": "待机",
        "Armed": "已就绪",
        "Running": "运行中",
        "Faulted": "故障",
        "Recovering": "恢复中",
        "ShuttingDown": "关闭中",
        "Unknown": "未知",
        "Mixed": "状态混合",
    }.get(normalized, value.strip() or "-")


def lifecycle_source_display_name(value: str) -> str:
    return {
        "runtime": "运行时反馈",
        "manager": "管理器",
        "diagnostics": "诊断",
        "service": "服务返回",
        "estimated:auto_startup": "自动启动估计",
        "estimated:diagnostics": "诊断估计",
        "unknown": "未知",
        "-": "-",
    }.get(value, value)


def backend_type_display_name(value: str) -> str:
    return {
        "hybrid": "混合后端",
        "canopen": "CANopen 后端",
    }.get(value, value)


def controller_state_display_name(value: str) -> str:
    return {
        "running": "运行中",
        "initialized": "已初始化",
        "stopped": "已停止",
        "waiting": "等待中",
        "aborted": "已中止",
    }.get(value, value)


def hardware_mode_display_name(value: str) -> str:
    return {
        "position": "位置",
        "velocity": "速度",
        "-": "-",
    }.get(value, value)


def bool_text(value: bool) -> str:
    return "是" if value else "否"


def field_display_name(value: str) -> str:
    return {
        "traj_time": "轨迹时长",
        "jog_duration": "jog 时长",
    }.get(value, value)


@dataclass
class CanopenDiagnosticState:
    summary: str = "过期:无数据"
    operational: bool = False
    fault: bool = False
    heartbeat_lost: bool = False


class FlipperMotorDebugUi:
    def __init__(
        self,
        flipper_ns: str,
        hybrid_ns: str,
        canopen_ns: str,
        backend_type_override: str,
    ):
        self.flipper_ns = ensure_absolute_name(flipper_ns)
        self.hybrid_ns = ensure_absolute_name(hybrid_ns)
        self.canopen_ns = ensure_absolute_name(canopen_ns)

        configured_backend = str(
            rospy.get_param(self.flipper_ns + "/backend_type", "hybrid")
        ).strip().lower()
        requested_backend = backend_type_override.strip().lower()
        if requested_backend in {"hybrid", "canopen"}:
            self.backend_type = requested_backend
        elif configured_backend in {"hybrid", "canopen"}:
            self.backend_type = configured_backend
        else:
            self.backend_type = "hybrid"

        self.command_topic = self.flipper_ns + "/command"
        self.jog_topic = self.flipper_ns + "/jog_cmd"
        self.profile_service = self.flipper_ns + "/set_control_profile"
        self.linkage_service = self.flipper_ns + "/set_linkage_mode"
        self.state_topic = self.flipper_ns + "/state"
        self.runtime_state_topic = ensure_absolute_name(
            str(
                rospy.get_param(
                    self.flipper_ns + "/runtime_state_topic",
                    self.hybrid_ns + "/joint_runtime_states",
                )
            )
        )
        self.canopen_diagnostics_topic = ensure_absolute_name(
            str(
                rospy.get_param(
                    self.flipper_ns + "/canopen_diagnostics_topic",
                    "/diagnostics",
                )
            )
        )
        self.controller_manager_ns = ensure_absolute_name(
            str(
                rospy.get_param(
                    self.flipper_ns + "/controller_manager_ns",
                    "/controller_manager",
                )
            )
        )
        self.backend_service_ns = (
            self.hybrid_ns if self.backend_type == "hybrid" else self.canopen_ns
        )

        joint_names = rospy.get_param(
            self.flipper_ns + "/joint_names", DEFAULT_JOINT_NAMES
        )
        if not isinstance(joint_names, list) or not joint_names:
            joint_names = list(DEFAULT_JOINT_NAMES)
        self.joint_names = [str(name) for name in joint_names]

        self.controllers = {
            "joint_state_controller": "joint_state_controller",
            "csp": str(
                rospy.get_param(
                    self.flipper_ns + "/controllers/csp", "flipper_csp_controller"
                )
            ),
            "csv": str(
                rospy.get_param(
                    self.flipper_ns + "/controllers/csv",
                    "flipper_csv_forward_controller",
                )
            ),
        }

        self.canopen_auto_init = parse_boolish(
            rospy.get_param(self.canopen_ns + "/auto_init", False)
        )
        self.canopen_auto_enable = parse_boolish(
            rospy.get_param(self.canopen_ns + "/auto_enable", False)
        )
        self.canopen_auto_release = parse_boolish(
            rospy.get_param(self.canopen_ns + "/auto_release", False)
        )

        self.state_lock = threading.Lock()
        self.flipper_state: Optional[FlipperControlState] = None
        self.runtime_states: Dict[str, object] = {}
        self.canopen_diagnostics: Dict[str, CanopenDiagnosticState] = {}
        self.controller_states: Dict[str, str] = {}
        self.status_text = "就绪"
        self.lifecycle_estimate = ""
        self.lifecycle_source = "unknown"
        self.controller_poll_inflight = False
        self.last_controller_poll_monotonic = 0.0
        self.controller_poll_interval_sec = 1.0
        self.closed = False
        self.position_slider_seeded = False
        self.position_slider_dirty = False
        self.last_jog_slider_nonzero_command = False
        self.last_position_slider_command = None

        self.command_timeout_sec = self.parse_float_param(
            self.flipper_ns + "/command_timeout", 0.4
        )
        self.slider_limits = self.load_slider_limits()
        self.position_stream_period_ms = 100
        self.jog_stream_period_ms = int(
            max(50.0, min(100.0, self.command_timeout_sec * 250.0))
        )

        self.trajectory_pub = rospy.Publisher(
            self.command_topic, JointTrajectory, queue_size=10
        )
        self.jog_pub = rospy.Publisher(self.jog_topic, JointJog, queue_size=10)

        self.state_sub = rospy.Subscriber(
            self.state_topic, FlipperControlState, self.on_flipper_state, queue_size=1
        )
        self.runtime_sub = None
        self.diagnostics_sub = None
        if self.backend_type == "hybrid":
            self.runtime_sub = rospy.Subscriber(
                self.runtime_state_topic,
                JointRuntimeStateArray,
                self.on_runtime_state,
                queue_size=1,
            )
        else:
            self.diagnostics_sub = rospy.Subscriber(
                self.canopen_diagnostics_topic,
                DiagnosticArray,
                self.on_diagnostics,
                queue_size=10,
            )

        self.root = tk.Tk()
        self.root.title("摆臂电机调试界面")
        self.root.geometry("1760x980")
        self.root.minsize(1200, 720)
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.profile_value_var = tk.StringVar(
            value=profile_display_name(PROFILE_OPTIONS[0])
        )
        self.linkage_value_var = tk.StringVar(
            value=linkage_display_name(LINKAGE_OPTIONS[0])
        )
        self.trajectory_time_var = tk.StringVar(value="1.0")
        self.jog_duration_var = tk.StringVar(value="0.10")
        self.smooth_trajectory_var = tk.BooleanVar(value=True)
        self.auto_send_position_slider_var = tk.BooleanVar(value=True)
        self.auto_stream_position_var = tk.BooleanVar(value=False)
        self.auto_stream_jog_var = tk.BooleanVar(value=True)
        self.status_var = tk.StringVar(value=self.status_text)

        self.backend_type_var = tk.StringVar(value=self.backend_type)
        self.backend_ns_var = tk.StringVar(value=self.backend_service_ns)
        self.active_profile_var = tk.StringVar(value="-")
        self.active_hardware_mode_var = tk.StringVar(value="-")
        self.active_controller_var = tk.StringVar(value="-")
        self.active_linkage_var = tk.StringVar(value="-")
        self.switch_state_var = tk.StringVar(value="-")
        self.lifecycle_var = tk.StringVar(value="-")
        self.lifecycle_source_var = tk.StringVar(value="-")
        self.ready_var = tk.StringVar(value="-")
        self.switching_var = tk.StringVar(value="-")
        self.timeout_var = tk.StringVar(value="-")
        self.degraded_var = tk.StringVar(value="-")
        self.detail_var = tk.StringVar(value="-")
        self.backend_detail_var = tk.StringVar(value="-")
        self.joint_state_controller_var = tk.StringVar(value="-")
        self.csp_controller_state_var = tk.StringVar(value="-")
        self.csv_controller_state_var = tk.StringVar(value="-")

        self.position_entry_vars = {}
        self.velocity_entry_vars = {}
        self.position_slider_vars = {}
        self.velocity_slider_vars = {}
        self.position_slider_value_vars = {}
        self.velocity_slider_value_vars = {}
        self.measured_vars = {}
        self.reference_vars = {}
        self.commanded_vel_vars = {}
        self.online_vars = {}
        self.enabled_vars = {}
        self.fault_vars = {}
        self.heartbeat_vars = {}
        self.runtime_lifecycle_vars = {}

        self.build_ui()
        self.root.after(100, self.refresh_ui)
        self.root.after(
            self.position_stream_period_ms, self.process_position_slider_stream
        )
        self.root.after(self.jog_stream_period_ms, self.process_jog_slider_stream)

    def build_ui(self) -> None:
        container = ttk.Frame(self.root)
        container.grid(row=0, column=0, sticky="nsew")
        container.columnconfigure(0, weight=1)
        container.rowconfigure(0, weight=1)

        self.scroll_canvas = tk.Canvas(container, highlightthickness=0, borderwidth=0)
        self.scroll_canvas.grid(row=0, column=0, sticky="nsew")
        self.scrollbar = ttk.Scrollbar(
            container, orient=tk.VERTICAL, command=self.scroll_canvas.yview
        )
        self.scrollbar.grid(row=0, column=1, sticky="ns")
        self.scroll_canvas.configure(yscrollcommand=self.scrollbar.set)

        self.scroll_content = ttk.Frame(self.scroll_canvas)
        self.scroll_content.columnconfigure(0, weight=1)
        self.scroll_window_id = self.scroll_canvas.create_window(
            (0, 0), window=self.scroll_content, anchor="nw"
        )
        self.scroll_content.bind("<Configure>", self.on_scroll_content_configure)
        self.scroll_canvas.bind("<Configure>", self.on_scroll_canvas_configure)
        self.root.bind_all("<MouseWheel>", self.on_mousewheel, add="+")
        self.root.bind_all("<Button-4>", self.on_mousewheel, add="+")
        self.root.bind_all("<Button-5>", self.on_mousewheel, add="+")

        main = ttk.Frame(self.scroll_content, padding=10)
        main.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)
        main.rowconfigure(4, weight=1)

        summary = ttk.LabelFrame(main, text="运行总览", padding=8)
        summary.grid(row=0, column=0, sticky="ew")
        for column in range(6):
            summary.columnconfigure(column, weight=1)

        summary_rows = [
            ("后端", self.backend_type_var),
            ("后端命名空间", self.backend_ns_var),
            ("当前模式", self.active_profile_var),
            ("硬件模式", self.active_hardware_mode_var),
            ("当前控制器", self.active_controller_var),
            ("联动模式", self.active_linkage_var),
            ("切换状态", self.switch_state_var),
            ("生命周期", self.lifecycle_var),
            ("生命周期来源", self.lifecycle_source_var),
            ("就绪", self.ready_var),
            ("切换中", self.switching_var),
            ("命令超时", self.timeout_var),
            ("降级", self.degraded_var),
        ]
        for index, (label, value_var) in enumerate(summary_rows):
            row = index // 3
            column = (index % 3) * 2
            ttk.Label(summary, text=label).grid(row=row, column=column, sticky="w")
            ttk.Label(summary, textvariable=value_var).grid(
                row=row, column=column + 1, padx=(4, 12), sticky="w"
            )

        ttk.Label(summary, text="详情").grid(row=5, column=0, sticky="w")
        ttk.Label(summary, textvariable=self.detail_var).grid(
            row=5, column=1, columnspan=5, sticky="w"
        )

        backend = ttk.LabelFrame(main, text="后端操作", padding=8)
        backend.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        for column in range(8):
            backend.columnconfigure(column, weight=1)

        ttk.Label(backend, text="joint_state 控制器").grid(row=0, column=0, sticky="w")
        ttk.Label(backend, textvariable=self.joint_state_controller_var).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(backend, text="csp 控制器").grid(row=0, column=2, sticky="w")
        ttk.Label(backend, textvariable=self.csp_controller_state_var).grid(
            row=0, column=3, sticky="w"
        )
        ttk.Label(backend, text="csv 控制器").grid(row=0, column=4, sticky="w")
        ttk.Label(backend, textvariable=self.csv_controller_state_var).grid(
            row=0, column=5, sticky="w"
        )
        ttk.Button(
            backend,
            text="刷新控制器",
            command=self.refresh_controller_states,
        ).grid(row=0, column=6, padx=(8, 0), sticky="w")

        for index, service_name in enumerate(LIFECYCLE_SERVICES):
            ttk.Button(
                backend,
                text=LIFECYCLE_SERVICE_LABELS.get(service_name, service_name),
                command=lambda name=service_name: self.call_lifecycle_service(name),
            ).grid(row=1, column=index, padx=2, pady=(8, 0), sticky="w")

        ttk.Label(backend, text="后端详情").grid(
            row=2, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Label(backend, textvariable=self.backend_detail_var).grid(
            row=2, column=1, columnspan=7, sticky="w", pady=(8, 0)
        )

        control = ttk.LabelFrame(main, text="模式控制", padding=8)
        control.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        for column in range(8):
            control.columnconfigure(column, weight=1)

        ttk.Label(control, text="模式").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            control,
            textvariable=self.profile_value_var,
            values=[profile_display_name(value) for value in PROFILE_OPTIONS],
            state="readonly",
            width=18,
        ).grid(row=0, column=1, sticky="w")
        ttk.Button(
            control,
            text="切换模式",
            command=self.on_switch_profile,
        ).grid(row=0, column=2, padx=(6, 18), sticky="w")

        ttk.Label(control, text="联动").grid(row=0, column=3, sticky="w")
        ttk.Combobox(
            control,
            textvariable=self.linkage_value_var,
            values=[linkage_display_name(value) for value in LINKAGE_OPTIONS],
            state="readonly",
            width=18,
        ).grid(row=0, column=4, sticky="w")
        ttk.Button(
            control,
            text="切换联动",
            command=self.on_switch_linkage,
        ).grid(row=0, column=5, padx=(6, 18), sticky="w")

        ttk.Label(control, text="轨迹时长").grid(
            row=1, column=0, sticky="w", pady=(8, 0)
        )
        ttk.Entry(control, textvariable=self.trajectory_time_var, width=10).grid(
            row=1, column=1, sticky="w", pady=(8, 0)
        )
        ttk.Label(control, text="jog 时长").grid(
            row=1, column=3, sticky="w", pady=(8, 0)
        )
        ttk.Entry(control, textvariable=self.jog_duration_var, width=10).grid(
            row=1, column=4, sticky="w", pady=(8, 0)
        )
        ttk.Checkbutton(
            control,
            text="平滑轨迹（终点速度/加速度归零）",
            variable=self.smooth_trajectory_var,
        ).grid(row=1, column=5, columnspan=3, sticky="w", pady=(8, 0))

        joints = ttk.LabelFrame(main, text="关节命令", padding=8)
        joints.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        for column in range(11):
            joints.columnconfigure(column, weight=1)

        headers = [
            "关节",
            "在线",
            "使能",
            "故障",
            "心跳丢失",
            "实测",
            "参考",
            "命令速度",
            "生命周期",
            "目标位置",
            "目标速度",
        ]
        for column, header in enumerate(headers):
            ttk.Label(joints, text=header).grid(
                row=0, column=column, padx=4, pady=(0, 6), sticky="w"
            )

        for row, joint_name in enumerate(self.joint_names, start=1):
            self.measured_vars[joint_name] = tk.StringVar(value="-")
            self.reference_vars[joint_name] = tk.StringVar(value="-")
            self.commanded_vel_vars[joint_name] = tk.StringVar(value="-")
            self.online_vars[joint_name] = tk.StringVar(value="-")
            self.enabled_vars[joint_name] = tk.StringVar(value="-")
            self.fault_vars[joint_name] = tk.StringVar(value="-")
            self.heartbeat_vars[joint_name] = tk.StringVar(value="-")
            self.runtime_lifecycle_vars[joint_name] = tk.StringVar(value="-")
            self.position_entry_vars[joint_name] = tk.StringVar(value="")
            self.velocity_entry_vars[joint_name] = tk.StringVar(value="")

            ttk.Label(joints, text=joint_name).grid(row=row, column=0, sticky="w", padx=4)
            ttk.Label(joints, textvariable=self.online_vars[joint_name]).grid(
                row=row, column=1, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.enabled_vars[joint_name]).grid(
                row=row, column=2, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.fault_vars[joint_name]).grid(
                row=row, column=3, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.heartbeat_vars[joint_name]).grid(
                row=row, column=4, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.measured_vars[joint_name]).grid(
                row=row, column=5, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.reference_vars[joint_name]).grid(
                row=row, column=6, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.commanded_vel_vars[joint_name]).grid(
                row=row, column=7, sticky="w", padx=4
            )
            ttk.Label(joints, textvariable=self.runtime_lifecycle_vars[joint_name]).grid(
                row=row, column=8, sticky="w", padx=4
            )
            ttk.Entry(
                joints,
                textvariable=self.position_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=9, sticky="ew", padx=4)
            ttk.Entry(
                joints,
                textvariable=self.velocity_entry_vars[joint_name],
                width=12,
            ).grid(row=row, column=10, sticky="ew", padx=4)

        sliders = ttk.LabelFrame(main, text="滑块调试", padding=8)
        sliders.grid(row=4, column=0, sticky="nsew", pady=(10, 0))
        sliders.columnconfigure(0, weight=1)
        sliders.rowconfigure(1, weight=1)

        pos_sliders = ttk.LabelFrame(sliders, text="csp_position 滑块", padding=8)
        pos_sliders.grid(row=0, column=0, sticky="ew")
        pos_sliders.columnconfigure(1, weight=1)
        ttk.Checkbutton(
            pos_sliders,
            text="松手自动发送",
            variable=self.auto_send_position_slider_var,
        ).grid(row=0, column=0, sticky="w")
        ttk.Checkbutton(
            pos_sliders,
            text="拖动时连续发送",
            variable=self.auto_stream_position_var,
            command=self.on_position_stream_toggle,
        ).grid(row=0, column=1, padx=(12, 0), sticky="w")
        ttk.Button(
            pos_sliders,
            text="读取实测值到滑块",
            command=self.on_sync_position_sliders_from_measured,
        ).grid(row=0, column=2, padx=(12, 8), sticky="w")
        ttk.Button(
            pos_sliders,
            text="立即发送位置滑块",
            command=self.on_send_position_sliders,
        ).grid(row=0, column=3, sticky="w")

        ttk.Label(pos_sliders, text="关节").grid(
            row=1, column=0, padx=4, pady=(8, 4), sticky="w"
        )
        ttk.Label(pos_sliders, text="滑块").grid(
            row=1, column=1, padx=4, pady=(8, 4), sticky="w"
        )
        ttk.Label(pos_sliders, text="数值").grid(
            row=1, column=2, padx=4, pady=(8, 4), sticky="w"
        )
        ttk.Label(pos_sliders, text="范围").grid(
            row=1, column=3, padx=4, pady=(8, 4), sticky="w"
        )

        jog_sliders = ttk.LabelFrame(
            sliders, text="csp_jog / csv_velocity 滑块", padding=8
        )
        jog_sliders.grid(row=1, column=0, sticky="nsew", pady=(10, 0))
        jog_sliders.columnconfigure(1, weight=1)
        ttk.Checkbutton(
            jog_sliders,
            text="滑块非零时连续发送",
            variable=self.auto_stream_jog_var,
        ).grid(row=0, column=0, sticky="w")
        ttk.Button(
            jog_sliders,
            text="立即发送速度滑块",
            command=self.on_send_jog_sliders,
        ).grid(row=0, column=1, padx=(12, 8), sticky="w")
        ttk.Button(
            jog_sliders,
            text="速度滑块归零",
            command=self.on_zero_jog_sliders,
        ).grid(row=0, column=2, sticky="w")

        ttk.Label(jog_sliders, text="关节").grid(
            row=1, column=0, padx=4, pady=(8, 4), sticky="w"
        )
        ttk.Label(jog_sliders, text="滑块").grid(
            row=1, column=1, padx=4, pady=(8, 4), sticky="w"
        )
        ttk.Label(jog_sliders, text="数值").grid(
            row=1, column=2, padx=4, pady=(8, 4), sticky="w"
        )
        ttk.Label(jog_sliders, text="范围").grid(
            row=1, column=3, padx=4, pady=(8, 4), sticky="w"
        )

        for row, joint_name in enumerate(self.joint_names, start=2):
            limits = self.slider_limits.get(joint_name, {})
            min_position = float(limits.get("min_position", -2.356))
            max_position = float(limits.get("max_position", 2.356))
            max_velocity = float(limits.get("max_velocity", 1.57))
            initial_position = min(max(0.0, min_position), max_position)

            self.position_slider_vars[joint_name] = tk.DoubleVar(value=initial_position)
            self.position_slider_value_vars[joint_name] = tk.StringVar(
                value=self.format_number(initial_position)
            )
            self.velocity_slider_vars[joint_name] = tk.DoubleVar(value=0.0)
            self.velocity_slider_value_vars[joint_name] = tk.StringVar(
                value=self.format_number(0.0)
            )

            ttk.Label(pos_sliders, text=joint_name).grid(
                row=row, column=0, padx=4, sticky="w"
            )
            position_scale = tk.Scale(
                pos_sliders,
                from_=min_position,
                to=max_position,
                orient=tk.HORIZONTAL,
                resolution=0.01,
                length=560,
                showvalue=False,
                highlightthickness=0,
                variable=self.position_slider_vars[joint_name],
                command=lambda raw, joint=joint_name: self.on_position_slider_change(
                    joint, raw
                ),
            )
            position_scale.grid(row=row, column=1, padx=4, sticky="ew")
            position_scale.bind(
                "<ButtonRelease-1>",
                lambda _event, joint=joint_name: self.on_position_slider_release(joint),
            )
            ttk.Label(
                pos_sliders, textvariable=self.position_slider_value_vars[joint_name]
            ).grid(row=row, column=2, padx=4, sticky="w")
            ttk.Label(
                pos_sliders,
                text=f"[{min_position:.2f}, {max_position:.2f}]",
            ).grid(row=row, column=3, padx=4, sticky="w")

            ttk.Label(jog_sliders, text=joint_name).grid(
                row=row, column=0, padx=4, sticky="w"
            )
            velocity_scale = tk.Scale(
                jog_sliders,
                from_=-max_velocity,
                to=max_velocity,
                orient=tk.HORIZONTAL,
                resolution=0.01,
                length=560,
                showvalue=False,
                highlightthickness=0,
                variable=self.velocity_slider_vars[joint_name],
                command=lambda raw, joint=joint_name: self.on_velocity_slider_change(
                    joint, raw
                ),
            )
            velocity_scale.grid(row=row, column=1, padx=4, sticky="ew")
            ttk.Label(
                jog_sliders, textvariable=self.velocity_slider_value_vars[joint_name]
            ).grid(row=row, column=2, padx=4, sticky="w")
            ttk.Label(
                jog_sliders,
                text=f"[-{max_velocity:.2f}, {max_velocity:.2f}]",
            ).grid(row=row, column=3, padx=4, sticky="w")

        actions = ttk.Frame(main, padding=(0, 10, 0, 0))
        actions.grid(row=5, column=0, sticky="ew")
        ttk.Button(
            actions,
            text="发送位置轨迹",
            command=self.on_send_trajectory,
        ).grid(row=0, column=0, padx=(0, 8), sticky="w")
        ttk.Button(
            actions,
            text="发送速度 jog",
            command=self.on_send_jog,
        ).grid(row=0, column=1, padx=(0, 8), sticky="w")
        ttk.Label(actions, textvariable=self.status_var).grid(row=0, column=2, sticky="w")

    def run(self) -> None:
        self.root.mainloop()

    def on_scroll_content_configure(self, _event: tk.Event) -> None:
        self.scroll_canvas.configure(scrollregion=self.scroll_canvas.bbox("all"))

    def on_scroll_canvas_configure(self, event: tk.Event) -> None:
        self.scroll_canvas.itemconfigure(self.scroll_window_id, width=event.width)

    def on_mousewheel(self, event: tk.Event) -> None:
        steps = 0
        if getattr(event, "delta", 0):
            steps = -1 if event.delta > 0 else 1
        elif getattr(event, "num", None) == 4:
            steps = -1
        elif getattr(event, "num", None) == 5:
            steps = 1

        if steps != 0:
            self.scroll_canvas.yview_scroll(3 * steps, "units")

    def on_close(self) -> None:
        self.closed = True
        try:
            self.root.unbind_all("<MouseWheel>")
            self.root.unbind_all("<Button-4>")
            self.root.unbind_all("<Button-5>")
        except Exception:
            pass
        try:
            if self.state_sub is not None:
                self.state_sub.unregister()
        except Exception:
            pass
        try:
            if self.runtime_sub is not None:
                self.runtime_sub.unregister()
        except Exception:
            pass
        try:
            if self.diagnostics_sub is not None:
                self.diagnostics_sub.unregister()
        except Exception:
            pass
        rospy.signal_shutdown("ui closed")
        try:
            self.root.quit()
            self.root.destroy()
        except Exception:
            pass

    def set_status(self, text: str) -> None:
        with self.state_lock:
            self.status_text = text

    def parse_float_param(self, name: str, default: float) -> float:
        try:
            value = float(rospy.get_param(name, default))
        except (TypeError, ValueError):
            return default
        if not math.isfinite(value):
            return default
        return value

    def load_slider_limits(self) -> Dict[str, Dict[str, float]]:
        fallback_min_position = self.parse_float_param(
            self.flipper_ns + "/fallback_min_position", -2.356
        )
        fallback_max_position = self.parse_float_param(
            self.flipper_ns + "/fallback_max_position", 2.356
        )
        fallback_max_velocity = abs(
            self.parse_float_param(self.flipper_ns + "/fallback_max_velocity", 1.57)
        )

        direction_corrections_param = rospy.get_param(
            self.flipper_ns + "/direction_corrections", {}
        )
        direction_corrections = {}
        for joint_name in self.joint_names:
            correction = 1.0
            if (
                isinstance(direction_corrections_param, dict)
                and joint_name in direction_corrections_param
            ):
                try:
                    correction = float(direction_corrections_param[joint_name])
                except (TypeError, ValueError):
                    correction = 1.0
            if not math.isfinite(correction) or abs(correction) < 1e-9:
                correction = 1.0
            direction_corrections[joint_name] = correction

        limits = {
            joint_name: {
                "min_position": fallback_min_position,
                "max_position": fallback_max_position,
                "max_velocity": fallback_max_velocity,
            }
            for joint_name in self.joint_names
        }

        robot_description = str(rospy.get_param("/robot_description", "") or "")
        if not robot_description:
            return limits

        try:
            root = ET.fromstring(robot_description)
        except ET.ParseError:
            return limits

        for joint_element in root.findall(".//joint"):
            joint_name = str(joint_element.get("name", ""))
            if joint_name not in limits:
                continue

            limit_element = joint_element.find("limit")
            if limit_element is None:
                continue

            direction = direction_corrections.get(joint_name, 1.0)
            joint_type = str(joint_element.get("type", "")).strip().lower()

            lower_raw = limit_element.get("lower")
            upper_raw = limit_element.get("upper")
            if (
                joint_type != "continuous"
                and lower_raw is not None
                and upper_raw is not None
            ):
                try:
                    lower = float(lower_raw) / direction
                    upper = float(upper_raw) / direction
                except (TypeError, ValueError, ZeroDivisionError):
                    lower = limits[joint_name]["min_position"]
                    upper = limits[joint_name]["max_position"]
                limits[joint_name]["min_position"] = min(lower, upper)
                limits[joint_name]["max_position"] = max(lower, upper)

            velocity_raw = limit_element.get("velocity")
            if velocity_raw is None:
                continue
            try:
                hardware_max_velocity = float(velocity_raw)
            except (TypeError, ValueError):
                continue
            if hardware_max_velocity > 0.0:
                limits[joint_name]["max_velocity"] = (
                    hardware_max_velocity / abs(direction)
                )

        return limits

    @staticmethod
    def clamp_value(value: float, lower: float, upper: float) -> float:
        return max(lower, min(value, upper))

    def current_active_profile(self) -> str:
        with self.state_lock:
            flipper_state = self.flipper_state
        if flipper_state is not None and flipper_state.active_profile:
            return str(flipper_state.active_profile)
        return profile_internal_name(self.profile_value_var.get().strip())

    def position_slider_profile_active(self) -> bool:
        return self.current_active_profile() == "csp_position"

    def jog_slider_profile_active(self) -> bool:
        return self.current_active_profile() in {"csp_jog", "csv_velocity"}

    def set_position_slider_value(self, joint_name: str, value: float) -> None:
        limits = self.slider_limits.get(joint_name, {})
        clamped = self.clamp_value(
            value,
            float(limits.get("min_position", -2.356)),
            float(limits.get("max_position", 2.356)),
        )
        self.position_slider_vars[joint_name].set(clamped)
        self.position_entry_vars[joint_name].set(self.format_number(clamped))
        self.position_slider_value_vars[joint_name].set(self.format_number(clamped))

    def set_velocity_slider_value(self, joint_name: str, value: float) -> None:
        limits = self.slider_limits.get(joint_name, {})
        max_velocity = abs(float(limits.get("max_velocity", 1.57)))
        clamped = self.clamp_value(value, -max_velocity, max_velocity)
        self.velocity_slider_vars[joint_name].set(clamped)
        self.velocity_entry_vars[joint_name].set(self.format_number(clamped))
        self.velocity_slider_value_vars[joint_name].set(self.format_number(clamped))

    def on_position_slider_change(self, joint_name: str, raw_value: str) -> None:
        value = float(raw_value)
        self.position_entry_vars[joint_name].set(self.format_number(value))
        self.position_slider_value_vars[joint_name].set(self.format_number(value))
        self.position_slider_dirty = True

    def on_velocity_slider_change(self, joint_name: str, raw_value: str) -> None:
        value = float(raw_value)
        self.velocity_entry_vars[joint_name].set(self.format_number(value))
        self.velocity_slider_value_vars[joint_name].set(self.format_number(value))

    def collect_position_slider_targets(self) -> tuple[list[str], list[float]]:
        names = list(self.joint_names)
        positions = [self.position_slider_vars[name].get() for name in names]
        return names, positions

    def collect_velocity_slider_targets(self) -> tuple[list[str], list[float]]:
        names = list(self.joint_names)
        velocities = [self.velocity_slider_vars[name].get() for name in names]
        return names, velocities

    @staticmethod
    def positions_snapshot(values: list[float]) -> tuple[float, ...]:
        return tuple(round(value, 4) for value in values)

    def mark_position_slider_command_sent(self, positions: list[float]) -> None:
        self.last_position_slider_command = self.positions_snapshot(positions)
        self.position_slider_dirty = False

    def publish_position_trajectory(
        self,
        names: list[str],
        positions: list[float],
        duration: float,
        *,
        update_status: bool = True,
        status_text: Optional[str] = None,
    ) -> None:
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = names
        point = JointTrajectoryPoint()
        point.positions = positions
        if self.smooth_trajectory_var.get():
            # Give JTC explicit endpoint derivatives so it can use cubic/quintic
            # interpolation instead of falling back to position-only linear motion.
            point.velocities = [0.0] * len(names)
            point.accelerations = [0.0] * len(names)
        point.time_from_start = rospy.Duration.from_sec(duration)
        traj.points = [point]
        self.trajectory_pub.publish(traj)
        if update_status:
            if status_text is None:
                if self.smooth_trajectory_var.get():
                    status_text = "已发送位置轨迹（终点速度/加速度归零）"
                else:
                    status_text = "已发送位置轨迹"
            self.set_status(status_text)

    def publish_velocity_jog(
        self,
        names: list[str],
        velocities: list[float],
        duration: float,
        *,
        update_status: bool = True,
        status_text: str = "已发送速度 jog",
    ) -> None:
        msg = JointJog()
        msg.header.stamp = rospy.Time.now()
        msg.joint_names = names
        msg.velocities = velocities
        msg.duration = duration
        self.jog_pub.publish(msg)
        if update_status:
            self.set_status(status_text)

    def on_position_slider_release(self, _joint_name: str) -> None:
        if not self.auto_send_position_slider_var.get():
            return
        if not self.position_slider_profile_active():
            self.set_status("位置滑块已就绪，请先切换到 csp_position")
            return
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return
        names, positions = self.collect_position_slider_targets()
        self.publish_position_trajectory(
            names,
            positions,
            duration,
            status_text="已发送位置滑块轨迹",
        )
        self.mark_position_slider_command_sent(positions)

    def on_position_stream_toggle(self) -> None:
        if self.auto_stream_position_var.get():
            self.position_slider_dirty = True

    def on_sync_position_sliders_from_measured(self) -> None:
        with self.state_lock:
            flipper_state = self.flipper_state
        if flipper_state is None:
            self.set_status("暂未收到实测位置")
            return

        measured = dict(zip(flipper_state.joint_names, flipper_state.measured_positions))
        updated = 0
        for joint_name in self.joint_names:
            if joint_name not in measured:
                continue
            self.set_position_slider_value(joint_name, measured[joint_name])
            updated += 1

        if updated == 0:
            self.set_status("暂未收到实测位置")
            return

        self.position_slider_seeded = True
        self.position_slider_dirty = False
        self.last_position_slider_command = self.positions_snapshot(
            [self.position_slider_vars[name].get() for name in self.joint_names]
        )
        self.set_status("已将实测位置同步到滑块")

    def on_send_position_sliders(self) -> None:
        if not self.position_slider_profile_active():
            self.set_status("位置滑块需要在 csp_position 模式下使用")
            return
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return
        names, positions = self.collect_position_slider_targets()
        self.publish_position_trajectory(
            names,
            positions,
            duration,
            status_text="已发送位置滑块",
        )
        self.mark_position_slider_command_sent(positions)

    def process_position_slider_stream(self) -> None:
        if self.closed or rospy.is_shutdown():
            return

        try:
            if self.auto_stream_position_var.get() and self.position_slider_profile_active():
                if self.position_slider_dirty:
                    duration = self.parse_duration(
                        self.trajectory_time_var.get(), "traj_time"
                    )
                    names, positions = self.collect_position_slider_targets()
                    snapshot = self.positions_snapshot(positions)
                    if snapshot != self.last_position_slider_command:
                        self.publish_position_trajectory(
                            names,
                            positions,
                            duration,
                            update_status=False,
                        )
                        self.mark_position_slider_command_sent(positions)
                        self.set_status("位置滑块连续发送中")
        except ValueError:
            pass
        finally:
            if not self.closed and not rospy.is_shutdown():
                self.root.after(
                    self.position_stream_period_ms, self.process_position_slider_stream
                )

    def on_send_jog_sliders(self) -> None:
        if not self.jog_slider_profile_active():
            self.set_status("速度滑块需要在 csp_jog/csv_velocity 模式下使用")
            return
        try:
            duration = self.parse_duration(self.jog_duration_var.get(), "jog_duration")
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return
        names, velocities = self.collect_velocity_slider_targets()
        self.publish_velocity_jog(
            names,
            velocities,
            duration,
            status_text="已发送速度滑块",
        )

    def on_zero_jog_sliders(self) -> None:
        for joint_name in self.joint_names:
            self.set_velocity_slider_value(joint_name, 0.0)
        try:
            duration = self.parse_duration(self.jog_duration_var.get(), "jog_duration")
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return
        names, velocities = self.collect_velocity_slider_targets()
        self.publish_velocity_jog(
            names,
            velocities,
            duration,
            status_text="已将 jog 滑块归零",
        )
        self.last_jog_slider_nonzero_command = False

    def process_jog_slider_stream(self) -> None:
        if self.closed or rospy.is_shutdown():
            return

        try:
            if self.auto_stream_jog_var.get():
                duration = self.parse_duration(
                    self.jog_duration_var.get(), "jog_duration"
                )
                names, velocities = self.collect_velocity_slider_targets()
                has_nonzero = any(abs(value) > 1e-3 for value in velocities)
                if has_nonzero and self.jog_slider_profile_active():
                    self.publish_velocity_jog(
                        names, velocities, duration, update_status=False
                    )
                    self.last_jog_slider_nonzero_command = True
                elif (
                    not has_nonzero
                    and self.last_jog_slider_nonzero_command
                    and self.jog_slider_profile_active()
                ):
                    self.publish_velocity_jog(
                        names,
                        [0.0] * len(names),
                        duration,
                        update_status=False,
                    )
                    self.last_jog_slider_nonzero_command = False
                    self.set_status("已发送 jog 停止命令")
                elif not has_nonzero:
                    self.last_jog_slider_nonzero_command = False
        except ValueError:
            pass
        finally:
            if not self.closed and not rospy.is_shutdown():
                self.root.after(self.jog_stream_period_ms, self.process_jog_slider_stream)

    def apply_lifecycle_observation(self, state: Optional[str], source: str) -> None:
        if not state or state == "Unknown":
            return
        with self.state_lock:
            self.lifecycle_estimate = state
            self.lifecycle_source = source

    def infer_lifecycle_state_from_service_result(
        self, service_name: str, success: bool, message: str
    ) -> Optional[str]:
        if success:
            return SERVICE_SUCCESS_STATES.get(service_name)

        lower = message.lower()
        for pattern, state in SERVICE_MESSAGE_STATES:
            if pattern in lower:
                return state

        match = CANNOT_TRANSITION_RE.search(message)
        if match:
            return canonical_lifecycle_name(match.group(1))

        if "faulted" in lower:
            return "Faulted"
        if "recovering" in lower:
            return "Recovering"
        return None

    def lifecycle_service_name(self, service_name: str) -> str:
        return self.backend_service_ns + "/" + service_name

    def call_lifecycle_service(self, service_name: str) -> None:
        threading.Thread(
            target=self._call_lifecycle_service_worker,
            args=(service_name,),
            daemon=True,
        ).start()

    def _call_lifecycle_service_worker(self, service_name: str) -> None:
        full_name = self.lifecycle_service_name(service_name)
        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            proxy = rospy.ServiceProxy(full_name, Trigger)
            response = proxy()
            lifecycle = self.infer_lifecycle_state_from_service_result(
                service_name, bool(response.success), response.message
            )
            if lifecycle is not None:
                self.apply_lifecycle_observation(lifecycle, "service")
            prefix = "成功" if response.success else "失败"
            display_name = LIFECYCLE_SERVICE_LABELS.get(service_name, service_name)
            self.set_status(f"{display_name}：{prefix} {response.message}")
        except Exception as exc:
            display_name = LIFECYCLE_SERVICE_LABELS.get(service_name, service_name)
            self.set_status(f"{display_name}：错误 {exc}")

    def on_switch_profile(self) -> None:
        threading.Thread(
            target=self._call_profile_service_worker,
            args=(self.profile_value_var.get(),),
            daemon=True,
        ).start()

    def _call_profile_service_worker(self, profile: str) -> None:
        try:
            rospy.wait_for_service(self.profile_service, timeout=2.0)
            proxy = rospy.ServiceProxy(self.profile_service, SetControlProfile)
            internal_profile = profile_internal_name(profile)
            response = proxy(profile=internal_profile)
            prefix = "成功" if response.success else "失败"
            self.set_status(
                f"切换模式到 {internal_profile}：{prefix} {response.message}"
            )
        except Exception as exc:
            internal_profile = profile_internal_name(profile)
            self.set_status(f"切换模式到 {internal_profile}：错误 {exc}")

    def on_switch_linkage(self) -> None:
        threading.Thread(
            target=self._call_linkage_service_worker,
            args=(self.linkage_value_var.get(),),
            daemon=True,
        ).start()

    def _call_linkage_service_worker(self, linkage: str) -> None:
        try:
            rospy.wait_for_service(self.linkage_service, timeout=2.0)
            proxy = rospy.ServiceProxy(self.linkage_service, SetLinkageMode)
            internal_linkage = linkage_internal_name(linkage)
            response = proxy(mode=internal_linkage)
            prefix = "成功" if response.success else "失败"
            self.set_status(
                f"切换联动到 {linkage_display_name(internal_linkage)}：{prefix} {response.message}"
            )
        except Exception as exc:
            internal_linkage = linkage_internal_name(linkage)
            self.set_status(
                f"切换联动到 {linkage_display_name(internal_linkage)}：错误 {exc}"
            )

    def refresh_controller_states(self) -> None:
        self._maybe_poll_controller_states(force=True)

    def _maybe_poll_controller_states(self, force: bool = False) -> None:
        now = time.monotonic()
        with self.state_lock:
            if self.controller_poll_inflight:
                return
            if (
                not force
                and (now - self.last_controller_poll_monotonic)
                < self.controller_poll_interval_sec
            ):
                return
            self.controller_poll_inflight = True
            self.last_controller_poll_monotonic = now

        threading.Thread(
            target=self._poll_controller_states_worker,
            daemon=True,
        ).start()

    def _poll_controller_states_worker(self) -> None:
        try:
            full_name = self.controller_manager_ns + "/list_controllers"
            rospy.wait_for_service(full_name, timeout=1.0)
            proxy = rospy.ServiceProxy(full_name, ListControllers)
            response = proxy()
            updated = {item.name: item.state for item in response.controller}
            with self.state_lock:
                self.controller_states = updated
        except Exception:
            pass
        finally:
            with self.state_lock:
                self.controller_poll_inflight = False

    def parse_duration(self, value: str, label: str) -> float:
        try:
            duration = float(value)
        except ValueError as exc:
            raise ValueError(f"{field_display_name(label)}必须是数字") from exc
        if duration <= 0.0:
            raise ValueError(f"{field_display_name(label)}必须大于 0")
        return duration

    def on_send_trajectory(self) -> None:
        try:
            duration = self.parse_duration(self.trajectory_time_var.get(), "traj_time")
            names = []
            positions = []
            for joint_name in self.joint_names:
                raw = self.position_entry_vars[joint_name].get().strip()
                if not raw:
                    continue
                names.append(joint_name)
                positions.append(float(raw))
            if not names:
                raise ValueError("请至少填写一个目标位置")
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return

        self.publish_position_trajectory(names, positions, duration)

    def on_send_jog(self) -> None:
        try:
            duration = self.parse_duration(self.jog_duration_var.get(), "jog_duration")
            names = []
            velocities = []
            for joint_name in self.joint_names:
                raw = self.velocity_entry_vars[joint_name].get().strip()
                if not raw:
                    continue
                names.append(joint_name)
                velocities.append(float(raw))
            if not names:
                raise ValueError("请至少填写一个目标速度")
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))
            return

        self.publish_velocity_jog(names, velocities, duration)

    def on_flipper_state(self, msg: FlipperControlState) -> None:
        with self.state_lock:
            self.flipper_state = msg
        lifecycle = canonical_lifecycle_name(msg.lifecycle_state)
        if lifecycle and lifecycle != "Unknown":
            self.apply_lifecycle_observation(lifecycle, "manager")

    def on_runtime_state(self, msg: JointRuntimeStateArray) -> None:
        updated = {}
        first_lifecycle = ""
        for state in msg.states:
            updated[state.joint_name] = state
            if not first_lifecycle and state.lifecycle_state:
                first_lifecycle = canonical_lifecycle_name(state.lifecycle_state)
        with self.state_lock:
            self.runtime_states = updated
        if first_lifecycle and first_lifecycle != "Unknown":
            self.apply_lifecycle_observation(first_lifecycle, "runtime")

    def match_joint_name(self, status_name: str) -> str:
        for joint_name in self.joint_names:
            if status_name == joint_name:
                return joint_name
            if status_name.endswith(joint_name):
                return joint_name
        return ""

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        updated: Dict[str, CanopenDiagnosticState] = {}
        for status in msg.status:
            joint_name = self.match_joint_name(status.name)
            if not joint_name:
                continue

            kv = {item.key: item.value for item in status.values}
            updated[joint_name] = CanopenDiagnosticState(
                summary=f"{level_text(status.level)}:{status.message}",
                operational=parse_boolish(kv.get("is_operational", "false")),
                fault=parse_boolish(kv.get("is_fault", "false")),
                heartbeat_lost=parse_boolish(kv.get("heartbeat_lost_flag", "false")),
            )

        if not updated:
            return

        with self.state_lock:
            merged = dict(self.canopen_diagnostics)
            merged.update(updated)
            self.canopen_diagnostics = merged

        if any(item.fault or item.heartbeat_lost for item in updated.values()):
            self.apply_lifecycle_observation("Faulted", "diagnostics")

    @staticmethod
    def format_number(value: float) -> str:
        return f"{value:.4f}"

    def summarize_runtime_states(self, runtime_states: Dict[str, object]) -> str:
        if not runtime_states:
            return "等待 joint_runtime_states 话题"

        fault_joints = sorted(
            joint_name
            for joint_name, state in runtime_states.items()
            if getattr(state, "fault", False)
        )
        offline_joints = sorted(
            joint_name
            for joint_name, state in runtime_states.items()
            if not getattr(state, "online", False)
        )
        if fault_joints:
            return "故障: " + ", ".join(fault_joints)
        if offline_joints:
            return "离线: " + ", ".join(offline_joints)
        return "运行时状态正常"

    def summarize_canopen_diagnostics(
        self, diagnostics: Dict[str, CanopenDiagnosticState]
    ) -> str:
        if not diagnostics:
            return "等待 diagnostics 话题"

        fault_joints = sorted(
            joint_name for joint_name, state in diagnostics.items() if state.fault
        )
        heartbeat_joints = sorted(
            joint_name for joint_name, state in diagnostics.items() if state.heartbeat_lost
        )
        not_operational_joints = sorted(
            joint_name
            for joint_name, state in diagnostics.items()
            if not state.operational and not state.fault and not state.heartbeat_lost
        )
        if fault_joints:
            return "故障: " + ", ".join(fault_joints)
        if heartbeat_joints:
            return "心跳丢失: " + ", ".join(heartbeat_joints)
        if not_operational_joints:
            return "未运行: " + ", ".join(not_operational_joints)
        return "全部运行正常"

    def resolve_hybrid_lifecycle(
        self,
        flipper_state: Optional[FlipperControlState],
        runtime_states: Dict[str, object],
        estimate: str,
        estimate_source: str,
    ) -> tuple[str, str, str]:
        lifecycles = sorted(
            {
                canonical_lifecycle_name(getattr(state, "lifecycle_state", ""))
                for state in runtime_states.values()
                if getattr(state, "lifecycle_state", "")
            }
        )
        lifecycles = [state for state in lifecycles if state != "Unknown"]
        if len(lifecycles) == 1:
            return lifecycles[0], "runtime", self.summarize_runtime_states(runtime_states)
        if len(lifecycles) > 1:
            return "Mixed", "runtime", self.summarize_runtime_states(runtime_states)

        if flipper_state is not None:
            lifecycle = canonical_lifecycle_name(flipper_state.lifecycle_state)
            if lifecycle and lifecycle != "Unknown":
                return lifecycle, "manager", self.summarize_runtime_states(runtime_states)

        if estimate:
            return estimate, estimate_source, self.summarize_runtime_states(runtime_states)

        return "-", "-", self.summarize_runtime_states(runtime_states)

    def resolve_canopen_lifecycle(
        self,
        flipper_state: Optional[FlipperControlState],
        diagnostics: Dict[str, CanopenDiagnosticState],
        estimate: str,
        estimate_source: str,
    ) -> tuple[str, str, str]:
        detail = self.summarize_canopen_diagnostics(diagnostics)
        if any(item.fault or item.heartbeat_lost for item in diagnostics.values()):
            return "Faulted", "diagnostics", detail

        if flipper_state is not None:
            lifecycle = canonical_lifecycle_name(flipper_state.lifecycle_state)
            if lifecycle and lifecycle != "Unknown":
                return lifecycle, "manager", detail

        if estimate:
            return estimate, estimate_source, detail

        if diagnostics:
            if self.canopen_auto_release:
                return "Running", "estimated:auto_startup", detail
            if self.canopen_auto_enable or self.canopen_auto_init:
                return "Armed", "estimated:auto_startup", detail
            return "Unknown", "estimated:diagnostics", detail

        if not self.canopen_auto_init:
            return "Configured", "estimated:auto_startup", detail
        if self.canopen_auto_release:
            return "Running", "estimated:auto_startup", detail
        return "Armed", "estimated:auto_startup", detail

    def refresh_ui(self) -> None:
        self._maybe_poll_controller_states()

        with self.state_lock:
            flipper_state = self.flipper_state
            runtime_states = dict(self.runtime_states)
            canopen_diagnostics = dict(self.canopen_diagnostics)
            controller_states = dict(self.controller_states)
            status_text = self.status_text
            estimate = self.lifecycle_estimate
            estimate_source = self.lifecycle_source

        if self.backend_type == "hybrid":
            lifecycle, lifecycle_source, backend_detail = self.resolve_hybrid_lifecycle(
                flipper_state, runtime_states, estimate, estimate_source
            )
        else:
            lifecycle, lifecycle_source, backend_detail = self.resolve_canopen_lifecycle(
                flipper_state, canopen_diagnostics, estimate, estimate_source
            )

        self.status_var.set(status_text)
        self.backend_type_var.set(backend_type_display_name(self.backend_type))
        self.backend_ns_var.set(self.backend_service_ns)
        self.lifecycle_var.set(lifecycle_display_name(lifecycle))
        self.lifecycle_source_var.set(lifecycle_source_display_name(lifecycle_source))
        self.backend_detail_var.set(backend_detail)
        self.joint_state_controller_var.set(
            controller_state_display_name(
                controller_states.get(self.controllers["joint_state_controller"], "-")
            )
        )
        self.csp_controller_state_var.set(
            controller_state_display_name(
                controller_states.get(self.controllers["csp"], "-")
            )
        )
        self.csv_controller_state_var.set(
            controller_state_display_name(
                controller_states.get(self.controllers["csv"], "-")
            )
        )

        if flipper_state is not None:
            self.active_profile_var.set(
                profile_display_name(flipper_state.active_profile or "-")
            )
            self.active_hardware_mode_var.set(
                hardware_mode_display_name(flipper_state.active_hardware_mode or "-")
            )
            self.active_controller_var.set(flipper_state.active_controller or "-")
            self.active_linkage_var.set(
                linkage_display_name(flipper_state.linkage_mode or "-")
            )
            self.switch_state_var.set(flipper_state.switch_state or "-")
            self.ready_var.set(bool_text(flipper_state.ready))
            self.switching_var.set(bool_text(flipper_state.switching))
            self.timeout_var.set(bool_text(flipper_state.command_timed_out))
            self.degraded_var.set(bool_text(flipper_state.degraded))
            self.detail_var.set(flipper_state.detail or "-")

            measured = dict(
                zip(flipper_state.joint_names, flipper_state.measured_positions)
            )
            reference = dict(
                zip(flipper_state.joint_names, flipper_state.reference_positions)
            )
            commanded_vel = dict(
                zip(flipper_state.joint_names, flipper_state.commanded_velocities)
            )
        else:
            self.active_profile_var.set("-")
            self.active_hardware_mode_var.set("-")
            self.active_controller_var.set("-")
            self.active_linkage_var.set("-")
            self.switch_state_var.set("-")
            self.ready_var.set("-")
            self.switching_var.set("-")
            self.timeout_var.set("-")
            self.degraded_var.set("-")
            self.detail_var.set("-")
            measured = {}
            reference = {}
            commanded_vel = {}

        if measured and not self.position_slider_seeded:
            for joint_name in self.joint_names:
                if joint_name in measured:
                    self.set_position_slider_value(joint_name, measured[joint_name])
            self.position_slider_seeded = True
            self.position_slider_dirty = False
            self.last_position_slider_command = self.positions_snapshot(
                [self.position_slider_vars[name].get() for name in self.joint_names]
            )

        for joint_name in self.joint_names:
            if joint_name in measured:
                self.measured_vars[joint_name].set(
                    self.format_number(measured[joint_name])
                )
            else:
                self.measured_vars[joint_name].set("-")

            if joint_name in reference:
                self.reference_vars[joint_name].set(
                    self.format_number(reference[joint_name])
                )
            else:
                self.reference_vars[joint_name].set("-")

            if joint_name in commanded_vel:
                self.commanded_vel_vars[joint_name].set(
                    self.format_number(commanded_vel[joint_name])
                )
            else:
                self.commanded_vel_vars[joint_name].set("-")

            if self.backend_type == "hybrid":
                runtime = runtime_states.get(joint_name)
                if runtime is None:
                    self.online_vars[joint_name].set("-")
                    self.enabled_vars[joint_name].set("-")
                    self.fault_vars[joint_name].set("-")
                    self.heartbeat_vars[joint_name].set("-")
                    self.runtime_lifecycle_vars[joint_name].set("-")
                else:
                    self.online_vars[joint_name].set(bool_text(runtime.online))
                    self.enabled_vars[joint_name].set(bool_text(runtime.enabled))
                    self.fault_vars[joint_name].set(bool_text(runtime.fault))
                    self.heartbeat_vars[joint_name].set("-")
                    self.runtime_lifecycle_vars[joint_name].set(
                        lifecycle_display_name(runtime.lifecycle_state)
                    )
                continue

            diagnostic = canopen_diagnostics.get(joint_name)
            if diagnostic is None:
                self.online_vars[joint_name].set("-")
                self.enabled_vars[joint_name].set("-")
                self.fault_vars[joint_name].set("-")
                self.heartbeat_vars[joint_name].set("-")
                self.runtime_lifecycle_vars[joint_name].set(lifecycle)
                continue

            enabled = (
                lifecycle in {"Armed", "Running"}
                and diagnostic.operational
                and not diagnostic.fault
                and not diagnostic.heartbeat_lost
            )
            self.online_vars[joint_name].set(bool_text(diagnostic.operational))
            self.enabled_vars[joint_name].set(bool_text(enabled))
            self.fault_vars[joint_name].set(bool_text(diagnostic.fault))
            self.heartbeat_vars[joint_name].set(bool_text(diagnostic.heartbeat_lost))
            self.runtime_lifecycle_vars[joint_name].set(
                lifecycle_display_name(lifecycle)
            )

        if not self.closed and not rospy.is_shutdown():
            self.root.after(200, self.refresh_ui)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="摆臂电机调试界面")
    parser.add_argument(
        "--flipper-ns",
        default=DEFAULT_FLIPPER_NS,
        help="flipper_control 话题与服务的命名空间。",
    )
    parser.add_argument(
        "--hybrid-ns",
        default=DEFAULT_HYBRID_NS,
        help="混合后端服务与运行时话题的命名空间。",
    )
    parser.add_argument(
        "--canopen-ns",
        default=DEFAULT_CANOPEN_NS,
        help="CANopen 后端服务的命名空间。",
    )
    parser.add_argument(
        "--backend-type",
        default="auto",
        choices=["auto", "hybrid", "canopen"],
        help="覆盖后端类型。默认从 flipper_control/backend_type 读取。",
    )
    return parser.parse_known_args()[0]


def main() -> None:
    args = parse_args()
    rospy.init_node("flipper_motor_debug_ui", anonymous=True, disable_signals=True)
    ui = FlipperMotorDebugUi(
        args.flipper_ns,
        args.hybrid_ns,
        args.canopen_ns,
        args.backend_type,
    )
    ui.run()


if __name__ == "__main__":
    main()
