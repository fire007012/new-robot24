#!/usr/bin/env python3

import argparse
import os
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Dict, List

import actionlib
import rospy
import yaml
from Eyou_Canopen_Master.srv import ApplyLimits
from Eyou_Canopen_Master.srv import SetMode
from Eyou_Canopen_Master.srv import SetZero
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryActionFeedback,
    FollowJointTrajectoryGoal,
)
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint


@dataclass
class JointItem:
    name: str
    node_id: int


def normalize_action_ns(action_ns: str) -> str:
    action_ns = action_ns.strip()
    if not action_ns:
        raise ValueError("action_ns is empty")
    if not action_ns.startswith("/"):
        action_ns = "/" + action_ns
    return action_ns.rstrip("/")


def normalize_service_ns(service_ns: str) -> str:
    service_ns = service_ns.strip()
    if not service_ns:
        raise ValueError("service_ns is empty")
    if not service_ns.startswith("/"):
        service_ns = "/" + service_ns
    return service_ns.rstrip("/")


def load_joint_items(yaml_path: str) -> List[JointItem]:
    with open(yaml_path, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f)
    joints = root.get("joints")
    if not isinstance(joints, list) or not joints:
        raise ValueError("invalid joints.yaml: top-level 'joints' must be a non-empty list")

    items: List[JointItem] = []
    for idx, joint in enumerate(joints):
        if not isinstance(joint, dict):
            raise ValueError(f"invalid joints.yaml: joints[{idx}] must be a map")
        name = str(joint.get("name", f"joint_{idx + 1}"))
        canopen = joint.get("canopen") if isinstance(joint.get("canopen"), dict) else {}
        node_id = canopen.get("node_id", joint.get("node_id", idx + 1))
        items.append(JointItem(name=name, node_id=int(node_id)))
    return items


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def level_text(level: int) -> str:
    return {
        0: "OK",
        1: "WARN",
        2: "ERROR",
        3: "STALE",
    }.get(level, f"L{level}")


class JointActionUi:
    def __init__(
        self,
        root: tk.Tk,
        yaml_path: str,
        action_ns: str,
        service_ns: str,
        slider_limit: float,
        slider_resolution: float,
        goal_duration: float,
        refresh_ms: int,
    ):
        self.root = root
        self.yaml_path = yaml_path
        self.action_ns = action_ns
        self.service_ns = service_ns
        self.slider_limit = slider_limit
        self.slider_resolution = slider_resolution
        self.refresh_ms = refresh_ms
        self.goal_duration_default = goal_duration

        self.items = load_joint_items(yaml_path)
        self.joint_names = [x.name for x in self.items]
        self.joint_set = set(self.joint_names)

        self.lock = threading.Lock()
        self.action_client_gen = 0
        self.server_connected = False
        self.goal_state_text = "IDLE"
        self.service_status_text = "READY"

        self.actual: Dict[str, float] = {n: 0.0 for n in self.joint_names}
        self.target: Dict[str, float] = {n: 0.0 for n in self.joint_names}
        self.diag_summary: Dict[str, str] = {n: "STALE:no data" for n in self.joint_names}
        self.diag_all: Dict[str, str] = {n: "" for n in self.joint_names}
        self.diag_op: Dict[str, str] = {n: "-" for n in self.joint_names}
        self.diag_fault: Dict[str, str] = {n: "-" for n in self.joint_names}
        self.diag_hb: Dict[str, str] = {n: "-" for n in self.joint_names}

        self.slider_vars: Dict[str, tk.DoubleVar] = {}
        self.actual_vars: Dict[str, tk.StringVar] = {}
        self.target_vars: Dict[str, tk.StringVar] = {}
        self.error_vars: Dict[str, tk.StringVar] = {}
        self.status_vars: Dict[str, tk.StringVar] = {}
        self.diag_all_vars: Dict[str, tk.StringVar] = {}
        self.op_vars: Dict[str, tk.StringVar] = {}
        self.fault_vars: Dict[str, tk.StringVar] = {}
        self.hb_vars: Dict[str, tk.StringVar] = {}

        self.action_ns_var = tk.StringVar(value=action_ns)
        self.server_var = tk.StringVar(value="DISCONNECTED")
        self.goal_state_var = tk.StringVar(value=self.goal_state_text)
        self.service_status_var = tk.StringVar(value=self.service_status_text)
        self.duration_var = tk.StringVar(value=f"{self.goal_duration_default:.3f}")
        self.mode_joint_var = tk.StringVar(value=self.joint_names[0])
        self.mode_value_var = tk.StringVar(value="7")
        self.zero_joint_var = tk.StringVar(value=self.joint_names[0])
        self.limit_min_var = tk.StringVar(value="0.0")
        self.limit_max_var = tk.StringVar(value="0.0")
        self.use_urdf_limits_var = tk.BooleanVar(value=True)
        self.require_current_inside_limits_var = tk.BooleanVar(value=False)
        self.action_candidates: List[str] = []

        self.client = None
        self.feedback_sub = None
        self.reconnect_action_client(action_ns)
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.on_joint_state, queue_size=1)
        self.diag_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.on_diagnostics, queue_size=10)

        self.build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.refresh_action_candidates()
        self.root.after(self.refresh_ms, self.refresh_ui)

    def build_ui(self) -> None:
        self.root.title("Joint Action UI")

        top = ttk.Frame(self.root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(1, weight=1)

        ttk.Label(top, text="joints.yaml").grid(row=0, column=0, sticky="w")
        ttk.Label(top, text=self.yaml_path).grid(row=0, column=1, sticky="w")
        ttk.Label(top, text="action_ns").grid(row=1, column=0, sticky="w")
        self.action_combo = ttk.Combobox(
            top,
            width=56,
            textvariable=self.action_ns_var,
            state="normal",
        )
        self.action_combo.grid(row=1, column=1, sticky="ew")
        ttk.Button(top, text="应用", command=self.apply_action_ns).grid(row=1, column=2, sticky="w", padx=4)
        ttk.Button(top, text="刷新候选", command=self.refresh_action_candidates).grid(row=1, column=3, sticky="w", padx=4)

        ttk.Label(top, text="service_ns").grid(row=2, column=0, sticky="w")
        ttk.Label(top, text=self.service_ns).grid(row=2, column=1, sticky="w")
        ttk.Label(top, text="server").grid(row=0, column=4, sticky="w", padx=(12, 0))
        ttk.Label(top, textvariable=self.server_var).grid(row=0, column=5, sticky="w")
        ttk.Label(top, text="goal").grid(row=1, column=4, sticky="w", padx=(12, 0))
        ttk.Label(top, textvariable=self.goal_state_var).grid(row=1, column=5, sticky="w")
        ttk.Label(top, text="service").grid(row=2, column=4, sticky="w", padx=(12, 0))
        ttk.Label(top, textvariable=self.service_status_var).grid(row=2, column=5, sticky="w")

        ctrl = ttk.Frame(self.root, padding=8)
        ctrl.grid(row=1, column=0, sticky="ew")
        ttk.Label(ctrl, text="duration(s)").grid(row=0, column=0, sticky="w")
        ttk.Entry(ctrl, width=8, textvariable=self.duration_var).grid(row=0, column=1, sticky="w")
        ttk.Button(ctrl, text="目标=实际", command=self.fill_target_from_actual).grid(row=0, column=2, padx=6)
        ttk.Button(ctrl, text="发送 Action", command=self.send_goal).grid(row=0, column=3, padx=6)
        ttk.Button(ctrl, text="取消 Goal", command=self.cancel_goal).grid(row=0, column=4, padx=6)

        srv = ttk.Frame(self.root, padding=8)
        srv.grid(row=2, column=0, sticky="ew")
        ttk.Button(srv, text="init", command=lambda: self.call_trigger_service("init")).grid(row=0, column=0, padx=2)
        ttk.Button(srv, text="enable", command=lambda: self.call_trigger_service("enable")).grid(row=0, column=1, padx=2)
        ttk.Button(srv, text="disable", command=lambda: self.call_trigger_service("disable")).grid(row=0, column=2, padx=2)
        ttk.Button(srv, text="halt", command=lambda: self.call_trigger_service("halt")).grid(row=0, column=3, padx=2)
        ttk.Button(srv, text="resume", command=lambda: self.call_trigger_service("resume")).grid(row=0, column=4, padx=2)
        ttk.Button(srv, text="recover", command=lambda: self.call_trigger_service("recover")).grid(row=0, column=5, padx=2)
        ttk.Button(srv, text="shutdown", command=lambda: self.call_trigger_service("shutdown")).grid(row=0, column=6, padx=2)

        ttk.Label(srv, text="set_mode joint").grid(row=0, column=7, padx=(10, 2))
        ttk.Combobox(
            srv,
            width=10,
            textvariable=self.mode_joint_var,
            values=self.joint_names,
            state="readonly",
        ).grid(row=0, column=8, padx=2)
        ttk.Label(srv, text="mode").grid(row=0, column=9, padx=(6, 2))
        ttk.Combobox(
            srv,
            width=4,
            textvariable=self.mode_value_var,
            values=["7", "8", "9", "10"],
            state="readonly",
        ).grid(row=0, column=10, padx=2)
        ttk.Button(srv, text="set_mode", command=self.call_set_mode_service).grid(row=0, column=11, padx=(4, 0))

        zero = ttk.Frame(self.root, padding=8)
        zero.grid(row=3, column=0, sticky="ew")
        zero.columnconfigure(8, weight=1)
        ttk.Label(zero, text="zero joint").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            zero,
            width=18,
            textvariable=self.zero_joint_var,
            values=self.joint_names,
            state="readonly",
        ).grid(row=0, column=1, padx=2, sticky="w")
        ttk.Button(
            zero,
            text="当前点设零",
            command=self.call_set_zero_service,
        ).grid(row=0, column=2, padx=(6, 2), sticky="w")
        ttk.Button(
            zero,
            text="应用限位",
            command=self.call_apply_limits_service,
        ).grid(row=0, column=3, padx=(2, 2), sticky="w")
        ttk.Label(
            zero,
            text="min",
        ).grid(row=0, column=4, padx=(8, 2), sticky="w")
        ttk.Entry(zero, width=8, textvariable=self.limit_min_var).grid(
            row=0, column=5, padx=2, sticky="w"
        )
        ttk.Label(
            zero,
            text="max",
        ).grid(row=0, column=6, padx=(8, 2), sticky="w")
        ttk.Entry(zero, width=8, textvariable=self.limit_max_var).grid(
            row=0, column=7, padx=2, sticky="w"
        )
        ttk.Checkbutton(
            zero,
            text="使用URDF限位",
            variable=self.use_urdf_limits_var,
        ).grid(row=1, column=1, padx=(0, 6), sticky="w")
        ttk.Checkbutton(
            zero,
            text="要求当前点在限位内",
            variable=self.require_current_inside_limits_var,
        ).grid(row=1, column=2, columnspan=2, padx=(0, 6), sticky="w")
        ttk.Label(
            zero,
            text="调试建议：先“当前点设零”，再“应用限位”；两者都要求当前系统处于 Standby。",
        ).grid(row=1, column=4, columnspan=5, padx=(8, 0), sticky="w")

        table = ttk.Frame(self.root, padding=8)
        table.grid(row=4, column=0, sticky="nsew")
        self.root.rowconfigure(4, weight=1)
        self.root.columnconfigure(0, weight=1)

        headers = [
            "joint",
            "node_id",
            "actual(rad)",
            "target(rad)",
            "error(rad)",
            "status",
            "diag(all fields)",
            "op",
            "fault",
            "hb_lost",
            "slider(rad)",
            "value",
        ]
        for col, h in enumerate(headers):
            ttk.Label(table, text=h).grid(row=0, column=col, sticky="w", padx=2, pady=2)

        for row_idx, item in enumerate(self.items, start=1):
            name = item.name
            self.actual_vars[name] = tk.StringVar(value="0.0000")
            self.target_vars[name] = tk.StringVar(value="0.0000")
            self.error_vars[name] = tk.StringVar(value="0.0000")
            self.status_vars[name] = tk.StringVar(value="STALE:no data")
            self.diag_all_vars[name] = tk.StringVar(value="")
            self.op_vars[name] = tk.StringVar(value="-")
            self.fault_vars[name] = tk.StringVar(value="-")
            self.hb_vars[name] = tk.StringVar(value="-")
            self.slider_vars[name] = tk.DoubleVar(value=0.0)

            ttk.Label(table, text=name).grid(row=row_idx, column=0, sticky="w", padx=2, pady=2)
            ttk.Label(table, text=str(item.node_id)).grid(row=row_idx, column=1, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.actual_vars[name]).grid(row=row_idx, column=2, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.target_vars[name]).grid(row=row_idx, column=3, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.error_vars[name]).grid(row=row_idx, column=4, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.status_vars[name]).grid(row=row_idx, column=5, sticky="w", padx=2, pady=2)
            ttk.Label(
                table,
                textvariable=self.diag_all_vars[name],
                anchor="w",
                justify="left",
                wraplength=420,
            ).grid(row=row_idx, column=6, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.op_vars[name]).grid(row=row_idx, column=7, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.fault_vars[name]).grid(row=row_idx, column=8, sticky="w", padx=2, pady=2)
            ttk.Label(table, textvariable=self.hb_vars[name]).grid(row=row_idx, column=9, sticky="w", padx=2, pady=2)

            slider = tk.Scale(
                table,
                from_=-self.slider_limit,
                to=self.slider_limit,
                resolution=self.slider_resolution,
                orient=tk.HORIZONTAL,
                showvalue=False,
                variable=self.slider_vars[name],
                length=300,
            )
            slider.grid(row=row_idx, column=10, sticky="ew", padx=2, pady=2)
            ttk.Entry(table, width=10, textvariable=self.slider_vars[name]).grid(
                row=row_idx, column=11, sticky="w", padx=2, pady=2
            )

        table.columnconfigure(6, weight=1)
        table.columnconfigure(10, weight=1)

    def wait_action_server(self, gen: int, client) -> None:
        while not rospy.is_shutdown():
            if gen != self.action_client_gen:
                return
            if client.wait_for_server(rospy.Duration(0.5)):
                with self.lock:
                    if gen == self.action_client_gen:
                        self.server_connected = True
                return

    def reconnect_action_client(self, action_ns: str) -> None:
        action_ns = normalize_action_ns(action_ns)

        with self.lock:
            self.action_ns = action_ns
            self.action_ns_var.set(action_ns)
            self.server_connected = False
            self.goal_state_text = "IDLE"
            self.action_client_gen += 1
            gen = self.action_client_gen

        if self.feedback_sub is not None:
            try:
                self.feedback_sub.unregister()
            except Exception:
                pass
            self.feedback_sub = None

        client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
        self.client = client
        self.feedback_sub = rospy.Subscriber(
            f"{action_ns}/feedback",
            FollowJointTrajectoryActionFeedback,
            self.on_action_feedback,
            queue_size=10,
        )
        threading.Thread(target=self.wait_action_server, args=(gen, client), daemon=True).start()

    def refresh_action_candidates(self) -> None:
        candidates = set()
        candidates.add(normalize_action_ns(self.action_ns_var.get()))
        try:
            configured = rospy.get_param(
                "/canopen_hw_node/ip_executor_action_ns",
                "/arm_position_controller/follow_joint_trajectory",
            )
            candidates.add(normalize_action_ns(configured))
        except Exception:
            pass

        try:
            for topic_name, topic_type in rospy.get_published_topics():
                if (
                    topic_type == "control_msgs/FollowJointTrajectoryActionGoal"
                    and topic_name.endswith("/goal")
                ):
                    candidates.add(normalize_action_ns(topic_name[:-5]))
                elif (
                    topic_type == "control_msgs/FollowJointTrajectoryActionFeedback"
                    and topic_name.endswith("/feedback")
                ):
                    candidates.add(normalize_action_ns(topic_name[:-9]))
        except Exception as e:
            self.set_service_status(f"refresh topics failed: {e}")
            return

        self.action_candidates = sorted(candidates)
        self.action_combo["values"] = self.action_candidates
        self.set_service_status(f"action candidates: {len(self.action_candidates)}")

    def apply_action_ns(self) -> None:
        raw = self.action_ns_var.get().strip()
        if not raw:
            messagebox.showerror("action_ns", "action_ns must not be empty")
            return
        try:
            self.reconnect_action_client(raw)
            self.refresh_action_candidates()
            self.set_service_status(f"action_ns switched: {self.action_ns}")
        except Exception as e:
            messagebox.showerror("action_ns", str(e))

    def on_joint_state(self, msg: JointState) -> None:
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_set and i < len(msg.position):
                    self.actual[name] = msg.position[i]

    def match_joint_name(self, status_name: str) -> str:
        if status_name in self.joint_set:
            return status_name
        for name in self.joint_names:
            if status_name.endswith(name):
                return name
        return ""

    def on_diagnostics(self, msg: DiagnosticArray) -> None:
        with self.lock:
            for status in msg.status:
                name = self.match_joint_name(status.name)
                if not name:
                    continue
                kv = {x.key: x.value for x in status.values}
                self.diag_summary[name] = f"{level_text(status.level)}:{status.message}"
                self.diag_all[name] = ", ".join([f"{k}={v}" for k, v in kv.items()])
                self.diag_op[name] = "1" if parse_bool(kv.get("is_operational", "false")) else "0"
                self.diag_fault[name] = "1" if parse_bool(kv.get("is_fault", "false")) else "0"
                self.diag_hb[name] = "1" if parse_bool(kv.get("heartbeat_lost_flag", "false")) else "0"

    def on_action_feedback(self, msg: FollowJointTrajectoryActionFeedback) -> None:
        fb = msg.feedback
        names = list(fb.joint_names) if fb.joint_names else self.joint_names
        with self.lock:
            for i, name in enumerate(names):
                if name not in self.joint_set:
                    continue
                if i < len(fb.desired.positions):
                    self.target[name] = fb.desired.positions[i]
                if i < len(fb.actual.positions):
                    self.actual[name] = fb.actual.positions[i]

    def fill_target_from_actual(self) -> None:
        with self.lock:
            for name in self.joint_names:
                value = self.actual.get(name, 0.0)
                self.slider_vars[name].set(value)
                self.target[name] = value

    def set_goal_state(self, text: str) -> None:
        with self.lock:
            self.goal_state_text = text

    def on_goal_done(self, status: int, _result) -> None:
        status_text = {
            2: "PREEMPTED",
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            8: "RECALLED",
            9: "LOST",
        }.get(status, f"STATUS_{status}")
        self.set_goal_state(f"DONE:{status_text}")

    def on_goal_active(self) -> None:
        self.set_goal_state("ACTIVE")

    def send_goal(self) -> None:
        try:
            duration = float(self.duration_var.get())
        except ValueError:
            messagebox.showerror("invalid duration", "duration must be a number")
            return
        if duration <= 0.0:
            messagebox.showerror("invalid duration", "duration must be > 0")
            return

        if self.client is None:
            messagebox.showwarning("action server", "action client is not initialized")
            return
        if not self.server_connected:
            messagebox.showwarning("action server", f"action server not connected: {self.action_ns}")
            return

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [self.slider_vars[name].get() for name in self.joint_names]
        point.time_from_start = rospy.Duration.from_sec(duration)
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = rospy.Time.now()

        with self.lock:
            for i, name in enumerate(self.joint_names):
                self.target[name] = point.positions[i]
            self.goal_state_text = "PENDING"

        self.client.send_goal(goal, done_cb=self.on_goal_done, active_cb=self.on_goal_active)

    def cancel_goal(self) -> None:
        if self.client is not None:
            self.client.cancel_all_goals()
        self.set_goal_state("CANCEL_SENT")

    def set_service_status(self, text: str) -> None:
        with self.lock:
            self.service_status_text = text

    def _call_trigger_service_worker(self, service_name: str) -> None:
        full_name = f"{self.service_ns}/{service_name}"
        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            proxy = rospy.ServiceProxy(full_name, Trigger)
            res = proxy()
            prefix = "OK" if res.success else "FAIL"
            self.set_service_status(f"{service_name}: {prefix} {res.message}")
        except Exception as e:
            self.set_service_status(f"{service_name}: ERROR {e}")

    def call_trigger_service(self, service_name: str) -> None:
        threading.Thread(
            target=self._call_trigger_service_worker,
            args=(service_name,),
            daemon=True,
        ).start()

    def _call_set_mode_service_worker(self, axis_index: int, mode: int) -> None:
        full_name = f"{self.service_ns}/set_mode"
        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            proxy = rospy.ServiceProxy(full_name, SetMode)
            res = proxy(axis_index=axis_index, mode=mode)
            prefix = "OK" if res.success else "FAIL"
            self.set_service_status(
                f"set_mode[{axis_index}]={mode}: {prefix} {res.message}"
            )
        except Exception as e:
            self.set_service_status(f"set_mode: ERROR {e}")

    def call_set_mode_service(self) -> None:
        joint = self.mode_joint_var.get().strip()
        if joint not in self.joint_set:
            messagebox.showerror("set_mode", f"invalid joint: {joint}")
            return
        try:
            mode = int(self.mode_value_var.get().strip())
        except ValueError:
            messagebox.showerror("set_mode", "mode must be integer")
            return
        if mode not in (7, 8, 9, 10):
            messagebox.showerror("set_mode", "mode must be one of 7/8/9/10")
            return
        axis_index = self.joint_names.index(joint)
        threading.Thread(
            target=self._call_set_mode_service_worker,
            args=(axis_index, mode),
            daemon=True,
        ).start()

    def _call_set_zero_service_worker(self, axis_index: int, joint_name: str) -> None:
        full_name = f"{self.service_ns}/set_zero"
        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            proxy = rospy.ServiceProxy(full_name, SetZero)
            res = proxy(
                axis_index=axis_index,
                zero_offset_rad=0.0,
                use_current_position_as_zero=True,
            )
            prefix = "OK" if res.success else "FAIL"
            self.set_service_status(
                f"set_zero[{joint_name}/axis={axis_index}]: {prefix} {res.message} "
                f"(current={res.current_position:.4f}, zero={res.applied_zero_offset:.4f})"
            )
        except Exception as e:
            self.set_service_status(f"set_zero[{joint_name}]: ERROR {e}")

    def call_set_zero_service(self) -> None:
        joint = self.zero_joint_var.get().strip()
        if joint not in self.joint_set:
            messagebox.showerror("set_zero", f"invalid joint: {joint}")
            return
        axis_index = self.joint_names.index(joint)
        threading.Thread(
            target=self._call_set_zero_service_worker,
            args=(axis_index, joint),
            daemon=True,
        ).start()

    def _call_apply_limits_service_worker(self, axis_index: int, joint_name: str) -> None:
        full_name = f"{self.service_ns}/apply_limits"
        try:
            min_position = float(self.limit_min_var.get().strip())
            max_position = float(self.limit_max_var.get().strip())
        except ValueError:
            self.set_service_status("apply_limits: ERROR min/max must be numeric")
            return

        try:
            rospy.wait_for_service(full_name, timeout=2.0)
            proxy = rospy.ServiceProxy(full_name, ApplyLimits)
            res = proxy(
                axis_index=axis_index,
                min_position=min_position,
                max_position=max_position,
                use_urdf_limits=bool(self.use_urdf_limits_var.get()),
                require_current_inside_limits=bool(
                    self.require_current_inside_limits_var.get()
                ),
            )
            prefix = "OK" if res.success else "FAIL"
            self.set_service_status(
                f"apply_limits[{joint_name}/axis={axis_index}]: {prefix} {res.message} "
                f"(current={res.current_position:.4f}, limits=[{res.applied_min_position:.4f}, {res.applied_max_position:.4f}])"
            )
        except Exception as e:
            self.set_service_status(f"apply_limits[{joint_name}]: ERROR {e}")

    def call_apply_limits_service(self) -> None:
        joint = self.zero_joint_var.get().strip()
        if joint not in self.joint_set:
            messagebox.showerror("apply_limits", f"invalid joint: {joint}")
            return
        axis_index = self.joint_names.index(joint)
        threading.Thread(
            target=self._call_apply_limits_service_worker,
            args=(axis_index, joint),
            daemon=True,
        ).start()

    def refresh_ui(self) -> None:
        with self.lock:
            self.server_var.set("CONNECTED" if self.server_connected else "DISCONNECTED")
            self.goal_state_var.set(self.goal_state_text)
            self.service_status_var.set(self.service_status_text)

            for name in self.joint_names:
                actual = self.actual.get(name, 0.0)
                target = self.target.get(name, self.slider_vars[name].get())
                error = target - actual
                self.actual_vars[name].set(f"{actual:.4f}")
                self.target_vars[name].set(f"{target:.4f}")
                self.error_vars[name].set(f"{error:.4f}")
                self.status_vars[name].set(self.diag_summary.get(name, "STALE:no data"))
                self.diag_all_vars[name].set(self.diag_all.get(name, ""))
                self.op_vars[name].set(self.diag_op.get(name, "-"))
                self.fault_vars[name].set(self.diag_fault.get(name, "-"))
                self.hb_vars[name].set(self.diag_hb.get(name, "-"))

        if not rospy.is_shutdown():
            self.root.after(self.refresh_ms, self.refresh_ui)

    def on_close(self) -> None:
        try:
            if self.client is not None:
                self.client.cancel_all_goals()
        except Exception:
            pass
        if self.feedback_sub is not None:
            try:
                self.feedback_sub.unregister()
            except Exception:
                pass
        rospy.signal_shutdown("ui closed")
        self.root.quit()
        self.root.destroy()


def resolve_yaml_path(explicit_path: str) -> str:
    if explicit_path:
        return os.path.abspath(explicit_path)

    param_path = rospy.get_param("/canopen_hw_node/joints_path", "")
    if param_path and os.path.isfile(param_path):
        return os.path.abspath(param_path)

    default_path = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "joints.yaml")
    )
    return default_path


def resolve_action_ns(explicit_ns: str) -> str:
    if explicit_ns:
        return normalize_action_ns(explicit_ns)
    param_ns = rospy.get_param(
        "/canopen_hw_node/ip_executor_action_ns",
        "/arm_position_controller/follow_joint_trajectory",
    )
    return normalize_action_ns(param_ns)


def resolve_service_ns(explicit_ns: str) -> str:
    if explicit_ns:
        return normalize_service_ns(explicit_ns)
    return normalize_service_ns("/canopen_hw_node")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Joint trajectory action UI for Eyou_Canopen_Master."
    )
    parser.add_argument(
        "--joints-yaml",
        default="",
        help="Path to joints.yaml. Default: /canopen_hw_node/joints_path param or package config/joints.yaml",
    )
    parser.add_argument(
        "--action-ns",
        default="",
        help="FollowJointTrajectory action namespace. Default: /canopen_hw_node/ip_executor_action_ns",
    )
    parser.add_argument(
        "--service-ns",
        default="",
        help="Service namespace root. Default: /canopen_hw_node",
    )
    parser.add_argument(
        "--slider-limit",
        type=float,
        default=3.1416,
        help="Absolute slider range in radians.",
    )
    parser.add_argument(
        "--slider-resolution",
        type=float,
        default=0.001,
        help="Slider resolution in radians.",
    )
    parser.add_argument(
        "--goal-duration",
        type=float,
        default=1.0,
        help="Default time_from_start in seconds.",
    )
    parser.add_argument(
        "--refresh-ms",
        type=int,
        default=100,
        help="UI refresh period in milliseconds.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rospy.init_node("joint_action_ui", anonymous=True, disable_signals=True)

    yaml_path = resolve_yaml_path(args.joints_yaml)
    if not os.path.isfile(yaml_path):
        raise RuntimeError(f"joints.yaml not found: {yaml_path}")

    action_ns = resolve_action_ns(args.action_ns)
    service_ns = resolve_service_ns(args.service_ns)
    root = tk.Tk()
    JointActionUi(
        root=root,
        yaml_path=yaml_path,
        action_ns=action_ns,
        service_ns=service_ns,
        slider_limit=args.slider_limit,
        slider_resolution=args.slider_resolution,
        goal_duration=args.goal_duration,
        refresh_ms=max(20, args.refresh_ms),
    )
    root.mainloop()


if __name__ == "__main__":
    raise SystemExit(main())
