#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rospy
import yaml
from Eyou_ROS1_Master.msg import JointRuntimeStateArray
from Eyou_ROS1_Master.srv import ApplyJointLimits, SetJointMode, SetJointZero


DEFAULT_SERVICE_NS = "/hybrid_motor_hw_node"
DEFAULT_ACTION_NS = "/arm_position_controller/follow_joint_trajectory"


def rospack_find(package_name: str) -> str:
    return subprocess.check_output(
        ["rospack", "find", package_name], text=True
    ).strip()


def try_rosparam_get(param_name: str) -> str:
    try:
        return subprocess.check_output(
            ["rosparam", "get", param_name], text=True, stderr=subprocess.DEVNULL
        ).strip()
    except subprocess.CalledProcessError:
        return ""
    except FileNotFoundError:
        return ""


def resolve_default_paths() -> tuple[str, str, str, str, str]:
    canopen_pkg = rospack_find("Eyou_Canopen_Master")
    master_pkg = rospack_find("Eyou_ROS1_Master")
    can_driver_pkg = rospack_find("can_driver")
    runtime_canopen_yaml = try_rosparam_get("/hybrid_motor_hw_node/canopen_joints_path")
    runtime_can_driver_yaml = try_rosparam_get("/hybrid_motor_hw_node/can_driver_config")
    runtime_mode_mapping_file = try_rosparam_get("/hybrid_motor_hw_node/joint_mode_mappings_file")
    return (
        runtime_canopen_yaml or os.path.join(canopen_pkg, "config", "joints.yaml"),
        runtime_can_driver_yaml or os.path.join(can_driver_pkg, "config", "can_driver.yaml"),
        os.path.join(canopen_pkg, "scripts", "joint_action_ui.py"),
        DEFAULT_ACTION_NS,
        runtime_mode_mapping_file or os.path.join(master_pkg, "config", "joint_mode_mappings.yaml"),
    )


def parse_motor_id(raw_value) -> int:
    if isinstance(raw_value, int):
        return raw_value
    if isinstance(raw_value, str):
        return int(raw_value, 0)
    raise ValueError(f"invalid motor_id: {raw_value!r}")


def load_yaml(path: str):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def extract_canopen_joints(root) -> list[dict]:
    joints = root.get("joints")
    if not isinstance(joints, list) or not joints:
        raise ValueError("invalid canopen joints.yaml: top-level 'joints' must be a non-empty list")

    merged = []
    for index, joint in enumerate(joints):
        if not isinstance(joint, dict):
            raise ValueError(f"invalid canopen joints[{index}]: must be a map")
        name = str(joint.get("name", f"joint_{index + 1}"))
        canopen = joint.get("canopen") if isinstance(joint.get("canopen"), dict) else {}
        node_id = canopen.get("node_id", joint.get("node_id", index + 1))
        merged.append(
            {
                "name": name,
                "node_id": int(node_id),
                "backend": "canopen",
                "axis_index": index,
            }
        )
    return merged


def extract_can_driver_joints(root) -> list[dict]:
    node_root = root.get("can_driver_node")
    if not isinstance(node_root, dict):
        raise ValueError("invalid can_driver.yaml: missing top-level 'can_driver_node'")

    joints = node_root.get("joints")
    if not isinstance(joints, list) or not joints:
        raise ValueError("invalid can_driver.yaml: 'can_driver_node.joints' must be a non-empty list")

    merged = []
    for index, joint in enumerate(joints):
        if not isinstance(joint, dict):
            raise ValueError(f"invalid can_driver joints[{index}]: must be a map")

        control_mode = str(joint.get("control_mode", "")).strip()
        if control_mode == "velocity":
            continue
        if control_mode not in {"position", "csp"}:
            raise ValueError(
                f"invalid can_driver joint '{joint.get('name', index)}': unknown control_mode '{control_mode}'"
            )

        if "name" not in joint or "motor_id" not in joint:
            raise ValueError(
                f"invalid can_driver joints[{index}]: missing name/motor_id"
            )
        merged.append(
            {
                "name": str(joint["name"]),
                "node_id": parse_motor_id(joint["motor_id"]),
                "backend": "can_driver",
                "motor_id": parse_motor_id(joint["motor_id"]),
            }
        )
    return merged


def build_merged_joint_yaml(canopen_yaml: str, can_driver_yaml: str) -> tuple[str, list[dict]]:
    canopen_joints = extract_canopen_joints(load_yaml(canopen_yaml))
    can_driver_joints = extract_can_driver_joints(load_yaml(can_driver_yaml))

    merged = []
    seen_names = set()
    for item in canopen_joints + can_driver_joints:
        if item["name"] in seen_names:
            raise ValueError(f"duplicate joint name in merged UI config: {item['name']}")
        seen_names.add(item["name"])
        merged.append(item)

    temp_file = tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", suffix=".yaml", prefix="hybrid_joint_ui_", delete=False
    )
    with temp_file:
        yaml.safe_dump({"joints": merged}, temp_file, allow_unicode=False, sort_keys=False)
    return temp_file.name, merged


def load_mode_options(mode_mapping_file: str) -> tuple[list[str], dict[str, dict]]:
    param_yaml = try_rosparam_get("/hybrid_motor_hw_node/joint_mode_mappings/modes")
    if param_yaml:
        param_modes = yaml.safe_load(param_yaml)
        if isinstance(param_modes, dict) and param_modes:
            normalized = {str(name).lower(): value for name, value in param_modes.items()}
            return list(normalized.keys()), normalized

    root = load_yaml(mode_mapping_file)
    if not isinstance(root, dict):
        raise ValueError("invalid joint_mode_mappings.yaml: root must be a map")
    mode_root = root.get("joint_mode_mappings")
    if not isinstance(mode_root, dict):
        raise ValueError("invalid joint_mode_mappings.yaml: missing joint_mode_mappings")
    modes = mode_root.get("modes")
    if not isinstance(modes, dict) or not modes:
        raise ValueError("invalid joint_mode_mappings.yaml: joint_mode_mappings.modes must be a non-empty map")
    normalized = {str(name).lower(): value for name, value in modes.items()}
    return list(normalized.keys()), normalized


def load_upstream_module(script_path: str):
    spec = importlib.util.spec_from_file_location(
        "eyou_joint_action_ui_upstream", script_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load joint_action_ui module from: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_hybrid_ui_class(upstream_module):
    class HybridJointActionUi(upstream_module.JointActionUi):
        def __init__(self, *args, joint_metadata_by_name: dict[str, dict], available_modes: list[str], temp_yaml_path: str, keep_temp: bool, **kwargs):
            self.hybrid_joint_metadata_by_name = joint_metadata_by_name
            self.available_modes = available_modes
            self.temp_yaml_path = temp_yaml_path
            self.keep_temp = keep_temp
            self.zero_joint_var = None
            self.action_joint_names = []
            self.runtime_state_by_joint = {}
            self.runtime_tree = None
            super().__init__(*args, **kwargs)
            topic_name = f"{self.service_ns}/joint_runtime_states"
            self.runtime_state_sub = rospy.Subscriber(
                topic_name, JointRuntimeStateArray, self.on_runtime_states, queue_size=1
            )

        def build_ui(self) -> None:
            super().build_ui()
            self._reconfigure_mode_controls()

            if not self.joint_names:
                return

            self.zero_joint_var = tk.StringVar(value=self.joint_names[0])
            zero = ttk.Frame(self.root, padding=8)
            zero.grid(row=4, column=0, sticky="ew")
            zero.columnconfigure(4, weight=1)

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
                command=self.call_zero_service,
            ).grid(row=0, column=2, padx=(6, 2), sticky="w")
            ttk.Button(
                zero,
                text="应用URDF限位",
                command=self.call_apply_limits_service,
            ).grid(row=0, column=3, padx=(2, 2), sticky="w")
            ttk.Label(
                zero,
                text="统一调用 set_joint_zero / apply_joint_limits",
            ).grid(row=0, column=4, padx=(8, 0), sticky="w")

            runtime = ttk.LabelFrame(self.root, text="关节生命周期状态", padding=8)
            runtime.grid(row=5, column=0, sticky="nsew")
            runtime.columnconfigure(0, weight=1)
            self.root.rowconfigure(5, weight=0)

            columns = ("joint", "backend", "lifecycle", "online", "enabled", "fault")
            tree = ttk.Treeview(runtime, columns=columns, show="headings", height=8)
            tree.heading("joint", text="joint")
            tree.heading("backend", text="backend")
            tree.heading("lifecycle", text="lifecycle")
            tree.heading("online", text="online")
            tree.heading("enabled", text="enabled")
            tree.heading("fault", text="fault")
            tree.column("joint", width=180, anchor="w")
            tree.column("backend", width=90, anchor="center")
            tree.column("lifecycle", width=120, anchor="center")
            tree.column("online", width=80, anchor="center")
            tree.column("enabled", width=80, anchor="center")
            tree.column("fault", width=80, anchor="center")
            tree.grid(row=0, column=0, sticky="ew")
            self.runtime_tree = tree

        def _reconfigure_mode_controls(self) -> None:
            if self.available_modes:
                self.mode_value_var.set(self.available_modes[0])
            target_var = str(self.mode_value_var)

            def walk(widget: tk.Misc) -> None:
                for child in widget.winfo_children():
                    try:
                        if str(child.cget("textvariable")) == target_var:
                            child.configure(values=self.available_modes)
                    except Exception:
                        pass
                    walk(child)

            walk(self.root)

        def _call_zero_service_worker(self, joint_name: str) -> None:
            try:
                full_name = f"{self.service_ns}/set_joint_zero"
                rospy.wait_for_service(full_name, timeout=2.0)
                proxy = rospy.ServiceProxy(full_name, SetJointZero)
                res = proxy(
                    joint_name=joint_name,
                    zero_offset_rad=0.0,
                    use_current_position_as_zero=True,
                )
                prefix = "OK" if res.success else "FAIL"
                self.set_service_status(
                    f"set_joint_zero[{joint_name}/{res.backend or '-'}]: {prefix} {res.message} "
                    f"(current={res.current_position_rad:.4f}, zero={res.applied_zero_offset_rad:.4f})"
                )
            except Exception as exc:
                self.set_service_status(f"set_zero[{joint_name}]: ERROR {exc}")

        def call_zero_service(self) -> None:
            joint_name = self.zero_joint_var.get().strip() if self.zero_joint_var else ""
            if joint_name not in self.joint_set:
                messagebox.showerror("set_zero", f"invalid joint: {joint_name}")
                return
            threading.Thread(
                target=self._call_zero_service_worker,
                args=(joint_name,),
                daemon=True,
            ).start()

        def _call_apply_limits_service_worker(self, joint_name: str) -> None:
            try:
                full_name = f"{self.service_ns}/apply_joint_limits"
                rospy.wait_for_service(full_name, timeout=2.0)
                proxy = rospy.ServiceProxy(full_name, ApplyJointLimits)
                res = proxy(
                    joint_name=joint_name,
                    min_position_rad=0.0,
                    max_position_rad=0.0,
                    use_urdf_limits=True,
                    require_current_inside_limits=False,
                )
                prefix = "OK" if res.success else "FAIL"
                self.set_service_status(
                    f"apply_joint_limits[{joint_name}/{res.backend or '-'}]: {prefix} {res.message} "
                    f"(current={res.current_position_rad:.4f}, limits=[{res.applied_min_rad:.4f}, {res.applied_max_rad:.4f}])"
                )
            except Exception as exc:
                self.set_service_status(f"apply_limits[{joint_name}]: ERROR {exc}")

        def call_apply_limits_service(self) -> None:
            joint_name = self.zero_joint_var.get().strip() if self.zero_joint_var else ""
            if joint_name not in self.joint_set:
                messagebox.showerror("apply_limits", f"invalid joint: {joint_name}")
                return
            threading.Thread(
                target=self._call_apply_limits_service_worker,
                args=(joint_name,),
                daemon=True,
            ).start()

        def on_runtime_states(self, msg: JointRuntimeStateArray) -> None:
            self.runtime_state_by_joint = {
                item.joint_name: item for item in msg.states
            }

        def refresh_ui(self) -> None:
            super().refresh_ui()
            if self.runtime_tree is None:
                return

            existing = set(self.runtime_tree.get_children(""))
            desired = set(self.joint_names)

            for joint_name in existing - desired:
                self.runtime_tree.delete(joint_name)

            for joint_name in self.joint_names:
                item = self.runtime_state_by_joint.get(joint_name)
                values = (
                    joint_name,
                    item.backend if item else "-",
                    item.lifecycle_state if item else "-",
                    "1" if item and item.online else "0",
                    "1" if item and item.enabled else "0",
                    "1" if item and item.fault else "0",
                )
                if self.runtime_tree.exists(joint_name):
                    self.runtime_tree.item(joint_name, values=values)
                else:
                    self.runtime_tree.insert("", "end", iid=joint_name, values=values)

        def _call_set_joint_mode_worker(self, joint_name: str, mode_name: str) -> None:
            full_name = f"{self.service_ns}/set_joint_mode"
            try:
                rospy.wait_for_service(full_name, timeout=2.0)
                proxy = rospy.ServiceProxy(full_name, SetJointMode)
                res = proxy(joint_name=joint_name, mode=mode_name)
                prefix = "OK" if res.success else "FAIL"
                backend_text = f"{res.backend}:{res.mapped_mode}" if res.backend else "-"
                self.set_service_status(
                    f"set_joint_mode[{joint_name}]={mode_name}: {prefix} {res.message} ({backend_text})"
                )
            except Exception as exc:
                self.set_service_status(f"set_joint_mode[{joint_name}]: ERROR {exc}")

        def call_set_mode_service(self) -> None:
            joint_name = self.mode_joint_var.get().strip()
            if joint_name not in self.joint_set:
                messagebox.showerror("set_mode", f"invalid joint: {joint_name}")
                return
            mode_name = self.mode_value_var.get().strip().lower()
            if mode_name not in self.available_modes:
                messagebox.showerror(
                    "set_mode",
                    f"mode must be one of: {', '.join(self.available_modes)}",
                )
                return
            threading.Thread(
                target=self._call_set_joint_mode_worker,
                args=(joint_name, mode_name),
                daemon=True,
            ).start()

        def on_close(self) -> None:
            try:
                super().on_close()
            finally:
                if not self.keep_temp and self.temp_yaml_path and os.path.exists(self.temp_yaml_path):
                    os.unlink(self.temp_yaml_path)

        def reconnect_action_client(self, action_ns: str) -> None:
            super().reconnect_action_client(action_ns)
            self.refresh_action_joint_filter()

        def apply_action_ns(self) -> None:
            super().apply_action_ns()
            self.refresh_action_joint_filter()

        def refresh_action_joint_filter(self) -> None:
            action_ns = upstream_module.normalize_action_ns(self.action_ns)
            suffix = "/follow_joint_trajectory"
            controller_ns = action_ns[:-len(suffix)] if action_ns.endswith(suffix) else action_ns
            controller_ns = controller_ns.rstrip("/") or "/"
            param_name = f"{controller_ns}/joints"

            configured = []
            try:
                value = rospy.get_param(param_name)
                if isinstance(value, list):
                    configured = [str(name) for name in value]
            except Exception:
                configured = []

            if not configured:
                self.action_joint_names = list(self.joint_names)
                self.set_service_status(
                    f"action joints: using all UI joints; missing param {param_name}"
                )
                return

            known = [name for name in configured if name in self.joint_set]
            unknown = [name for name in configured if name not in self.joint_set]
            self.action_joint_names = known
            detail = f"action joints: {len(known)}/{len(configured)} from {param_name}"
            if unknown:
                detail += f"; unknown={','.join(unknown)}"
            self.set_service_status(detail)

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

            self.refresh_action_joint_filter()
            action_joint_names = list(self.action_joint_names)
            if not action_joint_names:
                messagebox.showerror(
                    "action joints",
                    f"no UI joints match controller joints for {self.action_ns}",
                )
                return
            not_running = [
                name
                for name in action_joint_names
                if self.runtime_state_by_joint.get(name) is None
                or self.runtime_state_by_joint[name].lifecycle_state != "Running"
            ]
            if not_running:
                messagebox.showwarning(
                    "lifecycle",
                    "action requires Running lifecycle; call resume before sending goals: "
                    + ", ".join(not_running[:4])
                    + (" ..." if len(not_running) > 4 else ""),
                )
                return

            goal = upstream_module.FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = action_joint_names
            point = upstream_module.JointTrajectoryPoint()
            point.positions = [self.slider_vars[name].get() for name in action_joint_names]
            point.time_from_start = rospy.Duration.from_sec(duration)
            goal.trajectory.points = [point]
            goal.trajectory.header.stamp = rospy.Time.now()

            with self.lock:
                for i, name in enumerate(action_joint_names):
                    self.target[name] = point.positions[i]
                self.goal_state_text = "PENDING"

            self.client.send_goal(goal, done_cb=self.on_goal_done, active_cb=self.on_goal_active)

    return HybridJointActionUi


def parse_args():
    parser = argparse.ArgumentParser(
        description="Hybrid joint trajectory UI with backend-aware zero setting."
    )
    parser.add_argument("--canopen-joints-yaml", default="", help="Override Eyou_Canopen_Master joints.yaml path.")
    parser.add_argument("--can-driver-yaml", default="", help="Override can_driver.yaml path.")
    parser.add_argument("--joint-action-ui", default="", help="Override the upstream joint_action_ui.py path.")
    parser.add_argument("--service-ns", default=DEFAULT_SERVICE_NS, help="Default hybrid service namespace.")
    parser.add_argument(
        "--action-ns",
        default="",
        help="Override hybrid action namespace. Default: /arm_position_controller/follow_joint_trajectory.",
    )
    parser.add_argument("--slider-limit", type=float, default=3.1416, help="Absolute slider range in radians.")
    parser.add_argument("--slider-resolution", type=float, default=0.001, help="Slider resolution in radians.")
    parser.add_argument("--goal-duration", type=float, default=1.0, help="Default time_from_start in seconds.")
    parser.add_argument("--refresh-ms", type=int, default=100, help="UI refresh period in milliseconds.")
    parser.add_argument("--dry-run", action="store_true", help="Print resolved config and exit without launching the UI.")
    parser.add_argument("--keep-temp", action="store_true", help="Keep the generated merged joints yaml after exit.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    (
        default_canopen_yaml,
        default_can_driver_yaml,
        default_joint_action_ui,
        default_action_ns,
        default_mode_mapping_file,
    ) = resolve_default_paths()

    canopen_yaml = os.path.abspath(args.canopen_joints_yaml or default_canopen_yaml)
    can_driver_yaml = os.path.abspath(args.can_driver_yaml or default_can_driver_yaml)
    joint_action_ui = os.path.abspath(args.joint_action_ui or default_joint_action_ui)
    action_ns = args.action_ns or default_action_ns
    mode_mapping_file = os.path.abspath(default_mode_mapping_file)

    merged_yaml, merged_joints = build_merged_joint_yaml(canopen_yaml, can_driver_yaml)
    joint_metadata_by_name = {item["name"]: item for item in merged_joints}
    available_modes, mode_mappings = load_mode_options(mode_mapping_file)

    if args.dry_run:
        print(f"canopen_joints_yaml: {canopen_yaml}")
        print(f"can_driver_yaml: {can_driver_yaml}")
        print(f"joint_action_ui: {joint_action_ui}")
        print(f"joint_mode_mappings_file: {mode_mapping_file}")
        print(f"merged_joints_yaml: {merged_yaml}")
        print(f"service_ns: {args.service_ns}")
        print(f"action_ns: {action_ns}")
        print("available_modes:")
        for mode_name in available_modes:
            print(f"  - {mode_name}: {mode_mappings.get(mode_name, {})}")
        print("merged_joints:")
        for item in merged_joints:
            detail_parts = [f"backend={item['backend']}"]
            if "axis_index" in item:
                detail_parts.append(f"axis_index={item['axis_index']}")
            if "motor_id" in item:
                detail_parts.append(f"motor_id={item['motor_id']}")
            detail = ", ".join(detail_parts)
            print(f"  - {item['name']} (node_id={item['node_id']}, {detail})")
        if not args.keep_temp:
            os.unlink(merged_yaml)
        return 0

    upstream_module = load_upstream_module(joint_action_ui)
    HybridJointActionUi = make_hybrid_ui_class(upstream_module)

    rospy.init_node("hybrid_joint_action_ui", anonymous=True, disable_signals=True)
    root = tk.Tk()
    HybridJointActionUi(
        root=root,
        yaml_path=merged_yaml,
        action_ns=upstream_module.normalize_action_ns(action_ns),
        service_ns=upstream_module.normalize_service_ns(args.service_ns),
        slider_limit=args.slider_limit,
        slider_resolution=args.slider_resolution,
        goal_duration=args.goal_duration,
        refresh_ms=max(20, args.refresh_ms),
        joint_metadata_by_name=joint_metadata_by_name,
        available_modes=available_modes,
        temp_yaml_path=merged_yaml,
        keep_temp=args.keep_temp,
    )
    root.mainloop()

    if not args.keep_temp and os.path.exists(merged_yaml):
        os.unlink(merged_yaml)
    return 0


if __name__ == "__main__":
    sys.exit(main())
