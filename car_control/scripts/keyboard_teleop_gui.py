#!/usr/bin/env python3

import json
import threading

import rospy
from std_msgs.msg import String

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError as exc:
    raise SystemExit("keyboard_teleop_gui.py requires tkinter: %s" % exc)


class KeyboardTeleopGui(object):
    PUBLISH_PULSE_KEYS = {"tab", "enter"}

    def __init__(self):
        self.raw_state_topic = rospy.get_param("~raw_state_topic")
        self.status_topic = rospy.get_param("~status_topic")
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.window_title = rospy.get_param("~window_title", "Robot24 Keyboard Teleop")

        self.state_pub = rospy.Publisher(self.raw_state_topic, String, queue_size=10)
        self.status_sub = rospy.Subscriber(
            self.status_topic, String, self.status_cb, queue_size=10
        )

        self.lock = threading.Lock()
        self.pressed_keys = set()
        self.pulse_keys = set()
        self.focused = False
        self.last_status = {}

        self.root = tk.Tk()
        self.root.title(self.window_title)
        self.root.geometry("980x620")
        self.root.minsize(900, 560)
        self.root.configure(bg="#f2f3f5")

        self.mode_var = tk.StringVar(value="Translator mode: unknown")
        self.speed_var = tk.StringVar(value="Speed: normal")
        self.focus_var = tk.StringVar(value="Focus: inactive")
        self.gripper_var = tk.StringVar(value="Gripper target: --")
        self.flipper_var = tk.StringVar(value="Flipper profile: pending")
        self.keys_var = tk.StringVar(value="Pressed keys: none")

        self.key_labels = {}

        self.build_ui()
        self.bind_events()

        self.root.after(200, self.focus_window)
        self.root.after(0, self.tick)

        rospy.loginfo(
            "keyboard_teleop_gui started: raw=%s status=%s", self.raw_state_topic, self.status_topic
        )

    def build_ui(self):
        style = ttk.Style()
        style.configure("Card.TFrame", background="#ffffff")
        style.configure("Info.TLabel", background="#ffffff", font=("TkDefaultFont", 11))
        style.configure(
            "Title.TLabel", background="#f2f3f5", font=("TkDefaultFont", 16, "bold")
        )

        container = ttk.Frame(self.root, padding=16, style="Card.TFrame")
        container.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)

        ttk.Label(
            container,
            text="Robot24 Keyboard Teleop",
            style="Title.TLabel",
        ).pack(anchor=tk.W, pady=(0, 8))

        info = ttk.Frame(container, style="Card.TFrame")
        info.pack(fill=tk.X, pady=(0, 12))

        for variable in (
            self.mode_var,
            self.speed_var,
            self.focus_var,
            self.gripper_var,
            self.flipper_var,
            self.keys_var,
        ):
            ttk.Label(info, textvariable=variable, style="Info.TLabel").pack(
                anchor=tk.W, pady=2
            )

        hint = tk.Label(
            container,
            text=(
                "Click this window once to capture keys. Tab switches BASE/ARM mode. "
                "Enter sends emergency stop. Shift is fast mode and Z is slow mode."
            ),
            anchor="w",
            justify="left",
            bg="#f2f3f5",
            fg="#364152",
        )
        hint.pack(fill=tk.X, pady=(0, 12))

        mappings = [
            (
                "Global",
                [
                    ("tab", "toggle base / arm mode"),
                    ("enter", "emergency stop"),
                    ("shift", "fast speed"),
                    ("z", "slow speed"),
                ],
            ),
            (
                "Base Mode",
                [
                    ("w / s", "forward / backward"),
                    ("a / d", "rotate left / right"),
                    ("u / j", "left front flipper + / -"),
                    ("i / k", "right front flipper + / -"),
                    ("o / l", "left rear flipper + / -"),
                    ("p / ;", "right rear flipper + / -"),
                ],
            ),
            (
                "Arm Mode",
                [
                    ("w / s", "linear z + / -"),
                    ("a / d", "linear y + / -"),
                    ("u / o", "linear x + / -"),
                    ("i / k", "pitch + / -"),
                    ("j / l", "yaw + / -"),
                    ("q / e", "roll + / -"),
                    ("f / h", "gripper open / close"),
                ],
            ),
        ]

        mapping_frame = ttk.Frame(container, style="Card.TFrame")
        mapping_frame.pack(fill=tk.BOTH, expand=True)

        for column, (title, rows) in enumerate(mappings):
            card = ttk.LabelFrame(mapping_frame, text=title, padding=12)
            card.grid(row=0, column=column, sticky="nsew", padx=6, pady=6)
            mapping_frame.columnconfigure(column, weight=1)
            for row, (keys, description) in enumerate(rows):
                row_frame = ttk.Frame(card)
                row_frame.grid(row=row, column=0, sticky="ew", pady=4)
                ttk.Label(
                    row_frame,
                    text=description,
                    width=24,
                    anchor="w",
                ).pack(side=tk.RIGHT, fill=tk.X, expand=True)
                self.build_key_badges(row_frame, keys)

        footer = tk.Label(
            container,
            text="Focused keys are highlighted in green. Pulse keys flash amber for one update.",
            anchor="w",
            justify="left",
            bg="#f2f3f5",
            fg="#5c6b7a",
        )
        footer.pack(fill=tk.X, pady=(12, 0))

    def build_key_badges(self, parent, keys):
        badge_frame = ttk.Frame(parent)
        badge_frame.pack(side=tk.LEFT, padx=(0, 10))
        for token in [item.strip() for item in keys.split("/")]:
            normalized = self.canonical_key(token.strip())
            if not normalized:
                continue
            label = tk.Label(
                badge_frame,
                text=token.strip(),
                width=max(4, len(token.strip()) + 1),
                relief=tk.RIDGE,
                borderwidth=1,
                bg="#ffffff",
                fg="#273240",
                padx=4,
                pady=2,
            )
            label.pack(side=tk.LEFT, padx=2)
            self.key_labels.setdefault(normalized, []).append(label)

    def bind_events(self):
        self.root.bind("<FocusIn>", self.on_focus_in)
        self.root.bind("<FocusOut>", self.on_focus_out)
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.bind("<Button-1>", lambda _event: self.focus_window())
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def focus_window(self):
        try:
            self.root.lift()
            self.root.focus_force()
        except tk.TclError:
            pass

    def on_focus_in(self, _event):
        with self.lock:
            self.focused = True
        self.focus_var.set("Focus: active")

    def on_focus_out(self, _event):
        with self.lock:
            self.focused = False
            self.pressed_keys.clear()
            self.pulse_keys.clear()
        self.focus_var.set("Focus: inactive")
        self.publish_state()

    def on_key_press(self, event):
        key = self.normalize_keysym(event.keysym)
        if not key:
            return "break"

        with self.lock:
            if key == "escape":
                self.root.after(0, self.on_close)
                return "break"

            if key in self.PUBLISH_PULSE_KEYS:
                if key == "enter":
                    self.pressed_keys.clear()
                self.pulse_keys.add(key)
                return "break"

            self.pressed_keys.add(key)
        return "break"

    def on_key_release(self, event):
        key = self.normalize_keysym(event.keysym)
        if not key or key in self.PUBLISH_PULSE_KEYS:
            return "break"

        with self.lock:
            self.pressed_keys.discard(key)
        return "break"

    def status_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            return

        with self.lock:
            self.last_status = payload

    def publish_state(self):
        with self.lock:
            payload = {
                "pressed": sorted(self.pressed_keys),
                "pulse": sorted(self.pulse_keys),
                "focused": self.focused,
                "stamp": rospy.Time.now().to_sec(),
            }
            self.pulse_keys.clear()

        self.state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def refresh_ui(self):
        with self.lock:
            pressed = set(self.pressed_keys)
            pulses = set(self.pulse_keys)
            status = dict(self.last_status)
            focused = self.focused

        translator_mode = status.get("mode", "unknown")
        speed_mode = status.get("speed_mode", "normal")
        stale = bool(status.get("stale", False))
        self.mode_var.set("Translator mode: %s" % translator_mode.upper())
        self.speed_var.set("Speed: %s" % speed_mode)
        self.focus_var.set(
            "Focus: %s%s"
            % ("active" if focused else "inactive", " (stale input)" if stale else "")
        )
        self.gripper_var.set(
            "Gripper target: %s" % status.get("gripper_target", "--")
        )
        self.flipper_var.set(
            "Flipper profile: %s"
            % status.get("flipper_profile_result", "pending")
        )
        visible_keys = sorted(pressed | pulses)
        self.keys_var.set(
            "Pressed keys: %s" % (", ".join(visible_keys) if visible_keys else "none")
        )

        for key, widgets in self.key_labels.items():
            if key in pulses:
                bg = "#f6c453"
            elif key in pressed:
                bg = "#7bd88f"
            else:
                bg = "#ffffff"
            for widget in widgets:
                widget.configure(bg=bg)

    def tick(self):
        if rospy.is_shutdown():
            self.on_close()
            return

        self.publish_state()
        self.refresh_ui()
        period_ms = max(int(1000.0 / max(self.publish_rate, 1.0)), 20)
        self.root.after(period_ms, self.tick)

    def on_close(self):
        with self.lock:
            self.focused = False
            self.pressed_keys.clear()
            self.pulse_keys.clear()
        try:
            self.publish_state()
        except rospy.ROSException:
            pass
        self.root.destroy()
        rospy.signal_shutdown("keyboard teleop GUI closed")

    def canonical_key(self, token):
        return self.normalize_keysym(token.replace(" ", ""))

    def normalize_keysym(self, keysym):
        if not keysym:
            return ""

        lowered = keysym.lower()
        if lowered in ("shift_l", "shift_r", "shift"):
            return "shift"
        if lowered in ("return", "kp_enter"):
            return "enter"
        if lowered == "tab":
            return "tab"
        if lowered in ("semicolon", "colon", ";"):
            return "semicolon"
        if lowered == "escape":
            return "escape"
        if len(lowered) == 1 and lowered.isprintable():
            return lowered
        return ""

    def spin(self):
        self.root.mainloop()


if __name__ == "__main__":
    rospy.init_node("keyboard_teleop_gui")
    KeyboardTeleopGui().spin()
