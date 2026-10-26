"""Serialized whole-robot lifecycle; adapter owns ROS service calls, no hidden master."""
import threading

class Coordinator:
    COMPONENTS = ('RobotCanopenSystem', 'CanDriverSystem')
    def __init__(self, io, policy):
        self.io, self.policy = io, policy
        self.lock = threading.RLock()
        self.state = 'Stopped'
        self.profile = 'csv'
        self.session_generation = 0

    def generation(self, group):
        return self.session_generation

    def group_state(self, group):
        return self.state

    def stop(self):
        self.session_generation += 1
        errors = []
        for action in (lambda: self.io.running(False), lambda: self.io.controllers([], self.io.active_controllers())):
            try: action()
            except Exception as error: errors.append(str(error))
        self.state = 'Fault' if errors else 'Stopped'
        if errors: raise RuntimeError('; '.join(errors))

    def init(self):
        with self.lock:
            try:
                for name in self.COMPONENTS: self.io.hardware(name, 'inactive')
                self.policy.verify()
                self.state = 'Initialized'
            except Exception:
                self.converge(); raise

    def enable(self):
        with self.lock:
            if self.state != 'Initialized': raise RuntimeError('init required')
            try:
                for name in self.COMPONENTS: self.io.hardware(name, 'active')
                # Humble may reject stop requests while a failed component's interfaces are unavailable.
                # Restore availability under the closed gate, then retire every lingering controller.
                lingering = self.io.active_controllers()
                if lingering: self.io.controllers([], lingering)
                self.io.arm()
                self.state = 'Armed'
            except Exception:
                self.converge(); raise

    def resume(self):
        with self.lock:
            if self.state not in ('Armed', 'Halted'): raise RuntimeError('enable/recover required')
            try:
                if self.state == 'Halted': self.io.arm()
                controllers = ['arm_position_controller', 'gripper_controller', 'wheel_controller',
                               'flipper_csv_forward_controller' if self.profile == 'csv' else 'flipper_csp_controller']
                self.io.controllers(controllers, [])
                self.io.running(True)
                self.state = 'Running'
            except Exception:
                self.converge(); raise

    def halt(self):
        with self.lock:
            self.stop()
            self.state = 'Halted'

    def recover(self):
        with self.lock:
            self.converge()
            try:
                for name in reversed(self.COMPONENTS): self.io.hardware(name, 'unconfigured')
                self.init()
                self.enable()
            except Exception:
                self.converge(); raise  # Explicit resume and fresh controller session still required.

    def shutdown(self):
        with self.lock:
            self.converge()
            errors = []
            for name in reversed(self.COMPONENTS):
                try: self.io.hardware(name, 'unconfigured')
                except Exception as error: errors.append(str(error))
            self.state = 'Fault' if errors else 'Stopped'
            if errors: raise RuntimeError('; '.join(errors))

    def converge(self):
        # Attempt every backend even after one fails; never restore Running intent.
        try: self.stop()
        except Exception: pass
        for name in reversed(self.COMPONENTS):
            try: self.io.hardware(name, 'inactive')
            except Exception: pass
        self.state = 'Fault'

    def switch_profile(self, profile):
        if profile not in ('csv', 'csp'): raise ValueError('profile must be csv or csp')
        with self.lock:
            was_running = self.state == 'Running'
            if was_running: self.halt()
            self.profile = profile
            if was_running: self.resume()

    def maintain(self, joint, operation, lower, upper, save):
        with self.lock:
            allowed = ('Initialized', 'Halted', 'Armed', 'Fault') if operation == 'restore' else ('Initialized', 'Halted', 'Armed')
            if self.state not in allowed: raise RuntimeError('stop before maintenance')
            self.stop()
            # Disable every axis while retaining the single master's SDO services.
            for name in self.COMPONENTS: self.io.hardware(name, 'inactive')
            for axis in self.policy.baseline['axes']: self.io.disable(axis)
            self.policy.verify(allow_dirty=operation == 'restore')
            try:
                if operation == 'restore': self.policy.restore()
                else: self.policy.maintain(joint, operation, lower, upper, save)
            finally: self.state = 'Initialized' if not self.policy.dirty else 'Fault'
