"""ROS-independent flipper jog reference and session-fenced controller; joint signs remain the URDF/driver convention."""
import math
import threading
import time

NAMES = ('left_front_arm_joint', 'right_front_arm_joint', 'left_rear_arm_joint', 'right_rear_arm_joint')
GROUPS = {'independent': ((0,), (1,), (2,), (3,)), 'left_right_mirror': ((0,1),(2,3)),
          'front_rear_sync': ((0,1,2,3),), 'side_pair': ((0,2),(1,3)), 'diagonal_pair': ((0,3),(1,2))}
class Reference:
    def __init__(self, limits):
        self.limits = limits; self.linkage='independent'; self.reset()
    def reset(self):
        self.reference=None;self.velocity=[0.0]*4;self.filtered=[0.0]*4;self.stamp=None
    def command(self, names, values, now):
        if len(names)!=len(values) or not names or len(set(names))!=len(names):
            self.reset();raise ValueError('invalid jog names/size')
        expanded={}
        for name,value in zip(names,values):
            if name not in NAMES or not math.isfinite(value): self.reset();raise ValueError('invalid jog')
            i=NAMES.index(name)
            affected=next(group for group in GROUPS[self.linkage] if i in group)
            for index in affected:
                if index in expanded and abs(expanded[index]-value)>1e-9:
                    self.reset();raise ValueError('conflicting linkage commands')
                expanded[index]=value
        self.velocity=[max(-.3,min(.3,expanded.get(i,0.0))) for i in range(4)];self.stamp=now
    def step(self, measured, now, dt):
        if len(measured)!=4 or any(not math.isfinite(q) for q in measured): self.reset();raise ValueError('invalid feedback')
        if self.reference is None: self.reference=list(measured)
        timed_out=self.stamp is None or now<self.stamp or now-self.stamp>.25
        for i,name in enumerate(NAMES):
            lo,hi=self.limits[name]['lower'],self.limits[name]['upper']
            if abs(measured[i]-self.reference[i])>.35: self.reference[i]=measured[i]
            target=0.0 if timed_out else self.velocity[i]
            self.filtered[i]=0.0 if timed_out else .35*target+.65*self.filtered[i]
            if (measured[i]<=lo and self.filtered[i]<0) or (measured[i]>=hi and self.filtered[i]>0):self.filtered[i]=0.0
            self.reference[i]=max(lo,min(hi,self.reference[i]+self.filtered[i]*max(0,min(dt,.02))))
        return list(self.reference),list(self.filtered),timed_out


class FlipperController:
    """Session-fenced 50 Hz flipper jog/trajectory bridge operating on a host node."""

    @staticmethod
    def attach(host, limits):
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from trajectory_msgs.msg import JointTrajectory

        host.reference = Reference(limits)
        host.measured = None
        host.measured_at = 0.0
        host.last_step_at = 0.0
        host.last_profile = None
        host.last_subscription_session = None
        host.flipper_lock = threading.RLock()
        host.jog_active = False
        host.csv_pub = host.create_publisher(Float64MultiArray, '/flipper_csv_forward_controller/command', 1)
        host.csp_pub = host.create_publisher(JointTrajectory, '/flipper_csp_controller/joint_trajectory', 1)
        host.joint_sub = host.create_subscription(JointState, '/joint_states', host.joint_state, 1)
        host.retired_subscription_errors = 0
        host.last_subscription_retirement = 0.0
        host.jog_sub = host.trajectory_sub = None
        FlipperController.sync_session(host)
        host.flipper_timer = host.create_timer(0.02, host.flipper_step)

    @staticmethod
    def _lock_for(host):
        lock = getattr(host, 'flipper_lock', None)
        if lock is None:
            lock = threading.RLock()
            host.flipper_lock = lock
        return lock

    @staticmethod
    def set_linkage(host, request, response):
        if request.profile not in GROUPS:
            response.success = False
            response.message = 'unknown linkage mode'
            return response
        with FlipperController._lock_for(host):
            host.reference.reset()
            host.reference.linkage = request.profile
        response.success = True
        response.message = request.profile
        return response

    @staticmethod
    def update_joint_state(host, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in NAMES):
            with FlipperController._lock_for(host):
                host.measured = [values[name] for name in NAMES]
                host.measured_at = time.monotonic()

    @staticmethod
    def sync_session(host):
        from control_msgs.msg import JointJog
        from trajectory_msgs.msg import JointTrajectory

        with FlipperController._lock_for(host):
            state = (host.coordinator.group_state('flippers'), host.coordinator.profile,
                     host.coordinator.generation('flippers'))
            sub_session = (state[1], state[2])
            if state != host.last_profile:
                host.reference.reset()
                host.jog_active = False
                host.last_profile = state
            # Retire/recreate DDS subscriptions only when profile or session generation
            # changes (arm/stop), avoiding DDS rematch race on Armed -> Running.
            if sub_session != getattr(host, 'last_subscription_session', None) or host.jog_sub is None:
                host.last_subscription_session = sub_session
                for subscription in (host.jog_sub, host.trajectory_sub):
                    if subscription is not None:
                        # Humble may already have scheduled a take for this handle.
                        # Its InvalidHandle is consumed at the spin boundary below;
                        # the retired callback retains its old session fence.
                        host.retired_subscription_errors = min(8, host.retired_subscription_errors + 1)
                        host.last_subscription_retirement = time.monotonic()
                        host.destroy_subscription(subscription)
                host.jog_sub = host.create_subscription(
                    JointJog, '/flipper_control/jog_cmd',
                    lambda message, session=sub_session: host.jog(message, session), 1)
                host.trajectory_sub = host.create_subscription(
                    JointTrajectory, '/flipper_control/command',
                    lambda message, session=sub_session: host.trajectory(message, session), 1)
            return state

    @staticmethod
    def handle_jog(host, message, session):
        with FlipperController._lock_for(host):
            current = FlipperController.sync_session(host)
            current_session = (current[1], current[2])
            if (session != current and session != current_session) or current[0] != 'Running':
                return
            values = list(message.velocities)
            if not values and message.duration > 0:
                values = [d / message.duration for d in message.displacements]
            try:
                host.reference.command(list(message.joint_names), values, time.monotonic())
                host.jog_active = True
            except ValueError as error:
                host.get_logger().error(str(error))
                host.jog_active = False

    @staticmethod
    def handle_trajectory(host, message, session):
        with FlipperController._lock_for(host):
            current = FlipperController.sync_session(host)
            current_session = (current[1], current[2])
            if (session != current and session != current_session) or current[0] != 'Running' or host.coordinator.profile != 'csp':
                return
            try:
                if set(message.joint_names) != set(NAMES) or len(message.joint_names) != 4 or not message.points:
                    raise ValueError('CSP trajectory requires four unique flipper joints')
                previous = -1
                for point in message.points:
                    stamp = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
                    if stamp <= previous or len(point.positions) != 4:
                        raise ValueError('invalid trajectory timing/size')
                    previous = stamp
                    for name, q in zip(message.joint_names, point.positions):
                        limit = host.limits[name]
                        if not math.isfinite(q) or not limit['lower'] <= q <= limit['upper']:
                            raise ValueError('trajectory outside limits')
                    if any(not math.isfinite(v) for v in list(point.velocities) + list(point.accelerations)):
                        raise ValueError('non-finite trajectory derivative')
                host.reference.reset()
                host.jog_active = False
                host.csp_pub.publish(message)
            except ValueError as error:
                host.get_logger().error(str(error))

    @staticmethod
    def step(host):
        from std_msgs.msg import Float64MultiArray
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        with FlipperController._lock_for(host):
            state = FlipperController.sync_session(host)
            now = time.monotonic()
            prev = getattr(host, 'last_step_at', 0.0)
            dt = max(0.0, min(0.1, now - prev)) if prev > 0.0 else .02
            host.last_step_at = now
            if state[0] != 'Running' or host.measured is None:
                return
            if now - host.measured_at > .3:
                host.csv_pub.publish(Float64MultiArray(data=[0.0] * 4))
                host.reference.reset()
                return
            if not host.jog_active:
                return
            try:
                positions, velocities, expired = host.reference.step(host.measured, now, dt)
            except ValueError:
                return
            if host.coordinator.profile == 'csv':
                host.csv_pub.publish(Float64MultiArray(data=velocities))
            else:
                point = JointTrajectoryPoint(positions=positions)
                point.time_from_start.nanosec = 40000000
                host.csp_pub.publish(JointTrajectory(joint_names=list(NAMES), points=[point]))
            if expired:
                host.jog_active = False
