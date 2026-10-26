"""Device policy; I/O is supplied by the unique ros2_canopen master or a test adapter."""
import json
import math
from pathlib import Path

AXES = {'left_front_arm_joint': (1, 19857408), 'right_front_arm_joint': (2, 19857408),
        'left_rear_arm_joint': (3, 19857408), 'right_rear_arm_joint': (4, 19857408),
        'shoulder_yaw_joint': (6, 6619136), 'shoulder_pitch_joint': (5, 6619136)}
MAINTENANCE = ((0x607C, 0), (0x2003, 0), (0x607D, 1), (0x607D, 2))
IDENTITY = ((0x1000, 0), (0x1018, 1), (0x1018, 2), (0x1018, 3), (0x1018, 4), (0x100A, 0))

def key(index, sub): return f'{index:04X}:{sub:02X}'
def signed(value): return value - 2**32 if value >= 2**31 else value

def counts(radians, joint):
    if joint not in AXES or not math.isfinite(radians): raise ValueError('invalid joint/value')
    value = radians * AXES[joint][1] / (2 * math.pi)
    if not -2**31 <= value <= 2**31 - 1: raise ValueError('device count overflow')
    return int(math.floor(value + 0.5) if value >= 0 else math.ceil(value - 0.5))

def expected_pdos(bus, joint):
    expected = {}
    node = bus[joint]
    # Compare enabled and disabled PDOs; map length/content and COB-ID/type all matter.
    for kind, comm_base, map_base in [('rpdo', 0x1400, 0x1600), ('tpdo', 0x1800, 0x1A00)]:
        for number, pdo in node[kind].items():
            comm, mapping = comm_base + int(number) - 1, map_base + int(number) - 1
            expected[key(comm, 1)] = pdo['cob_id'] | (0 if pdo['enabled'] else 0x80000000)
            if not pdo['enabled']: continue
            expected[key(comm, 2)] = pdo['transmission']
            # Disabled mappings still have generated map entries, never assume empty.
            expected[key(mapping, 0)] = len(pdo['mapping'])
            for sub, obj in enumerate(pdo['mapping'], 1):
                bits = 8 if obj['index'] in (0x6060, 0x6061) else 16 if obj['index'] in (0x6040, 0x6041, 0x6077) else 32
                expected[key(mapping, sub)] = obj['index'] << 16 | obj['sub_index'] << 8 | bits
    return expected

class Policy:
    def __init__(self, io, bus, baseline, journal, limits=None):
        self.io, self.bus, self.baseline = io, bus, baseline
        self.limits = limits or {}
        self.journal = Path(journal)
        self.dirty = self.journal.exists() and json.loads(self.journal.read_text()).get('dirty', True)

    @property
    def journal_axis(self):
        return json.loads(self.journal.read_text()).get('joint')

    @property
    def dirty_axes(self):
        if not self.dirty: return set()
        data = json.loads(self.journal.read_text())
        joint = data.get('joint')
        # Legacy/unscoped journals remain conservative until explicitly resolved.
        return {joint} if joint in AXES else set(AXES)

    def read_objects(self, joint, objects):
        return {key(i, s): self.io.read(joint, i, s) for i, s in objects}

    def snapshot(self):
        return {'commissioned': False, 'axes': {
            joint: {'node_id': node, 'identity': self.read_objects(joint, IDENTITY),
                    'persistent': self.read_objects(joint, MAINTENANCE), 'save_support': self.io.read(joint, 0x1010, 1),
                    'physical_label': '', 'direction_verified': False}
            for joint, (node, _) in AXES.items()}}

    def verify(self, joints=None, allow_dirty=False):
        selected = set(AXES if joints is None else joints)
        if selected & self.dirty_axes and not allow_dirty: raise RuntimeError('maintenance outcome unresolved; restore/read back first')
        if not self.baseline.get('commissioned'): raise RuntimeError('现场基线尚未登记并确认')
        axes = self.baseline.get('axes', {})
        selected = set(AXES if joints is None else joints)
        if not selected or not selected <= set(AXES) or not selected <= set(axes):
            raise RuntimeError('baseline missing selected axes')
        for joint in selected:
            node, _ = AXES[joint]
            entry = axes[joint]
            if entry.get('node_id') != node or not entry.get('physical_label') or not entry.get('direction_verified'):
                raise RuntimeError(f'{joint}: incomplete physical baseline')
            expected = entry.get('identity', {})
            if set(expected) != {key(i, s) for i, s in IDENTITY} or any(v is None for v in expected.values()):
                raise RuntimeError(f'{joint}: incomplete identity/firmware baseline')
            actual = self.read_objects(joint, IDENTITY)
            if actual != expected: raise RuntimeError(f'{joint}: identity/firmware mismatch: {actual}')
            for obj, value in expected_pdos(self.bus, joint).items():
                index, sub = (int(x, 16) for x in obj.split(':'))
                if self.io.read(joint, index, sub) != value:
                    raise RuntimeError(f'{joint}: PDO mismatch {obj}; expected {value}')
            # Hardware zero and ROS offset must never be applied twice.
            if self.bus[joint].get('offset_pos_to_dev', 0) != 0 or self.bus[joint].get('offset_pos_from_dev', 0) != 0:
                raise RuntimeError(f'{joint}: software position offset must be zero')
            self.read_objects(joint, MAINTENANCE)  # Fail rather than mark unreadable device ready.
        return True

    def record(self, **data):
        self.journal.parent.mkdir(parents=True, exist_ok=True)
        temp = self.journal.with_suffix('.tmp')
        temp.write_text(json.dumps(data, indent=2))
        temp.replace(self.journal)
        self.dirty = data['dirty']

    def transaction(self, joint, updates, save=False):
        if joint not in AXES: raise ValueError('unknown joint')
        # Baseline acknowledgement is required even when this is a maintenance-only session.
        if not self.baseline.get('commissioned'): raise RuntimeError('persistent baseline required')
        if self.dirty: raise RuntimeError('previous operation unresolved')
        before = self.read_objects(joint, MAINTENANCE)
        self.record(dirty=True, joint=joint, before=before, updates=updates, save_requested=save)
        try:
            for index, sub, value in updates:
                self.io.write(joint, index, sub, value & 0xFFFFFFFF)
                if self.io.read(joint, index, sub) != value & 0xFFFFFFFF:
                    raise RuntimeError(f'readback mismatch {key(index, sub)}')
            if save:
                self.io.write(joint, 0x1010, 1, 0x65766173)
                # 1010 upload describes support, not proof of persistence. Reboot verification is separate.
                self.io.read(joint, 0x1010, 1)
            self.record(dirty=False, joint=joint, before=before, updates=updates,
                        reboot_verification_required=save)
        except Exception as error:
            restored = True
            # Restore enable state last, after restoring the range and home offset.
            for index, sub in ((0x607C, 0), (0x607D, 2), (0x607D, 1), (0x2003, 0)):
                try:
                    value = before[key(index, sub)]
                    self.io.write(joint, index, sub, value)
                    if self.io.read(joint, index, sub) != value: restored = False
                except Exception: restored = False
            # A timed-out save can have taken effect; runtime rollback cannot prove persistent rollback.
            self.record(dirty=save or not restored, joint=joint, before=before,
                        error=str(error), ram_rollback_verified=restored)
            raise RuntimeError(f'{joint}: {error}; RAM rollback={restored}, persistent outcome={"unknown" if save else "unchanged"}') from error

    def maintain(self, joint, operation, lower=0.0, upper=0.0, save=False):
        if operation == 'limits_from_urdf':
            if joint not in self.limits: raise ValueError('URDF limits unavailable')
            return self.maintain(joint, 'limits', self.limits[joint]['lower'], self.limits[joint]['upper'], save)
        if operation == 'set_zero':
            old = signed(self.io.read(joint, 0x607C, 0))
            actual = signed(self.io.read(joint, 0x6064, 0))
            value = old - actual
            if not -2**31 <= value <= 2**31-1: raise ValueError('zero offset overflow')
            updates = [(0x607C, 0, value)]
        elif operation == 'set_offset': updates = [(0x607C, 0, counts(lower, joint))]
        elif operation == 'limits':
            lo, hi = counts(lower, joint), counts(upper, joint)
            if lo >= hi: raise ValueError('limits must be strictly ordered')
            updates = [(0x2003, 0, 0), (0x607D, 2, hi), (0x607D, 1, lo), (0x2003, 0, 0x4C494D54)]
        elif operation == 'disable_limits': updates = [(0x2003, 0, 0)]
        else: raise ValueError('unknown maintenance operation')
        self.transaction(joint, updates, save)

    def restore(self):
        data = json.loads(self.journal.read_text())
        joint, before = data['joint'], data['before']
        if joint not in AXES or set(before) != {key(i, s) for i, s in MAINTENANCE}:
            raise ValueError('invalid restore journal')
        if any(type(v) is not int or not 0 <= v <= 0xFFFFFFFF for v in before.values()):
            raise ValueError('invalid recorded object value')
        # A failed restore is itself an uncertain maintenance outcome, even after a successful prior write.
        self.record(dirty=True, joint=joint, before=before, restore_in_progress=True, save_requested=True)
        try:
            self.io.write(joint, 0x2003, 0, 0)
            for i, s in ((0x607C, 0), (0x607D, 2), (0x607D, 1), (0x2003, 0)):
                value = before[key(i, s)]
                self.io.write(joint, i, s, value)
                if self.io.read(joint, i, s) != value: raise RuntimeError('restore readback failed')
            self.io.write(joint, 0x1010, 1, 0x65766173)
            self.io.read(joint, 0x1010, 1)
        except Exception as error:
            self.record(dirty=True, joint=joint, before=before, save_requested=True, error=str(error))
            raise
        self.record(dirty=False, joint=joint, before=before, restored=True, reboot_verification_required=True)
