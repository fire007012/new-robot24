import itertools
import json
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge_core import BridgeCore
from control_v2 import ControlV2, CAPABILITIES, SUPPORTED_CAPABILITIES
from motion_presets import PRESETS, groups
from output_adapters import RosOutput
from test_control_v2 import Capture, Clock


class PresetCapture(Capture):
    def __init__(self, clock):
        super().__init__(clock)
        self.group_calls = []
        self.motion = []
        self.group_result = dict(success=True, operation_id='runtime-op', message='ok')

    def call_group_operation(self, command, selected):
        self.group_calls.append((command, list(selected)))
        if self.block:
            self.entered.set()
            self.gate.wait(3)
        return self.group_result

    def publish_twist(self, value): self.motion.append(('base', value.linear_x))
    def publish_servo(self, value): self.motion.append(('arm', value.linear_z))
    def publish_gripper(self, value): self.motion.append(('gripper', value.velocity))
    def publish_flipper(self, value): self.motion.append(('flippers', max(value.velocities, default=0)))


class Tests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.output = PresetCapture(self.clock)
        self.core = BridgeCore(self.output, 500, .8, 1.5)
        self.c = ControlV2(self.core, self.clock)
        self.seq = 0

    def tearDown(self):
        self.output.gate.set()
        self.c.close()

    def hello(self, presets, capabilities=SUPPORTED_CAPABILITIES):
        result = self.c.handle('owner', dict(type='control_hello', protocol_version=2,
            robot_id=self.core.robot_id, role='control', capabilities=capabilities, presets=presets))
        if result['ok']:
            self.session = result['result']
        return result

    def call(self, kind, **fields):
        self.seq += 1
        return self.c.handle('owner', dict(type=kind, protocol_version=2, seq=self.seq,
            session_id=self.session['session_id'], epoch=self.session['epoch'],
            freshness_token=self.session['freshness_token'], ttl_ms=300, **fields))

    def authorize(self):
        self.assertTrue(self.call('operator_input', keyboard={'pressed_keys': []})['ok'])
        self.assertTrue(self.call('authorize_operations')['ok'])
        self.assertTrue(self.call('authorize')['ok'])

    def wait_op(self, response):
        self.assertTrue(response['ok'], response)
        oid = response['result']['operation_id']
        until = time.monotonic() + 3
        while self.c.operations[oid]['state'] in ('accepted', 'running'):
            self.assertLess(time.monotonic(), until)
            time.sleep(.005)
        return self.c.operations[oid]

    def partial_runtime(self, presets):
        selected = groups(presets)
        for g in self.output.runtime_state['groups']:
            if g not in selected:
                self.output.runtime_state['groups'][g] = 'Stopped'
                self.output.runtime_state['diagnostics']['groups'][g].update(state=0, ready=False, blockers=['not prepared'])

    def test_all_seven_combinations_permission_and_lifecycle(self):
        combinations = [list(p) for size in (1, 2, 3) for p in itertools.combinations(PRESETS, size)]
        for presets in combinations:
            with self.subTest(presets=presets):
                self.output.runtime_state = Capture(self.clock).runtime_state
                self.partial_runtime(presets)
                self.assertTrue(self.hello(presets)['ok'])
                self.authorize()
                status = self.call('control_status')['result']
                self.assertTrue(status['motion_permitted'])
                self.assertEqual(status['selected_groups'], groups(presets))
                for command in ('init', 'enable', 'resume', 'halt', 'recover', 'shutdown', 'disable'):
                    op = self.wait_op(self.call('system_command', command=command,
                        request_id='-'.join(presets)+command))
                    self.assertEqual(op['state'], 'succeeded')
                    self.assertEqual(self.output.group_calls[-1], ('halt' if command=='disable' else command, groups(presets)))
                    self.assertEqual(op['result'][0]['operation_id'], 'runtime-op')
                self.c.disconnect('owner')
                self.seq = 0

    def test_arm_includes_gripper_and_requires_both_ready(self):
        self.hello(['arm'])
        self.partial_runtime(['arm'])
        self.assertTrue(self.c.permission())
        self.output.runtime_state['groups']['gripper'] = 'Fault'
        self.assertFalse(self.c.permission())
        status = self.call('control_status')['result']
        self.assertEqual(status['presets']['arm']['states']['gripper'], 'Fault')
        self.assertFalse(status['presets']['arm']['motion_permitted'])

    def test_cold_stack_can_initialize_without_motion_authorization(self):
        self.hello(['arm'])
        for g in self.output.runtime_state['groups']:
            self.output.runtime_state['groups'][g] = 'Stopped'
            self.output.runtime_state['diagnostics']['groups'][g].update(state=0, ready=False)
        self.assertFalse(self.c.permission())
        self.call('operator_input', keyboard={'pressed_keys':[]})
        self.assertTrue(self.call('authorize_operations')['ok'])
        result = self.wait_op(self.call('system_command', command='init', request_id='cold'))
        self.assertEqual(result['state'], 'succeeded')
        self.assertEqual(self.output.group_calls, [('init',['arm','gripper'])])
        self.assertFalse(self.c.sessions['owner']['authorized'])

    def test_unselected_motion_is_never_forwarded(self):
        self.hello(['flippers'])
        self.partial_runtime(['flippers'])
        self.authorize()
        self.output.motion.clear()
        self.assertTrue(self.call('operator_input', keyboard={'pressed_keys': ['w', 'y']})['ok'])
        self.c.tick()
        self.assertEqual([g for g, _ in self.output.motion], ['flippers'])
        self.assertGreater(self.output.motion[-1][1], 0)

    def test_arm_input_includes_gripper_but_no_vehicle_output(self):
        self.hello(['arm'])
        self.partial_runtime(['arm'])
        self.authorize()
        self.core.state.control_mode = self.core.MODE_ARM
        for _ in range(2):
            self.output.motion.clear()
            self.call('operator_input', keyboard={'pressed_keys': ['w', 'f']})
            self.c.tick()
        self.assertEqual({g for g, _ in self.output.motion}, {'arm', 'gripper'})
        self.assertTrue(all(v > 0 for _, v in self.output.motion))

    def test_invalid_and_unnegotiated_presets_rejected(self):
        for value in ([], ['bogus'], ['arm','arm'], 'arm', [['arm']], None):
            self.assertEqual(self.hello(value)['error']['code'], 'PRESETS')
        self.assertEqual(self.hello(['arm'], CAPABILITIES)['error']['code'], 'CAPABILITIES')

    def test_subset_targeting_and_unselected_arm_commands(self):
        self.hello(['base','flippers'])
        self.authorize()
        result = self.wait_op(self.call('system_command', command='halt', request_id='subset', params={'presets':['base']}))
        self.assertEqual(result['groups'], ['base'])
        self.assertEqual(self.output.group_calls[-1], ('halt', ['base']))
        replay = self.call('system_command', command='halt', request_id='subset', params={'presets':['base']})
        self.assertEqual(replay['result']['operation_id'], result['operation_id'])
        self.assertEqual(len(self.output.group_calls), 1)
        conflict = self.call('system_command', command='halt', request_id='subset', params={'presets':['flippers']})
        self.assertEqual(conflict['error']['code'], 'REQUEST_CONFLICT')
        for cmd, params in [('init', {'presets':['arm']}), ('move_arm_named_target', {'target':'home'}),
                            ('request_arm_named_targets', {}), ('set_control_mode', {'mode':'arm'})]:
            self.assertEqual(self.call('system_command', command=cmd, request_id=cmd, params=params)['error']['code'], 'PRESET_NOT_SELECTED')
        self.assertEqual(self.call('system_command', command='init', request_id='raw-groups', params={'groups':['arm']})['error']['code'], 'PARAMS')
        self.assertEqual(self.call('system_command', command='set_control_mode', request_id='arm-alias', params={'mode':' ARM '})['error']['code'], 'PRESET_NOT_SELECTED')

    def test_failed_combined_operation_preserves_per_group_detail(self):
        self.hello(['arm'])
        self.authorize()
        self.output.group_result = dict(success=False, operation_id='partial', message=json.dumps(
            dict(groups={'arm':'Running','gripper':'Fault'}, operations={'gripper':{'state':'failed'}})))
        op = self.wait_op(self.call('system_command', command='resume', request_id='partial'))
        self.assertEqual(op['state'], 'failed')
        self.assertEqual(json.loads(op['result'][0]['message'])['groups']['gripper'], 'Fault')

    def test_timeout_remains_unknown(self):
        self.hello(['base'])
        self.authorize()
        with patch.object(self.output, 'call_group_operation', side_effect=TimeoutError('outcome uncertain')):
            op = self.wait_op(self.call('system_command', command='enable', request_id='timeout'))
        self.assertEqual(op['state'], 'result_unknown')

    def test_selection_requires_neutral_and_confirmed_removal(self):
        self.hello(['base','flippers'])
        self.assertEqual(self.call('select_presets', presets=['base'])['error']['code'], 'NEUTRAL_REQUIRED')
        self.authorize()
        self.call('operator_input', keyboard={'pressed_keys': []})
        self.assertEqual(self.call('select_presets', presets=['base'])['error']['code'], 'HALT_REMOVED_PRESETS')
        self.output.runtime_state['groups']['flippers'] = 'Halted'
        self.assertEqual(self.call('select_presets', presets=['base'])['error']['code'], 'HALT_REMOVED_PRESETS')
        self.output.runtime_state['diagnostics']['groups']['flippers']['state'] = 0
        self.output.runtime_received = 0
        self.assertEqual(self.call('select_presets', presets=['base'])['error']['code'], 'HALT_REMOVED_PRESETS')
        self.output.runtime_received = self.clock()
        old_epoch = self.session['epoch']
        response = self.call('select_presets', presets=['base'])
        self.assertTrue(response['ok'], response)
        self.assertNotEqual(old_epoch, response['result']['epoch'])
        self.assertFalse(self.c.sessions['owner']['authorized'])
        self.assertIsNone(self.c.latest)
        self.assertEqual(self.output.group_calls, [])
        self.assertEqual(self.call('authorize')['error']['code'], 'EPOCH')

    def test_addition_clears_input_without_restarting_existing_groups(self):
        self.hello(['base'])
        self.authorize()
        response = self.call('select_presets', presets=['base','arm'])
        self.assertTrue(response['ok'], response)
        self.assertFalse(response['result']['authorized'])
        self.assertEqual(self.output.calls, [])
        self.assertEqual(self.output.group_calls, [])
        self.assertTrue(self.c.safety.empty())
        self.assertEqual(self.output.runtime_state['groups']['base'], 'Running')

    def test_cannot_reselect_during_active_operation(self):
        self.hello(['base'])
        self.authorize()
        self.output.block = True
        response = self.call('system_command', command='init', request_id='blocking')
        self.assertTrue(self.output.entered.wait(1))
        self.call('operator_input', keyboard={'pressed_keys': []})
        self.assertEqual(self.call('select_presets', presets=['base','arm'])['error']['code'], 'OPERATIONS_BUSY')
        self.output.gate.set()
        self.wait_op(response)

    def test_vehicle_mode_does_not_touch_unselected_arm(self):
        self.hello(['flippers'])
        self.authorize()
        with patch.object(self.output, 'call_command_service', side_effect=AssertionError('unselected arm')):
            op = self.wait_op(self.call('system_command', command='set_control_mode', request_id='mode', params={'mode':'vehicle'}))
        self.assertEqual(op['state'], 'succeeded')

    def test_group_transition_does_not_globally_halt_healthy_peer(self):
        self.hello(['base','flippers'])
        self.authorize()
        def halt(command, selected):
            self.output.group_calls.append((command, selected))
            self.output.runtime_state['groups']['flippers'] = 'Halted'
            self.c.runtime_changed()
            return dict(success=True, operation_id='halt-one', message='ok')
        with patch.object(self.output, 'call_group_operation', side_effect=halt):
            op = self.wait_op(self.call('system_command', command='halt', request_id='halt-one', params={'presets':['flippers']}))
        self.assertEqual(op['state'], 'succeeded')
        self.assertTrue(op['invalidated'])
        self.assertEqual(self.output.runtime_state['groups']['base'], 'Running')
        self.assertEqual(self.output.calls, [])
        self.assertTrue(self.c.safety.empty())

    def test_authorize_cannot_race_group_operation(self):
        self.hello(['base'])
        self.authorize()
        self.output.block = True
        response = self.call('system_command', command='init', request_id='blocking-auth')
        self.assertTrue(self.output.entered.wait(1))
        self.call('operator_input', keyboard={'pressed_keys': []})
        self.assertEqual(self.call('authorize')['error']['code'], 'OPERATIONS_BUSY')
        self.output.gate.set()
        self.wait_op(response)

    def test_adapter_uses_single_group_service(self):
        output = RosOutput.__new__(RosOutput)
        requests = []
        def request(kind, path, value):
            requests.append((path, value))
            return SimpleNamespace(success=True, operation_id='ros-op', message='ok')
        output.request = request
        fake = SimpleNamespace(GroupOperation=SimpleNamespace(Request=lambda **kw: SimpleNamespace(**kw)))
        with patch.dict(sys.modules, {'robot_hardware.srv': fake}):
            result = output.call_group_operation('enable', ['arm','gripper'])
        self.assertEqual(requests[0][0], '/robot_runtime/group_operation')
        self.assertEqual(requests[0][1].groups, ['arm','gripper'])
        self.assertEqual(requests[0][1].resource, '')
        self.assertEqual(result['operation_id'], 'ros-op')


if __name__ == '__main__':
    unittest.main(verbosity=2)
