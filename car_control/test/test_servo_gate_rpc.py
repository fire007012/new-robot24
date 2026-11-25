"""A runtime epoch change must drain the prior Servo RPC before starting again."""
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import json,sys,time,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from car_control_runtime.servo_gate import ServoGate


class RpcEpochTest(unittest.TestCase):
    def test_epoch_change_preserves_pending_rpc_and_late_completion_starts_once(self):
        old=Future();old.set_result(SimpleNamespace(success=True));calls=[]
        next_request=Future()
        stop=SimpleNamespace(service_is_ready=lambda:True,
            call_async=lambda request:(calls.append('stop'),next_request)[1])
        gate=SimpleNamespace(session=(True,'old'),state_at=time.monotonic(),ready=True,
             last_input=0.,servo_at=0.,pending=old,retry_at=0.,input=None,
             pub=SimpleNamespace(publish=lambda msg:None),stop=stop)
        gate.output=gate.pub
        gate.try_start=lambda:ServoGate.try_start(gate)
        gate.stopped=lambda future,token:ServoGate.stopped(gate,future,token)
        message=SimpleNamespace(data=json.dumps(dict(groups={'arm':'Running'},arm_profile='servo',
                                                       operations={'arm':{'id':'new','state':'complete'}})))
        ServoGate.state(gate,message)
        self.assertIs(gate.pending,old)
        self.assertEqual(calls,[])
        ServoGate.started(gate,old,(True,'old'))
        self.assertEqual(calls,['stop'])
        self.assertIs(gate.pending,next_request)
        ServoGate.started(gate,old,(True,'old'))
        self.assertEqual(calls,['stop'])

    def test_stopped_epoch_never_restarts_after_late_rpc(self):
        old=Future();old.set_result(SimpleNamespace(success=True))
        gate=SimpleNamespace(session=(False,'stopped'),pending=old)
        gate.try_start=lambda:ServoGate.try_start(gate)
        ServoGate.started(gate,old,(True,'old'))
        self.assertIsNone(gate.pending)
        # A stopped session returns before accessing or sending any stop/start RPC.

    def test_started_engine_is_unpaused_without_recreating_threads(self):
        stopped=Future();stopped.set_result(SimpleNamespace(success=True))
        resumed=Future()
        resume=Mock();resume.call_async.return_value=resumed
        token=(True,'current')
        gate=SimpleNamespace(pending=stopped,session=token,engine_started=True,
                             start=Mock(),resume=resume,started=Mock())
        ServoGate.stopped(gate,stopped,token)
        gate.start.call_async.assert_not_called()
        resume.call_async.assert_called_once()
        self.assertIs(gate.pending,resumed)

    def test_operations_progress_does_not_reset_active_epoch_session(self):
        marker=object()
        gate=SimpleNamespace(session=(True,3),state_at=time.monotonic(),ready=True,
                             last_input=1.,servo_at=1.,pending=None,retry_at=0.,input=marker,
                             pub=SimpleNamespace(publish=lambda msg:None),
                             stop=SimpleNamespace(service_is_ready=lambda:False))
        gate.output=gate.pub
        gate.try_start=lambda:ServoGate.try_start(gate)
        gate.destroy_subscription=lambda sub:self.fail('unexpected subscription rebuild')
        for op_state in ('in_progress','complete'):
            message=SimpleNamespace(data=json.dumps(dict(
                groups={'arm':'Running'},arm_profile='servo',input_epochs={'arm':3},
                operations={'arm':{'id':'op-1','state':op_state}})))
            ServoGate.state(gate,message)
            self.assertTrue(gate.ready)
            self.assertIs(gate.input,marker)
            self.assertEqual(gate.session,(True,3))
