import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from manager import identity
from mock_recovery import process, recover_mock_resources


class MockRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory(prefix='robot-mock-orphan-')
        self.addCleanup(self.folder.cleanup)
        self.install = Path(self.folder.name) / 'install'
        self.state = Path(self.folder.name) / 'state'
        self.state.mkdir()
        self.runtime = self.install / 'robot_hardware/lib/robot_hardware/robot_runtime'
        self.runtime.parent.mkdir(parents=True)
        self.runtime.write_text('import time\ntime.sleep(60)\n')
        self.params = self.state / 'launch.yaml'
        self.params.write_text('/**:\n  ros__parameters:\n    simulation: true\n    journal: ' + str(self.state / 'maintenance.json') + '\n')

    def orphan(self, unknown=False):
        commands = [[sys.executable, str(self.runtime), '--params-file', str(self.params)]]
        if unknown:
            extra = self.install / 'unknown.py'; extra.write_text('import time\ntime.sleep(60)\n')
            commands.append([sys.executable, str(extra)])
        launcher = 'import json,subprocess,sys\nchildren=[subprocess.Popen(c,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL) for c in json.loads(sys.argv[1])]\nprint(json.dumps([p.pid for p in children]),flush=True)\nsys.stdin.readline()\n'
        leader = subprocess.Popen([sys.executable, '-c', launcher, json.dumps(commands)],
                                  start_new_session=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        children = json.loads(leader.stdout.readline())
        resource = dict(module='core', pid=leader.pid, identity=identity(leader.pid), pending=False)
        handles = [os.pidfd_open(pid) for pid in children]
        def cleanup():
            for handle in handles:
                try: signal.pidfd_send_signal(handle, signal.SIGKILL)
                except ProcessLookupError: pass
                finally: os.close(handle)
            if leader.poll() is None: leader.kill()
            leader.wait(); leader.stdin.close(); leader.stdout.close()
        self.addCleanup(cleanup)
        leader.stdin.write('\n'); leader.stdin.flush(); leader.wait(timeout=3)
        return [resource], children

    def test_verified_orphan_mock_runtime_is_stopped_by_pidfd(self):
        resources, children = self.orphan()
        self.assertIsNotNone(process(children[0]))
        result = recover_mock_resources(dict(mode='mock'), resources, self.install, self.state)
        self.assertEqual(result, children)
        self.assertIsNone(process(children[0]))

    def test_hardware_is_rejected_without_signalling(self):
        resources, children = self.orphan()
        with self.assertRaisesRegex(ValueError, 'hardware'):
            recover_mock_resources(dict(mode='hardware'), resources, self.install, self.state)
        self.assertIsNotNone(process(children[0]))

    def test_unknown_member_blocks_the_entire_cleanup_before_signalling(self):
        resources, children = self.orphan(unknown=True)
        with self.assertRaisesRegex(ValueError, 'unverified process'):
            recover_mock_resources(dict(mode='mock'), resources, self.install, self.state)
        self.assertTrue(all(process(pid) is not None for pid in children))

    def test_non_simulated_runtime_or_wrong_journal_is_rejected(self):
        resources, children = self.orphan()
        for contents in ('/**:\n  ros__parameters:\n    simulation: false\n',
                         '/**:\n  ros__parameters:\n    simulation: true\n    journal: /tmp/another-journal.json\n'):
            self.params.write_text(contents)
            with self.assertRaises(ValueError):
                recover_mock_resources(dict(mode='mock'), resources, self.install, self.state)
            self.assertIsNotNone(process(children[0]))

    def test_unknown_boot_pending_identity_and_reused_leader_pid_are_rejected(self):
        resources, children = self.orphan()
        cases = [dict(resources[0], identity=['unknown-boot', '1']), dict(resources[0], pending=True),
                 dict(resources[0], pid=os.getpid(), identity=[identity(os.getpid())[0], '1'])]
        for resource in cases:
            with self.assertRaises(ValueError):
                recover_mock_resources(dict(mode='mock'), [resource], self.install, self.state)
            self.assertIsNotNone(process(children[0]))


if __name__ == '__main__':
    unittest.main()
