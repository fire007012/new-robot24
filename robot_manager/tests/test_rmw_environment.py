"""The manager's launch tree and health helpers must use one middleware."""
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from manager import Backend


class MiddlewareTests(unittest.TestCase):
    def selected(self, override=None):
        with tempfile.TemporaryDirectory() as directory:
            setup=Path(directory)/'setup.bash';setup.write_text('true\n')
            backend=Backend.__new__(Backend);backend.config={'setup':str(setup)}
            env=dict(os.environ);env.pop('RMW_IMPLEMENTATION',None)
            if override is not None:env['RMW_IMPLEMENTATION']=override
            return subprocess.check_output(backend.command([sys.executable,'-c',
                'import os; print(os.environ["RMW_IMPLEMENTATION"])']),env=env,text=True).strip()

    def test_default_is_shared_by_launch_and_probe_commands(self):
        self.assertEqual(self.selected(),'rmw_cyclonedds_cpp')

    def test_explicit_site_override_is_preserved(self):
        self.assertEqual(self.selected('rmw_fastrtps_cpp'),'rmw_fastrtps_cpp')

if __name__=='__main__':unittest.main()
