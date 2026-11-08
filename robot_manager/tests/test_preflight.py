import hashlib
import json
from pathlib import Path
import socket
import sys
import subprocess
import os
import signal
import time
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from manager import Backend,Fault,identity

class Tests(unittest.TestCase):
    def test_invalid_device_port_release_and_paths(self):
        with tempfile.TemporaryDirectory(prefix='robot-preflight-') as folder:
            root=Path(folder)
            (root/'setup.bash').write_text('true\n')
            (root/'release.json').write_text(json.dumps(dict(build_version='test',management_protocol=1,control_protocol=2,files={'setup.bash':hashlib.sha256((root/'setup.bash').read_bytes()).hexdigest()})))
            with socket.socket() as occupied:
                occupied.bind(('0.0.0.0',0));occupied.listen()
                config=dict(build_version='test',setup=str(root/'setup.bash'),build_manifest=str(root/'release.json'),control_endpoint=dict(port=occupied.getsockname()[1]))
                b=Backend(config,root,root)
                b.run=lambda args,timeout:''
                profile=dict(mode='mock',video=False,build_version='test',devices=['robot24_nonexistent_can'])
                checks=b.preflight(profile)
                self.assertFalse(checks['passed'])
                self.assertEqual({x['code'] for x in checks['checks'] if not x['ok']},{'DEVICE','PORT'})
                (root/'setup.bash').write_text('changed\n')
                self.assertFalse(next(x for x in b.preflight(profile)['checks'] if x['name']=='release_manifest')['ok'])
                with self.assertRaises(Fault):b.validate(dict(mode='hardware',video=False))
                with self.assertRaises(Fault):b.validate(dict(mode='mock',video=True,video_config='/etc/passwd'))
                with self.assertRaises(Fault):b.validate(dict(mode='mock',video=False,required_modules=['bridge']))

    def test_crashed_owned_process_is_reaped(self):
        with tempfile.TemporaryDirectory(prefix='robot-process-') as folder:
            backend=Backend({},Path(folder),Path(folder))
            proc=subprocess.Popen(['/bin/sleep','30'],start_new_session=True)
            resource=dict(module='bridge',pid=proc.pid,identity=identity(proc.pid),pending=False)
            backend.children[proc.pid]=proc
            try:
                os.killpg(proc.pid,signal.SIGKILL)
                deadline=time.monotonic()+2
                while identity(proc.pid) is not None:
                    self.assertLess(time.monotonic(),deadline);time.sleep(.01)
                backend.stop(resource)
                self.assertIsNotNone(proc.returncode)
            finally:
                if proc.poll() is None:os.killpg(proc.pid,signal.SIGKILL)
                proc.wait()

if __name__=='__main__':unittest.main()
