import json
from pathlib import Path
import subprocess
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from video_probe import video_frames

class Tests(unittest.TestCase):
    def test_online_flag_requires_actual_frames(self):
        camera=dict(online=True,rtsp_url='rtsp://127.0.0.1:1/test')
        with patch('video_probe.subprocess.run',return_value=subprocess.CompletedProcess([],0,b'{"frames":[]}')):
            self.assertFalse(video_frames([camera])[0])
        response=json.dumps(dict(frames=[dict(width=640,height=480)])).encode()
        with patch('video_probe.subprocess.run',return_value=subprocess.CompletedProcess([],0,response)):
            self.assertTrue(video_frames([camera])[0])
        self.assertFalse(video_frames([])[0])
    def test_unreachable_rtsp_is_unhealthy(self):
        self.assertFalse(video_frames([dict(online=True,rtsp_url='rtsp://127.0.0.1:1/test')])[0])

if __name__=='__main__':unittest.main()
