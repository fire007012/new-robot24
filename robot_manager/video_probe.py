"""Bounded, credential-silent RTSP frame verification; no ROS dependency."""
import json
import subprocess
import time

def video_frames(cameras):
    """Manager online flags alone are not evidence of actual video flow."""
    deadline = time.monotonic()+4
    if not cameras or not all(c.get('online') for c in cameras):
        return False, 'no_online_sources'
    for camera in cameras:
        url = camera.get('rtsp_url','')
        if not isinstance(url,str) or not url.startswith(('rtsp://','rtsps://')):
            return False, 'invalid_rtsp_endpoint'
        remaining = deadline-time.monotonic()
        if remaining<=0: return False, 'video_probe_budget'
        try:
            result = subprocess.run(['ffprobe','-v','error','-rtsp_transport','tcp',
                '-read_intervals','%+0.2','-select_streams','v:0','-show_frames',
                '-show_entries','frame=width,height','-of','json',url],
                stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,timeout=min(2,remaining))
            frames=json.loads(result.stdout).get('frames',[])
            if result.returncode or not any(f.get('width',0)>0 and f.get('height',0)>0 for f in frames):
                return False, 'no_video_frames'
        except (OSError,ValueError,subprocess.TimeoutExpired):
            return False, 'video_probe_failed'
    return True, 'rtsp_frame_observed'
