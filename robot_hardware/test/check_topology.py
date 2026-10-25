import subprocess,sys,tempfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
share=Path(get_package_share_directory('robot_hardware'))
with tempfile.TemporaryDirectory(prefix='robot-topology-') as directory:
    for mock in ['false','true']:
        result=subprocess.run(['xacro',str(share/'urdf/robot.urdf.xacro'),'mock:='+mock],capture_output=True,text=True,check=True)
        import xml.etree.ElementTree as E
        root=E.fromstring(result.stdout);controls=root.findall('ros2_control')
        assert len(controls)==2
        assert sum(len(c.findall('joint')) for c in controls)==13
        if mock=='false':
            path=Path(directory)/'robot.urdf';path.write_text(result.stdout)
            subprocess.run([sys.argv[1],str(path)],check=True)
