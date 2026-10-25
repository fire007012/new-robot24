#!/usr/bin/env python3
"""Record reproducible source/artifact fingerprints and installed dependency versions."""
import hashlib,json,subprocess,sys
from pathlib import Path
source,prefix,output=map(Path,sys.argv[1:])
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
report={'source_base_commits':{},'source_sha256':{},'generated_sha256':{},'dependencies':{}}
for package in ['Eyou_Canopen_Master','ros2_canopen','can_driver']:
 report['source_base_commits'][package]=subprocess.check_output(['git','-C',str(source/package),'rev-parse','HEAD'],text=True).strip()
for package in ['robot_safety','robot_hardware','robot_canopen_config']:
 for path in sorted((source/package).rglob('*')):
  if path.is_file() and '__pycache__' not in path.parts:report['source_sha256'][str(path.relative_to(source))]=sha(path)
for package in ['ros2_canopen','can_driver']:
 paths=subprocess.check_output(['git','-C',str(source/package),'diff','--name-only','HEAD'],text=True).splitlines()
 paths+=subprocess.check_output(['git','-C',str(source/package),'ls-files','--others','--exclude-standard'],text=True).splitlines()
 for name in sorted(set(paths)):
  path=source/package/name
  if path.is_file():report['source_sha256'][str(path.relative_to(source))]=sha(path)
for path in sorted((prefix/'robot_canopen_config/share/robot_canopen_config/config').rglob('*')):
 if path.is_file() and path.suffix in ['.bin','.dcf','.yml']:report['generated_sha256'][str(path.relative_to(prefix))]=sha(path)
for package in ['ros-humble-controller-manager','ros-humble-hardware-interface','ros-humble-lely-core-libraries','ros-humble-joint-trajectory-controller','ros-humble-rclcpp']:
 report['dependencies'][package]=subprocess.check_output(['dpkg-query','-W','-f=${Version}',package],text=True).strip()
output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(output)
