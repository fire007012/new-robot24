"""Keep commissioned reference limits and named poses intact during motion fixes."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
ROOT=Path(__file__).resolve().parents[2]
NAMES=['shoulder_yaw_joint','shoulder_pitch_joint','elbow_pitch_joint','wrist_pitch_joint','wrist_yaw_joint','wrist_roll_joint']
class ReferenceLimits(unittest.TestCase):
    def test_arm_limits_and_servo_bounds_match_reference(self):
        reference=ET.parse(ROOT/'reference/ros1/car_urdf/urdf/car_urdf.urdf').getroot()
        current=ET.parse(ROOT/'car_urdf/urdf/robot.urdf.xacro').getroot()
        controllers=yaml.safe_load((ROOT/'robot_hardware/config/controllers.yaml').read_text())
        servo=controllers['arm_servo_controller']['ros__parameters']
        for name in NAMES:
            old=reference.find(f"joint[@name='{name}']/limit")
            new=current.find(f"joint[@name='{name}']/limit")
            self.assertEqual(old.attrib,new.attrib,name)
            i=servo['joints'].index(name)
            self.assertEqual(servo['lower'][i],float(old.get('lower')))
            self.assertEqual(servo['upper'][i],float(old.get('upper')))
    def test_named_poses_and_servo_protection_preserved(self):
        old=ET.parse(ROOT/'reference/ros1/car_moveit_config/config/car_urdf.srdf').getroot()
        new=ET.parse(ROOT/'car_moveit_config/config/car_urdf.srdf').getroot()
        urdf=ET.parse(ROOT/'car_urdf/urdf/robot.urdf.xacro').getroot()
        for name in ('zero','home','ready'):
            values=lambda r:{j.get('name'):float(j.get('value')) for j in r.find(f"group_state[@name='{name}']")}
            self.assertEqual(values(old),values(new),name)
            for joint,q in values(new).items():
                limit=urdf.find(f"joint[@name='{joint}']/limit")
                self.assertTrue(float(limit.get('lower'))<=q<=float(limit.get('upper')))
        legacy=yaml.safe_load((ROOT/'reference/ros1/car_control/config/moveit_servo.yaml').read_text())
        current=yaml.safe_load((ROOT/'car_control/config/servo_ros2.yaml').read_text())
        for key in ('lower_singularity_threshold','hard_stop_singularity_threshold','joint_limit_margin'):
            self.assertEqual(current[key],legacy[key],key)
if __name__=='__main__':unittest.main()
