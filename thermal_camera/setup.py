from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[],
    scripts=['scripts/thermal_camera_node.py'],
)

setup(**d)
