## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    package_dir={'': 'wrs'},
    packages=['basis',
              'drivers',
              'helper',
              'manipulation',
              'modeling',
              'motion',
              'grasping',
              'neuro',
              'robot_con',
              'robot_sim',
              'vision',
              'visualization',
              'trimesh',
              'pyhiro'],
)

setup(**setup_args)