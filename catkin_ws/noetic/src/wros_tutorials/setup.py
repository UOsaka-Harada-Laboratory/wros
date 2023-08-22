## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['basis',
              'drivers',
              'grasping',
              'helper',
              'manipulation',
              'modeling',
              'motion',
              'visualization',
              'grasping',
              'neuro',
              'robot_con',
              'robot_sim',
              'vision',
              'visualization'],
    package_dir={'': 'modules/wrs'},
    requires=['']
)

setup(**setup_args)