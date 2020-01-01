## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

import distutils.core 
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['panda_robot'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'rospy', 'message_filters', 'sensor_msgs']
)

distutils.core.setup(**setup_args)
