#
# Copyright (c) 2019 Houston Mechatronics Inc.
#
# Distribution of this file or its parts, via any medium is strictly
# prohibited. Permission to use must be explicitly granted by Houston
# Mechatronics Inc.
#

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

CONFIG = generate_distutils_setup(
    packages=['ntrip_ros'],
    package_dir={'': 'src'}
)

setup(**CONFIG)
