import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'tidybot_sim'

setup(
    name=package_name,
    version='0.1.0',
    # find_packages() auto-discovers the tidybot_sim/ Python subpackage
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required: registers this package with the ament index so ROS can find it
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Required: installs package.xml into the share directory
        ('share/' + package_name, ['package.xml']),

        # Install all launch files → share/tidybot_sim/launch/
        # get_package_share_directory() will point here at runtime
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),

        # Install URDF/xacro files → share/tidybot_sim/description/
        (os.path.join('share', package_name, 'description'),
            glob('description/*.xacro')),

        # Install world SDF files → share/tidybot_sim/worlds/
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),

        # Install config files (bridge.yaml, RViz) → share/tidybot_sim/config/
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ashvin Anilkumar',
    maintainer_email='anshvinanilkumarsh@gmail.com',
    description='Home-tidying robot simulation (Drift assignment)',
    license='MIT',
    # Console scripts: maps a CLI command name to a Python function.
    # After colcon build, these become runnable as:
    #   ros2 run tidybot_sim navigator
    #   ros2 run tidybot_sim arm_controller
    #   ros2 run tidybot_sim logger
    # (We'll add the actual Python files in Phases 4 and 5)
    entry_points={
        'console_scripts': [
            'navigator      = tidybot_sim.navigator:main',
            'arm_controller = tidybot_sim.arm_controller:main',
            'logger         = tidybot_sim.logger:main',
        ],
    },
)
