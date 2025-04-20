from setuptools import setup
import os
from glob import glob

package_name = 'fcuflex_plotter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This is the key part - make sure it's exactly like this:
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Data plotter and logger for FCUFLEX device',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fcuflex_plotter = fcuflex_plotter.fcuflex_plotter:main',
        ],
    },
)