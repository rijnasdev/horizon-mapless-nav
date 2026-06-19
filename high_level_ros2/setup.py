from setuptools import setup
from glob import glob
import os

package_name = 'mapless_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/mapless_navigation']),
        ('share/mapless_navigation',
            ['package.xml']),

        (
            os.path.join(
                'share',
                'mapless_navigation',
                'launch'
            ),
            glob('launch/*.py')
        ),

        (
            os.path.join(
                'share',
                'mapless_navigation',
                'urdf'
            ),
            glob('urdf/*')
        ),

        (
            os.path.join(
                'share',
                'mapless_navigation',
                'worlds'
            ),
            glob('worlds/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Anchery',
    maintainer_email='etcetra7n@gmail.com',
    description='Mars rover navigation node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'perception_node = mapless_navigation.perception_node:main',
        ],
    },
)
