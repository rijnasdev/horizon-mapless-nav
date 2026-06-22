from setuptools import setup
from glob import glob
import os

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.1',
    zip_safe=True,
    maintainer='John Anchery',
    maintainer_email='etcetra7n@gmail.com',
    description='Mars rover navigation node',
    license='MIT',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/perception']),
        ('share/perception',
            ['package.xml']),

        (
            os.path.join(
                'share',
                'perception',
                'launch'
            ),
            glob('launch/*.py')
        ),

        (
            os.path.join(
                'share',
                'perception',
                'urdf'
            ),
            glob('urdf/*')
        ),

        (
            os.path.join(
                'share',
                'perception',
                'worlds'
            ),
            glob('worlds/*')
        ),
    ],
    install_requires=['setuptools'],

    entry_points={
        'console_scripts': [
            'watchdog = perception.watchdog:main',
            'visualize = perception.visualize:main',
        ],
    },
)
