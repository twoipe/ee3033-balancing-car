from glob import glob
from setuptools import setup

setup(
    name='ee3033_search', version='10.0.0',
    packages=['ee3033_search'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ee3033_search']),
        ('share/ee3033_search', ['package.xml']),
        ('share/ee3033_search/launch', glob('launch/*.launch.py')),
        ('share/ee3033_search/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='EE3033 project group', maintainer_email='not-provided@example.invalid',
    description='EE3033 autonomous coverage search with camera-bearing triangulation', license='MIT',
    entry_points={'console_scripts': ['search = ee3033_search.mission_node:main']},
)
