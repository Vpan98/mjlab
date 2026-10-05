import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'unitree_g1_mujoco'


def collect_data_files(directory):
    """Walk `directory` and return (install_dir, [file]) pairs that preserve
    the source tree layout under share/<package_name>/, so relative asset
    references inside the MJCF files (e.g. meshdir="../meshes") keep working
    once installed.
    """
    pairs = []
    for path, _dirs, filenames in os.walk(directory):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_dir = os.path.join('share', package_name, path)
            pairs.append((install_dir, [file_path]))
    return pairs


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
]
data_files += collect_data_files('mjcf')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Vishnu',
    maintainer_email='vishnu@example.com',
    description='MuJoCo simulation and exercise scripts for the Unitree G1 humanoid, bridged to ROS 2 topics.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation = unitree_g1_mujoco.simulation:main',
            'exercise = unitree_g1_mujoco.exercise:main',
            'walk = unitree_g1_mujoco.walk:main',
            'crunches = unitree_g1_mujoco.crunches:main',
        ],
    },
)
