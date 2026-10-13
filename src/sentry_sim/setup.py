import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'sentry_sim'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.py')),
]
for directory in ['models', 'worlds', 'robots', 'config']:
    for path, directories, filenames in os.walk(directory):
        data_files.append((os.path.join('share', package_name, path), [os.path.join(path, name) for name in filenames]))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='701 navigation and SentryMaster scoring simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sentry_simulator = sentry_sim.simulator_node:main',
        ],
    },
)
