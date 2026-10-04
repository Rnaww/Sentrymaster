from glob import glob
from setuptools import find_packages, setup

package_name = 'sentry_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Temporary Odin and MCU test inputs for the first precheck.',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': ['test_input_node = sentry_test.test_input_node:main'],
    },
)
