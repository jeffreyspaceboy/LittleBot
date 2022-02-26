from setuptools import setup
from setuptools import find_packages

package_name = 'little_bot_2d_sim'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeffrey Fisher II',
    maintainer_email='jeffreyspaceboy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'little_bot_2d_sim_node = little_bot_2d_sim.little_bot_2d_sim_node:main'
        ],
    },
)
