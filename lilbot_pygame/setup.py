from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'lilbot_pygame'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/resource/images', glob('resource/images/*')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    maintainer='Jeffrey Fisher II',
    maintainer_email='jeffreyspaceboy@gmail.com',
    description='Pygame package for simple 2D simulation testing.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lilbot_pygame_node = lilbot_pygame.lilbot_pygame_node:main'
        ],
    },
)
