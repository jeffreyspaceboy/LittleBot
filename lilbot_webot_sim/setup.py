from glob import glob
from setuptools import setup

package_name = "lilbot_webot_sim"

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name + "/launch", glob("launch/*")))
data_files.append(("share/" + package_name + "/worlds", glob("worlds/*")))
data_files.append(("share/" + package_name + "/meshes", glob("meshes/*")))
data_files.append(("share/" + package_name + "/protos", glob("protos/*")))
data_files.append(("share/" + package_name + "/rviz", glob("rviz/*")))
data_files.append(("share/" + package_name + "/resource", glob("resource/*")))
data_files.append(("share/" + package_name, ["package.xml"]))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools", "launch"],
    zip_safe=True,
    keywords=["ROS2", "Webots", "Simulation", "Simple"],
    maintainer="Jeffrey Fisher II",
    maintainer_email="jeffreyspaceboy@gmail.com",
    description="Webots simulation for the Little Bot project. This package is meant to allow for faster development while working on the project.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "enable_robot = lilbot_webot_sim.slave:main",
            "line_follower = lilbot_webot_sim.master:main",
            "odom_publisher = lilbot_webot_sim.odom_publisher:main"
        ],
        "launch.frontend.launch_extension": ["launch_ros = launch_ros"],
    },
)