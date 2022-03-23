from glob import glob
from setuptools import setup

package_name = "lilbot_webots_sim_pkg"

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name + "/launch", glob("launch/*")))
data_files.append(("share/" + package_name + "/resource/meshes", glob("resource/meshes/*")))
data_files.append(("share/" + package_name + "/resource/worlds", glob("resource/worlds/*")))
data_files.append(("share/" + package_name + "/resource/urdf", glob("resource/urdf/*")))
data_files.append(("share/" + package_name, ["package.xml"]))

setup(
    name = package_name,
    version = "0.0.0",
    packages = [package_name],
    data_files = data_files,
    install_requires = ["setuptools"],
    maintainer = "Jeffrey Fisher II",
    maintainer_email = "jeffreyspaceboy@gmail.com",
    description = "This ROS2 package allows for 3D simulation of lilbot using webots. This package is mainly for development.",
    license = "Apache License 2.0",
    entry_points = {
        "console_scripts": [
            "lilbot_webots_driver = lilbot_webots_sim_pkg.lilbot_webots_driver:main",
        ],
    },
)