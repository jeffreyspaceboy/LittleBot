from glob import glob
from setuptools import setup

package_name = "lilbot_webots_sim_pkg"
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name + "/launch", glob("launch/*")))
data_files.append(("share/" + package_name + "/resource/meshes", glob("resource/meshes/*")))
data_files.append(("share/" + package_name + "/resource/protos", glob("resource/protos/*")))
data_files.append(("share/" + package_name + "/resource/worlds", glob("resource/worlds/*")))
data_files.append(("share/" + package_name + "/resource/urdf", glob("resource/urdf/*")))
data_files.append(("share/" + package_name, ["package.xml"]))

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user.name@mail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "lilbot_webots_driver = lilbot_webots_sim_pkg.lilbot_webots_driver:main",
            "obstacle_avoider = lilbot_webots_sim_pkg.obstacle_avoider:main"
        ],
    },
)