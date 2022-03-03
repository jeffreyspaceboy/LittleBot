from setuptools import setup

package_name = "lilbot_webot_sim"
data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name + "/launch", ["launch/lilbot_launch.py"]))
data_files.append(("share/" + package_name + "/worlds", ["worlds/lilbot_world.wbt"]))
data_files.append(("share/" + package_name + "/resource", ["resource/lilbot.urdf"]))
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
            "lilbot_driver = lilbot_webot_sim.lilbot_driver:main",
        ],
    },
)