import os
from glob import glob
from setuptools import find_packages, setup

package_name = "wifibot_control"

def collect_files(directory):
    data = []
    path = os.path.join(directory, "**", "*")
    for filename in glob(path, recursive=True):
        if os.path.isfile(filename):
            destination = os.path.join(
                "share",
                package_name,
                os.path.dirname(filename)
            )
            data.append((destination, [filename]))
    return data

data_files = [
    (
        "share/ament_index/resource_index/packages",
        ["resource/" + package_name]
    ),
    (
        "share/" + package_name,
        ["package.xml"]
    ),
]

for directory in ['launch', 'config']:
    data_files.extend(collect_files(directory))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="developer@example.com",
    description="Wifibot ROS 2 package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ['cmd_vel_adapter = wifibot_control.cmd_vel_adapter:main'],
    },
)
