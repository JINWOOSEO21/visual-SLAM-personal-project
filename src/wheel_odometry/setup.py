from setuptools import find_packages, setup

package_name = "wheel_odometry"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/odometry.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jinwooseo",
    maintainer_email="jinwooseo@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "encoder_node  = wheel_odometry.encoder_node:main",
            "odometry_node = wheel_odometry.odometry_node:main",
        ],
    },
)
