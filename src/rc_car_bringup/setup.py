from pathlib import Path

from setuptools import find_packages, setup

package_name = "rc_car_bringup"
share_dir = Path("share") / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(share_dir / "launch"), [str(p) for p in sorted(Path("launch").glob("*.launch.py"))]),
        (
            str(share_dir / "config"),
            [
                str(p)
                for p in sorted([*Path("config").glob("*.yaml"), *Path("config").glob("*.rviz")])
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jinwooseo",
    maintainer_email="jinwooseo@todo.todo",
    description="Bringup launch files for RC car Visual SLAM project",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
