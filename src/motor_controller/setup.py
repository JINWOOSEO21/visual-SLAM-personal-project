from pathlib import Path

from setuptools import find_packages, setup

package_name = "motor_controller"
share_dir = Path("share") / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(share_dir / "launch"), [str(p) for p in sorted(Path("launch").glob("*.launch.py"))]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jinwooseo",
    maintainer_email="jinuseo21@gmail.com",
    description="L298N motor driver node: /cmd_vel -> GPIO PWM for RC car",
    license="MIT",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "motor_driver_node = motor_controller.motor_driver_node:main",
        ],
    },
)
