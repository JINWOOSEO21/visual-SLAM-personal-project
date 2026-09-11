from pathlib import Path

from setuptools import find_packages, setup

package_name = "pid_velocity_controller"
share_dir = Path("share") / package_name

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (str(share_dir / "launch"), [str(p) for p in sorted(Path("launch").glob("*.launch.py"))]),
        (str(share_dir / "config"), [str(p) for p in sorted(Path("config").glob("*.yaml"))]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jinwooseo",
    maintainer_email="jinuseo21@gmail.com",
    description="Closed-loop wheel velocity PID controller for RC car",
    license="MIT",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "pid_controller_node = pid_velocity_controller.pid_controller_node:main",
            "step_cmd_vel_node = pid_velocity_controller.step_cmd_vel_node:main",
            "analyze_pid_step_response = pid_velocity_controller.analyze_pid_step_response:main",
        ],
    },
)
