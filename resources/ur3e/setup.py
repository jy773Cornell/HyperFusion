from setuptools import find_packages, setup
import os
from glob import glob

package_name = "hyperfusion_ur3e"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    install_requires=["PyYAML>=6.0", "numpy>=1.26"],
    zip_safe=True,
    maintainer="HyperFusion",
    maintainer_email="dev@hyperfusion.local",
    description="HyperFusion UR3e WSL sidecar (HTTP + ROS 2 bridge)",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "ur3e_server = hyperfusion_ur3e.sidecar.server:main",
            "joint_states_stamper = hyperfusion_ur3e.nodes.joint_states_stamper:main",
        ],
    },
)
