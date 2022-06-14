import os
from glob import glob

from setuptools import setup

package_name = "ld06_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "models/ld06"), glob("models/ld06/*.sdf")),
        (os.path.join("share", package_name, "models/ld06"), glob("models/ld06/*.config")),
        (os.path.join("share", package_name, "models/ld06/meshes"), glob("models/ld06/meshes/*")),
        (os.path.join("share", package_name, "description/urdf"), glob("description/urdf/*.urdf")),
        (os.path.join("share", package_name, "description/meshes"), glob("description/meshes/*")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshiyuki Oshima",
    maintainer_email="toshiyuki67026@gmail.com",
    description="ld06_sim",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "spawn_model=ld06_sim.spawn_model:main",
        ],
    },
)
