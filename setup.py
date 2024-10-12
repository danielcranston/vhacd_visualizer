from setuptools import find_packages, setup

package_name = "vhacd_visualizer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            f"share/{package_name}/config",
            ["config/vhacd_parameter_descriptions.yaml", "config/vhacd_visualizer.rviz"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/vhacd_visualizer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Daniel",
    maintainer_email="cranston.daniel@gmail.com",
    description="Interactively visualize convex decompositions of your OBJ files",
    license="MIT",
    entry_points={
        "console_scripts": [
            "vhacd_visualizer = vhacd_visualizer.vhacd_visualizer:main",
        ],
    },
)
