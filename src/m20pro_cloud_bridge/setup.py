from setuptools import setup

package_name = "m20pro_cloud_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="M20Pro Developer",
    maintainer_email="user@example.com",
    description="Local web dashboard and cloud bridge prototype for the M20 Pro runtime state.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_dashboard = m20pro_cloud_bridge.web_dashboard_node:main",
        ],
    },
)
