from setuptools import setup

package_name = 'robot_tools'

# This file is decorative for an ament_cmake package — the actual install is
# driven by CMakeLists.txt (ament_python_install_package + install(PROGRAMS)).
# Kept in sync so editor/lint tooling that introspects setup.py still works.
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrei Dragomir',
    maintainer_email='andrei.dragomir@ldrobots.ro',
    description='PyQt GUIs for the whole-body PVT stack (tuner + dashboard).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pvt_tuner = robot_tools.pvt_tuner:main',
            'pvt_dashboard = robot_tools.pvt_dashboard:main',
            'bus_voltage_viewer = robot_tools.bus_voltage_viewer:main',
        ],
    },
)
