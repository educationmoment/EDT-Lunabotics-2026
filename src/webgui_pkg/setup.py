from setuptools import find_packages, setup

package_name = 'webgui_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=['webgui_pkg'],
    package_data={
        'webgui_pkg': [
            'templates/*.html',
            'static/**/*',
            'static/**/**/*',
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'Flask'],
    zip_safe=True,
    maintainer='edt',
    maintainer_email='abaja5@uic.edu',
    description='Lunabotics web GUI – pilot, engineer, networking views',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Run with: ros2 run webgui_pkg webgui_server
            # Or set rosbridge URL: ROSBRIDGE_URL=ws://192.168.1.100:9090 ros2 run webgui_pkg webgui_server
            'webgui_server = webgui_pkg.app:main',
        ],
    },
)