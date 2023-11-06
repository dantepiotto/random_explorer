from setuptools import find_packages, setup

package_name = 'rand_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dante',
    maintainer_email='piotto.dante@gmail.com',
    description='random explorer for ros2 humble',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'explorer_node = rand_explorer.explorer:main',
            'timer = rand_explorer.timer:main'
        ],
    },
)
