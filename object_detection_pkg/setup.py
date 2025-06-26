from setuptools import find_packages, setup

package_name = 'obstacle_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=['obstacle_detection'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tarek Abdelmeguid',
    maintainer_email='tar5361s@hs-coburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
        'obstacle_detection = obstacle_detection.obstacle_detection:main',
    ],
},
)
