from setuptools import setup

package_name = 'lab3_wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bakhbyergyen Yerjan',
    maintainer_email='bakhbyergyen7@gmail.com',
    description='Lab3: Wall Follow',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab3_wall_follow = scripts.lab3_wall_follow:main',
        ],
    },
)
