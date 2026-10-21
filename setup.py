from setuptools import find_packages, setup

package_name = 'RGSE_python'

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
    maintainer='rgse',
    maintainer_email='kilian.roesner@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_list = RGSE_python.position_list:main',
            'haply_main = RGSE_python.haply_main:main',
            'rmi_position_list = RGSE_python.RMI_position_list:main'
        ],
    },
)
