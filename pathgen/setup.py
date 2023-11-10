from setuptools import find_packages, setup

package_name = 'pathgen'

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
    maintainer='Ashutosh Gupta',
    maintainer_email='59316418+Ashutosh781@users.noreply.github.com',
    description='Create robot path from svg image',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'svg2path_service = pathgen.svg2path_service:main',
            'svg2path_client = pathgen.svg2path_client:main',
        ],
    },
)
