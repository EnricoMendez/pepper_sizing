from setuptools import find_packages, setup

package_name = 'size_estimation'

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
    maintainer='enrico',
    maintainer_email='enrico.mendez@outlook.es',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_adquisition = size_estimation.data_adquisition:main',
            'image_porcessing_server = size_estimation.image_processing_server:main',
        ],
    },
)
