import os
from glob import glob
from setuptools import setup

package_name = 'golf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Salim',
    maintainer_email='Salimjbara@gmail.com',
    description='Runs Autonomous Golf Ball Search and Rescue Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pyIMUpub = golf.pyIMUpub:main',
            'pyMAGpub = golf.pyMAGpub:main',
            'pyGPSpub = golf.pyGPSpub:main',
            'pyMotor = golf.pyMotor:main'
        ],
    },
)