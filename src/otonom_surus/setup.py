import os
from glob import glob
from setuptools import setup

package_name = 'otonom_surus'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
]

# Gazebo modelleri (models/<model_adi>/model.sdf + model.config)
for model_dir in glob('models/*'):
    data_files.append(
        (os.path.join('share', package_name, model_dir), glob(model_dir + '/*'))
    )

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hasan',
    maintainer_email='hasan@todo.todo',
    description='TurtleBot3 ile Gazebo\'da otonom sürüş dönem projesi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = otonom_surus.teleop:main',
            'subscriber = otonom_surus.subscriber:main',
            'navigator = otonom_surus.otonom_navigator:main',
            'kesif = otonom_surus.otonom_kesif:main',
        ],
    },
)
