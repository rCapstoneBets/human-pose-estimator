from setuptools import setup
from glob import glob

package_name = 'human_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob("*.tflite") )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coalman321',
    maintainer_email='cjtucker321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_body_pose = human_pose_estimation.human_body_pose:main'
        ],
    },
    py_modules=[
        'human_pose_estimation.data',
        'human_pose_estimation.utils',
        'human_pose_estimation.ml.__init__',
        'human_pose_estimation.tracker.__init__'
    ]
)
