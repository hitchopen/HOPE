from setuptools import find_packages, setup

package_name = 'audio_examples_sender'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hsh',
    maintainer_email='yanxiaolong@agibot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_pub = audio_examples_sender.audio_publish_example:main',
        ],
    },
)
