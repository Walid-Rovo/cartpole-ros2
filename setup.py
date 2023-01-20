import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

data_files=[
  ...
  (os.path.join('share', package_name), glob('launch/*.py')),
  (os.path.join('share', package_name), glob('urdf/*'))
],
'console_scripts': [
    'state_publisher = urdf_tutorial_r2d2.state_publisher:main'
],