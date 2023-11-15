from setuptools import find_packages, setup

package_name = 'cv_pipeline'

setup(
  name=package_name,
  version='0.0.0',
  packages=find_packages(exclude=['test']),
  data_files=[
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='Andrew Dassonville',
  maintainer_email='dassonva@oregonstate.edu',
  description='Convert camera image to drawable vector image.',
  license='MIT',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'cv_pipeline_service = cv_pipeline.cv_pipeline_service:main',
      'cv_pipeline_client = cv_pipeline.cv_pipeline_client:main',
    ],
  },
)
