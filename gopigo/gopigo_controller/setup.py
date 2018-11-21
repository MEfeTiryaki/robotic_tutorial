#!/usr/bin/env python


packages = ['gopigo_controller']
package_dir = {'': 'scripts'}
try:
	from distutils.core import setup
	from catkin_pkg.python_setup import generate_distutils_setup
	d = generate_distutils_setup(
		packages=packages,
  		package_dir=package_dir
  		)

	setup(**d)
except:
	from setuptools import setup
	setup(packages=packages,
  		  package_dir=package_dir)
