from setuptools import setup, find_packages

with open('requirements.txt') as f:
    required = f.read().splitlines()


# find_packages(exclude=["*.tests", "*.tests.*", "tests.*", "tests"])
setup(
    packages=find_packages('src'),  # include all packages under src
    package_dir={'':'src'},   # tell distutils packages are under src
    scripts = ['src/aml_calib/test_hand_eye_calib.py'],

    include_package_data=True,    # include everything in source control

    # ...but exclude README.txt from all packages
    # exclude_package_data={'': ['README.txt']},
    package_data = { 'aml_calib': ['resources/*']},
    name='aml-tools',
    version='0.0.1',
    description='AML Tools',
    author='Ermano Arruda',
    author_email='ermano.arruda@gmail.com',
    url='https://todo.com',
    install_requires=required,
    #install_requires=['numpy','numpy-quaternion','pybullet','PyYAML','serial','future'],

)