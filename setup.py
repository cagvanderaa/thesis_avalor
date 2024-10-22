from setuptools import setup, find_packages

setup(
    name='offboard_py',
    version='0.0.1',
    packages=find_packages(),  # Automatically find packages, including src/
    package_dir={'offboard_py': 'src/offboard_py'},  # Tell setup.py where to find the Python code
    package_data={
        'offboard_py': ['cfg/*.cfg'],  # You can add any additional files here if needed
    },
    scripts=['scripts/target.py', 'scripts/velocity.py', 'scripts/analysis.py', 'scripts/main.py', 'scripts/tracker.py', 'scripts/image_resizer.py'],  # Executable scripts
    install_requires=['rospy', 'dynamic_reconfigure'],
)
