from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['tamu_sa', 'tamu_sa.animation', 'tamu_sa.graphs', 'tamu_sa.search'],
    package_dir={'': 'src'}
)
setup(**d)
