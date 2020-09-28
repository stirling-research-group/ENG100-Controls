from os import sys
from cx_Freeze import setup, Executable

base = None

if sys.platform == "win32":
    base = "Win32GUI"

executables = [Executable('GUI.py', base=base, targetName='ENG100Controls.exe')]

include_files = ['packages/mpl_toolkits',
                 'Documentation/',
                 'favicon.png']
packages = ['matplotlib', 'numpy', 'pandas', 'scipy', 'scipy.integrate',
            'scipy.spatial.ckdtree', 'time', 'tkinter']
options = {
    'build_exe': {    
        'packages': packages, 
        'include_files': include_files,
        'excludes': ['scipy.spatial.cKDTree', 'matplotlib.tests', 'numpy.random._examples']
    },
}

setup(
    name = 'ENG100 Controls',
    options = options,
    version = '1.0',
    description = '',
    executables = executables
)