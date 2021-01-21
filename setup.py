from os import sys
from cx_Freeze import setup, Executable

base = None

if sys.platform == "win32":
    base = "Win32GUI"

executables = [Executable('QtGUI.py', base=base, targetName='ENG100Controls.exe')]

include_files = ['packages/mpl_toolkits',
                 'Documentation/',
                 'favicon.png']
packages = ['matplotlib', 'numpy', 'pandas', 'PyQt5.QtCore', 'PyQt5.QtWidgets',
            'PyQt5.uic', 'scipy', 'scipy.integrate', 'scipy.spatial.ckdtree',
            'time']
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