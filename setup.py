from os import sys
from cx_Freeze import setup, Executable

base = None

if sys.platform == "win32":
    base = "Win32GUI"

executables = [Executable('QtGUI.py', base=base, target_name='OpenClosedLoop.exe')]

include_files = ['help_files/',
                 'packages/PyQt5',
                 'AboutWindow.ui',
                 'HelpWindow.ui',
                 'MainWindow.ui',
                 'Styling.qss',
                 'favicon.png']
packages = ['matplotlib', 'mpl_toolkits', 'numpy', 'PyQt5', 'scipy', 'scipy.integrate', 'scipy.spatial.ckdtree', 'time']
options = {
    'build_exe': {    
        'packages': packages, 
        'include_files': include_files,
        'excludes': ['scipy.spatial.cKDTree', 'matplotlib.tests', 'numpy.random._examples']
    },
}

setup(
    name = 'Open and Closed Loop GUI',
    options = options,
    version = '1.0',
    description = '',
    executables = executables
)