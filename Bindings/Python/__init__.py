import sys
import os
if (sys.version_info.major == 3 and sys.version_info.minor >= 8):
    cwd = os.getcwd()
    # print(cwd)
    print('Writing new dll_location.txt conatining '+cwd)
    with open('opensim/dll_location.txt', 'r') as reader:
        os.add_dll_directory(reader.read())

from .simbody import *
from .common import *
from .simulation import *
from .actuators import *
from .analyses import *
from .tools import *
from .examplecomponents import *
from .moco import *
from . import report

from .version import __version__
