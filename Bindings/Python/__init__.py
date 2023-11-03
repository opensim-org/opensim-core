import sys
import os
if (sys.platform.startswith('win')):
    curFolder = os.path.dirname(os.path.realpath(__file__))
    os.add_dll_directory(curFolder)
    opensimCMD = os.path.join(os.path.dirname(sys.executable), 'Scripts', 'opensim-cmd.exe')
    if os.path.isfile(opensimCMD):
        os.add_dll_directory(os.path.dirname(opensimCMD))

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
