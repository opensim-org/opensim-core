import sys
import os
if (sys.platform.startswith('win')):
    os.add_dll_directory(curFolder)
    os.add_dll_directory(os.path.join(os.path.dirname(sys.executable),'Scripts'))

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
