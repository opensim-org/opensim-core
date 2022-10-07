import sys
import os

if sys.version_info.major == 3 and sys.version_info.minor >= 8:
    os.add_dll_directory(os.path.dirname(os.path.realpath(__file__)))

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
