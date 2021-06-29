import sys
import os
from pathlib import Path
if (sys.version_info.major == 3 and sys.version_info.minor >= 8 and sys.platform.startswith('win')):
        if (Path('opensim/dll_location.txt').is_file()):
            with open('opensim/dll_location.txt', 'r') as reader:
                os.add_dll_directory(reader.read())
        else:
            os.add_dll_directory(DLL_PATH)

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
