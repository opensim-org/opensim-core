import sys
import os

if sys.platform.startswith('win'):
    curFolder = os.path.dirname(os.path.realpath(__file__))
    os.add_dll_directory(curFolder)
    # in install env, add relative path from init.py to bin, in build environment, we also add "../../../../Release"
    try: # pass only if DLL_PATH is updated by setup.py in local installation
        bin_path = DLL_PATH
        os.add_dll_directory(bin_path)
        os.environ["PATH"] = bin_path+os.pathsep+os.environ["PATH"] # add to the beginning
        dev_path = os.path.join(bin_path, "../Release")
        if (os.path.exists(dev_path)):
            os.add_dll_directory(dev_path)
    except:
        pass

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
