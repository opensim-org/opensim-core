import sys
import os
if (sys.platform.startswith('win')):
      curFolder = os.path.dirname(os.path.realpath(__file__))
      # print("adding dll_directory "+os.path.dirname(os.path.realpath(__file__)))
      os.add_dll_directory(curFolder)
      # in dev environment, we also add "../../../../Release"
      dev_path = os.path.join(curFolder, "../../../../Release")
      if (os.path.exists(dev_path)):
           os.add_dll_directory(dev_path)

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
