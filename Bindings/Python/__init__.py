import sys
import os

print("at top of init.py")
curFolder = os.path.dirname(os.path.realpath(__file__))
if (sys.platform.startswith('win')):
    os.add_dll_directory(curFolder)
    # When installed locally via "python -m pip install ." in Windows
    if os.path.isfile(os.path.join(curFolder, 'opensim-cmd.exe')):
        # This only sets the PATH for the Python environment (not permanent)
        os.environ['PATH'] = curFolder + os.pathsep + os.environ['PATH']
    # For local testing
    install_path = os.path.join(curFolder, "../../../bin")
    if (os.path.exists(install_path)):
        os.add_dll_directory(install_path)
    dev_path = os.path.join(curFolder, "../../../../Release")
    if (os.path.exists(dev_path)):
        os.add_dll_directory(dev_path)
    # if available tell simbody how to locate the visualizer in layout of python dist
    visualizerPath = os.path.join(curFolder, "bin", "simbody-visualizer.exe")
    if (os.path.exists(visualizerPath)):
        print("Found simbody-visualizer, setting SIMBODY_HOME env var to ", curFolder)
        os.environ["SIMBODY_HOME"]= curFolder 
# Non windows platforms have no .exe extension but visualizer should endup in the same folder
if (sys.platform.startswith('darwin')):
    os.environ['SIMBODY_HOME']= curFolder
    os.environ['PATH']= curFolder+ os.pathsep + os.environ['PATH']
    os.environ['DYLD_LIBRARY_PATH']= os.environ['PATH']
    visualizer_filepath = curFolder+"/simbody-visualizer.app/Contents/MacOS/simbody-visualizer"
    if (os.path.exists(visualizer_filepath)):
        if not os.access(visualizer_filepath, os.X_OK):
            os.chmod(visualizer_filepath, 0o777)
    

if (sys.platform.startswith('linux')):
    # print("Linux platform detected, setting SIMBODY_HOME env var to ", curFolder)
    os.environ["SIMBODY_HOME"]= curFolder
    os.environ["PATH"]= curFolder+ os.pathsep + os.environ['PATH']
    os.environ['LD_LIBRARY_PATH']= os.environ['PATH']
    visualizer_filepath = curFolder+"/simbody-visualizer"
    if (os.path.exists(visualizer_filepath)):
        if not os.access(visualizer_filepath, os.X_OK):
            os.chmod(visualizer_filepath, 0o777)

#curFolder = os.path.dirname(os.path.realpath(__file__))
#os.environ['PATH'] = curFolder + os.pathsep + os.environ['PATH']

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

geometry_path = os.path.join(curFolder, 'Geometry')
if os.path.exists(geometry_path):
    ModelVisualizer.addDirToGeometrySearchPaths(geometry_path)

import copy

# TODO consider moving this to the interface files.
def declare_concrete_object(original_class):
    """This method should be used as a decorator to achieve the similar
    behavior as the C++ OpenSim_DECALRE_CONCRETE_OBJECT macro.

    """

    # Add abstract methods.
    def getConcreteClassName(self):
        return original_class.__name__
    def clone(self):
        # TODO this implementation is incorrect!
        print "DEBUG", type(original_class().__disown__()), "ENDDEBUG"
        #print "DEBUG", type(copy.deepcopy(self)), "ENDDEBUG"
        #return None
        #return copy.deepcopy(self) #None
        obj = self.__class__()
        obj._markAdopted()
        #return obj #original_class().__disown__()
        return original_class().__disown__()

    original_class.getConcreteClassName = getConcreteClassName
    original_class.clone = clone

    # TODO getClassName()
    # TODO safeDownCast()
    # TODO Self
    # TODO Super

    return original_class

