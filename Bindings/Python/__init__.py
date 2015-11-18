from simbody import *
from common import *
from simulation import *
from actuators import *
from analyses import *
from tools import *

# TODO consider moving this to the interface files.
def declare_concrete_object(original_class):
    """This method should be used as a decorator to achieve the similar
    behavior as the C++ OpenSim_DECALRE_CONCRETE_OBJECT macro.

    """

    # Add abstract methods.
    def getConcreteClassName(self):
        return original_class.__name__
    def clone(self):
        return original_class().__disown__()

    original_class.getConcreteClassName = getConcreteClassName
    original_class.clone = clone

    # TODO getClassName()
    # TODO safeDownCast()
    # TODO Self
    # TODO Super

    return original_class
