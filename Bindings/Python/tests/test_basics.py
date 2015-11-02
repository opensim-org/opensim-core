"""These are basic tests to make sure that C++ classes were wrapped properly.
There shouldn't be any python-specifc tests here.

"""

import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestBasics(unittest.TestCase):
    def test_Thelen2003Muscle_helper_classes(self):
        # This test exists because some classes that Thelen2003Muscle used were
        # not accessibly in the bindings.
        muscle = osim.Thelen2003Muscle()

        fwpm = muscle.get_MuscleFixedWidthPennationModel()
        fwpm.get_optimal_fiber_length()

        adm = muscle.get_MuscleFirstOrderActivationDynamicModel()
        adm.get_activation_time_constant()





