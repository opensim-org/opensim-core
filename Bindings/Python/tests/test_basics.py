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

        fwpm = muscle.getPennationModel()
        fwpm.get_optimal_fiber_length()

        adm = muscle.getActivationModel()
        adm.get_activation_time_constant()

    def test_SimTKArray(self):
        # Initally created to test the creation of a separate simbody module.
        ad = osim.SimTKArrayDouble()
        ad.push_back(1)

        av3 = osim.SimTKArrayVec3()
        av3.push_back(osim.Vec3(8))
        assert av3.at(0).get(0) == 8

    def test_ToolAndModel(self):
        # Test tools module.
        cmc = osim.CMCTool()
        model = osim.Model()
        model.setName('alphabet')
        cmc.setModel(model)
        assert cmc.getModel().getName() == 'alphabet'

    def test_AnalysisToolModel(self):
        # Test analyses module.
        cmc = osim.CMCTool()
        model = osim.Model()
        model.setName('eggplant')
        fr = osim.ForceReporter()
        fr.setName('strong')
        cmc.setModel(model)
        cmc.getAnalysisSet().adoptAndAppend(fr)

        assert cmc.getModel().getName() == 'eggplant'
        assert cmc.getAnalysisSet().get(0).getName() == 'strong'




