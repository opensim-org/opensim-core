import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

class TestComponentInterface(unittest.TestCase):
    def test_printComponentsMatching(self):
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        num_matches = model.printComponentsMatching("_r")
        self.assertEqual(num_matches, 153)
    def test_attachGeometry_memory_management(self):
        model = osim.Model()
        model.getGround().attachGeometry(osim.Sphere(1.5))
