"""Test Simbody classes.

"""

import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestSimbody(unittest.TestCase):
    def test_SimbodyMatterSubsystem(self):
        model = osim.Model(os.path.join(test_dir,
            "gait10dof18musc_subject01.osim"))
        s = model.initSystem()
        smss = model.getMatterSubsystem()
        
        assert smss.calcSystemMass(s) == model.getTotalMass(s)
        assert (smss.calcSystemMassCenterLocationInGround(s)[0] == 
                model.calcMassCenterPosition(s)[0])
        assert (smss.calcSystemMassCenterLocationInGround(s)[1] == 
                model.calcMassCenterPosition(s)[1])
        assert (smss.calcSystemMassCenterLocationInGround(s)[2] == 
                model.calcMassCenterPosition(s)[2])

        J = osim.Matrix()
        smss.calcSystemJacobian(s, J)
        # 6 * number of mobilized bodies
        assert J.nrow() == 6 * (model.getBodySet().getSize() + 1)
        assert J.ncol() == model.getCoordinateSet().getSize()

        v = osim.Vec3()
        smss.calcStationJacobian(s, 2, v, J)
        assert J.nrow() == 3
        assert J.ncol() == model.getCoordinateSet().getSize()

