"""Test Simbody classes.

"""

import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

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

        # Inverse dynamics from SimbodyMatterSubsystem.calcResidualForce().
        # For the given inputs, we will actually be computing the first column
        # of the mass matrix. We accomplish this by setting all inputs to 0
        # except for the acceleration of the first coordinate.
        #   f_residual = M udot + f_inertial + f_applied 
        #              = M ~[1, 0, ...] + 0 + 0
        model.realizeVelocity(s)
        appliedMobilityForces = osim.Vector()
        appliedBodyForces = osim.VectorOfSpatialVec()
        knownUdot = osim.Vector(s.getNU(), 0.0); knownUdot[0] = 1.0
        knownLambda = osim.Vector()
        residualMobilityForces = osim.Vector()
        smss.calcResidualForce(s, appliedMobilityForces, appliedBodyForces,
                          knownUdot, knownLambda, residualMobilityForces)
        assert residualMobilityForces.size() == s.getNU()

        # Explicitly compute the first column of the mass matrix, then copmare.
        massMatrixFirstColumn = osim.Vector() 
        smss.multiplyByM(s, knownUdot, massMatrixFirstColumn)
        assert massMatrixFirstColumn.size() == residualMobilityForces.size()
        for i in range(massMatrixFirstColumn.size()):
            self.assertAlmostEqual(massMatrixFirstColumn[i],
                                   residualMobilityForces[i])

        # InverseDynamicsSolver.
        # Using accelerations from forward dynamics should give 0 residual.
        model.realizeAcceleration(s)
        idsolver = osim.InverseDynamicsSolver(model)
        residual = idsolver.solve(s, s.getUDot())
        assert residual.size() == s.getNU()
        for i in range(residual.size()):
            assert abs(residual[i]) < 1e-10
