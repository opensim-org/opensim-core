"""Test Simbody classes.

"""

import os
import unittest

import numpy as np

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Silence warning messages if mesh (.vtp) files cannot be found.
osim.Model.setDebugLevel(0)

class TestSimbody(unittest.TestCase):

    def test_vec3_typemaps(self):
        npv = np.array([5, 3, 6])
        v1 = osim.Vec3(npv)
        v2 = v1.to_numpy()
        assert (npv == v2).all()

        # Incorrect size.
        with self.assertRaises(RuntimeError):
            osim.Vec3(np.array([]))
        with self.assertRaises(RuntimeError):
            osim.Vec3(np.array([5, 1]))
        with self.assertRaises(RuntimeError):
            osim.Vec3(np.array([5, 1, 6, 3]))

        # createFromMat()
        npv = np.array([1, 6, 8])
        v1 = osim.Vec3.createFromMat(npv)
        v2 = v1.to_numpy()
        assert (npv == v2).all()

        # Incorrect size.
        with self.assertRaises(RuntimeError):
            osim.Vec3.createFromMat(np.array([]))
        with self.assertRaises(RuntimeError):
            osim.Vec3.createFromMat(np.array([5, 1]))
        with self.assertRaises(RuntimeError):
            osim.Vec3.createFromMat(np.array([5, 1, 6, 3]))

    def test_vec3_operators(self):
        v1 = osim.Vec3(1, 2, 3)
        # Tests __getitem__().
        assert v1[0] == 1
        assert v1[1] == 2

        # Out of bounds.
        with self.assertRaises(RuntimeError):
            v1[-1]
        with self.assertRaises(RuntimeError):
            v1[3]
        with self.assertRaises(RuntimeError):
            v1[5]

        # Tests __setitem__().
        v1[0] = 5
        assert v1[0] == 5

        # Out of bounds.
        with self.assertRaises(RuntimeError):
            v1[-1] = 5
        with self.assertRaises(RuntimeError):
            v1[3] = 1.3

        # Add. TODO removed for now.
        v2 = osim.Vec3(5, 6, 7)
        #v3 = v1 + v2
        #assert v3[0] == 10
        #assert v3[1] == 8
        #assert v3[2] == 10

        # Length.
        assert len(v2) == 3

    def test_vector_typemaps(self):
        npv = np.array([5, 3, 6, 2, 9])
        v1 = osim.Vector.createFromMat(npv)
        v2 = v1.to_numpy()
        assert (npv == v2).all()

        npv = np.array([])
        v1 = osim.Vector.createFromMat(npv)
        v2 = v1.to_numpy()
        assert (npv == v2).all()

    def test_rowvector_typemaps(self):
        npv = np.array([5, 3, 6, 2, 9])
        v1 = osim.RowVector.createFromMat(npv)
        v2 = v1.to_numpy()
        assert (npv == v2).all()

        npv = np.array([])
        v1 = osim.RowVector.createFromMat(npv)
        v2 = v1.to_numpy()
        assert (npv == v2).all()

    def test_vectorview_typemaps(self):
        # Use a TimeSeriesTable to obtain VectorViews.
        table = osim.TimeSeriesTable()
        table.setColumnLabels(['a', 'b'])
        table.appendRow(0.0, osim.RowVector([1.5, 2.0]))
        table.appendRow(1.0, osim.RowVector([2.5, 3.0]))
        column = table.getDependentColumn('a').to_numpy()
        assert len(column) == 2
        assert column[0] == 1.5
        assert column[1] == 2.5
        row = table.getRowAtIndex(0).to_numpy()
        assert len(row) == 2
        assert row[0] == 1.5
        assert row[1] == 2.0
        row = table.getRowAtIndex(1).to_numpy()
        assert len(row) == 2
        assert row[0] == 2.5
        assert row[1] == 3.0

    def test_matrix_typemaps(self):
        npm = np.array([[5, 3], [3, 6], [8, 1]])
        m1 = osim.Matrix.createFromMat(npm)
        m2 = m1.to_numpy()
        assert (npm == m2).all()

        npm = np.array([[]])
        m1 = osim.Matrix.createFromMat(npm)
        m2 = m1.to_numpy()
        assert (npm == m2).all()

        npm = np.array([])
        with self.assertRaises(TypeError):
            osim.Matrix.createFromMat(npm)

    def test_vector_operators(self):
        v = osim.Vector(5, 3)

        # Tests __getitem__()
        assert v[0] == 3
        assert v[4] == 3

        # Out of bounds.
        with self.assertRaises(RuntimeError):
            v[-1]
        with self.assertRaises(RuntimeError):
            v[5]

        # Tests __setitem__()
        v[0] = 15
        assert v[0] == 15

        with self.assertRaises(RuntimeError):
            v[-1] = 12
        with self.assertRaises(RuntimeError):
            v[5] = 14
        with self.assertRaises(RuntimeError):
            v[9] = 18

        # Size.
        assert len(v) == 5

    def test_exceptions(self):
        with self.assertRaises(RuntimeError):
            osim.Model("NONEXISTANT_FILE_NAME")
        with self.assertRaises(RuntimeError):
            m = osim.Model()
            # Asking for the visualizer just after constructing a model
            # throws an exception.
            m.getVisualizer()

    # def test_typemaps(self):
    #     # TODO disabled for now
    #     m = osim.Model()
    #     m.setGravity(osim.Vec3(1, 2, 3))
    #     m.setGravity([1, 2, 3])

    #     with self.assertRaises(ValueError):
    #         m.setGravity(['a', 2, 3])
    #     with self.assertRaises(ValueError):
    #         m.setGravity([1, 2])
    #     with self.assertRaises(ValueError):
    #         m.setGravity([1, 2, 6, 3])

    def test_printing(self):
        v1 = osim.Vec3(1, 3, 2)
        assert v1.__str__() == "~[1,3,2]"

        v2 = osim.Vector(7, 3)
        assert v2.__str__() == "~[3 3 3 3 3 3 3]"

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

        coordNames = model.getCoordinateNamesInMultibodyTreeOrder();
        print('firstCoord', coordNames.getElt(0));
        
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

    
