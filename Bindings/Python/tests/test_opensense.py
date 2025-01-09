"""
Test OpenSense, OpenSenseRT interfaces.
"""
import os, unittest
import opensim as osim
from opensim import Vec3
import numpy as np
from multiprocessing import Process, Queue

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestOpenSense(unittest.TestCase):
    def test_createObjects(self):
        # Make sure we can instantiate objects for interfacing to 
        # InverseKinematicsSolver
        modelfile = os.path.join(test_dir,'calibrated_model_imu.osim')
        model = osim.Model(modelfile)
        coordinates = model.getCoordinateSet();
        imuPlacer = osim.IMUPlacer();
        print("Created IMUPlacer object..")
        quatTable = osim.TimeSeriesTableQuaternion(os.path.join(test_dir, 
                             'orientation_quats.sto'))
        print("Created TimeSeriesTableQuaternion object..")
        orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
        print("Convert Quaternions to orientationsData")
        oRefs = osim.OrientationsReference(orientationsData)
        print("Created OrientationsReference object..")
        mRefs = osim.MarkersReference()
        print("Created MarkersReference object..")
        coordinateReferences = osim.SimTKArrayCoordinateReference()
        print("Created SimTKArrayCoordinateReference object..")
        constraint_var = .0001
        ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordinateReferences, constraint_var)
        print("Created InverseKinematicsSolver object..")
        oRefs = osim.BufferedOrientationsReference()
        print("Created BufferedOrientationsReference object..")
        ikSolver = osim.InverseKinematicsSolver(model, mRefs, oRefs, coordinateReferences, constraint_var)
        print("Created InverseKinematicsSolver object with BufferedOrientationsReference..")

    def test_vector_rowvector(self):
        print()
        print('Test transpose RowVector to Vector.')
        row = osim.RowVector([1, 2, 3, 4])
        col = row.transpose()
        assert (col[0] == row[0] and
                col[1] == row[1] and
                col[2] == row[2] and
                col[3] == row[3])

    def test_BufferedOrientationReferencePut(self):
        # Make sure that we can append new data to the BufferedOrientationReference object
        quatTable = osim.TimeSeriesTableQuaternion(os.path.join(test_dir,'orientation_quats.sto'))
        print("Created TimeSeriesTableQuaternion object..")
        orientationsData = osim.OpenSenseUtilities.convertQuaternionsToRotations(quatTable)
        print("Convert Quaternions to orientationsData")
        time = 0
        rowVecView = orientationsData.getNearestRow(time)
        print("Sliced orientationData")
        rowVec = osim.RowVectorRotation(rowVecView)
        print("Converted slice to row vector")
        oRefs = osim.BufferedOrientationsReference()
        print("Created BufferedOrienationReference object")
        oRefs.putValues(time, rowVec)
        print("Added row vector to BufferedOrientationReference object")
