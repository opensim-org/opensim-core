"""
Test DataAdapter interface.
"""
import os, unittest
import opensim as osim

class TestDataAdapter(unittest.TestCase):
    def test_TRCFileAdapter(self):
        adapter = osim.TRCFileAdapter()
        table = adapter.read('../../../OpenSim/Sandbox/' + 
                             'futureOrientationInverseKinematics.trc')
        assert table.getNumRows()    == 1202
        assert table.getNumColumns() == 2

        table = adapter.read('../../../OpenSim/Examples/DataAdapter/' + 
                             'TRCFileWithNANs.trc')
        assert table.getNumRows()    == 5
        assert table.getNumColumns() == 14

    def test_MOTFileAdapter(self):
        adapter = osim.MOTFileAdapter()
        table = adapter.read('../../../OpenSim/Common/Test/' + 
                             'subject02_grf_HiFreq.mot')
        assert table.getNumRows()    == 439
        assert table.getNumColumns() == 18

        table = adapter.read('../../../OpenSim/Common/Test/' + 
                             'std_subject01_walk1_ik.mot')
        assert table.getNumRows()    == 73
        assert table.getNumColumns() == 23

    def test_C3DFileAdapter(self):
        adapter = osim.C3DFileAdapter()
        tables = adapter.read('../../../OpenSim/Common/Test/' + 
                             'singleLeglanding_2.c3d')
        markers = tables['markers']
        forces = tables['forces']
        assert markers.getNumRows()    == 1219
        assert markers.getNumColumns() == 356
        assert forces.getNumRows()     == 9752
        assert forces.getNumColumns()  == 3

        tables = adapter.read('../../../OpenSim/Common/Test/' + 
                             'jogging.c3d')
        markers = tables['markers']
        forces = tables['forces']
        assert markers.getNumRows()    == 406
        assert markers.getNumColumns() == 209
        assert forces.getNumRows()     == 2030
        assert forces.getNumColumns()  == 12
