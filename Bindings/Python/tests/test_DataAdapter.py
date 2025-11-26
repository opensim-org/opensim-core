"""
Test DataAdapter interface.
"""
import os, unittest
import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

class TestDataAdapter(unittest.TestCase):
    def test_TRCFileAdapter(self):
        table = osim.TimeSeriesTableVec3(os.path.join(test_dir, 
                             'futureOrientationInverseKinematics.trc'))
        assert table.getNumRows()    == 1202
        assert table.getNumColumns() == 2

        table = osim.TimeSeriesTableVec3(os.path.join(test_dir, 'dataWithNaNsOfDifferentCases.trc'))
        assert table.getNumRows()    == 5
        assert table.getNumColumns() == 14

    def test_STOFileAdapter(self):
        table = osim.TimeSeriesTable(os.path.join(test_dir, 'subject02_grf_HiFreq.mot'))
        assert table.getNumRows()    == 439
        assert table.getNumColumns() == 18

        table = osim.TimeSeriesTable(os.path.join(test_dir, 
                                          'std_subject01_walk1_ik.mot'))
        assert table.getNumRows()    == 73
        assert table.getNumColumns() == 23

    def test_C3DFileAdapter(self):
        try:
            adapter = osim.C3DFileAdapter()
        except AttributeError:
            # C3D support not available. OpenSim was not compiled with ezc3d.
            return
        tables = adapter.read(os.path.join(test_dir, 'walking2.c3d'))
        forces = adapter.getForcesTable(tables)
        markers = adapter.getMarkersTable(tables)
        
        assert markers.getNumRows()    == 1249
        assert markers.getNumColumns() == 44
        assert forces.getNumRows()     == 9992
        assert forces.getNumColumns()  == 6
        adapter.setLocationForForceExpression(1)
        tables2 = adapter.read(os.path.join(test_dir, 'walking5.c3d'))

        # Marker data read from C3D.
        markers = adapter.getMarkersTable(tables2)
        assert markers.getNumRows()    == 1103
        assert markers.getNumColumns() == 40
        assert markers.getTableMetaDataString('DataRate') == '250.000000'
        assert markers.getTableMetaDataString('Units') == 'mm'

        # Flatten marker data.
        markersFlat = markers.flatten()
        assert markersFlat.getNumRows()    == 1103
        assert markersFlat.getNumColumns() == 40 * 3

        # Make sure flattenned marker data is writable/readable to/from file.
        markersFilename = 'markers.sto'
        stoAdapter = osim.STOFileAdapter()
        stoAdapter.write(markersFlat, markersFilename)
        markersDouble = osim.TimeSeriesTable(markersFilename)
        assert markersDouble.getNumRows()    == 1103
        assert markersDouble.getNumColumns() == 40 * 3

        # Forces data read from C3d.
        forces = adapter.getForcesTable(tables2)
        assert forces.getNumRows()     == 8824
        assert forces.getNumColumns()  == 6
        assert forces.getTableMetaDataString('DataRate') == '2000.000000'
        assert forces.getTableMetaDataVectorUnsigned('Types') == (2, 2)
        fpCalMats = forces.getTableMetaDataVectorMatrix("CalibrationMatrices")
        assert len(fpCalMats) == 2
        assert fpCalMats[0].nrow() == 6
        assert fpCalMats[0].ncol() == 6
        assert fpCalMats[1].nrow() == 6
        assert fpCalMats[1].ncol() == 6
        fpCorners = forces.getTableMetaDataVectorMatrix("Corners")
        assert len(fpCorners) == 2
        assert fpCorners[0].nrow() == 3
        assert fpCorners[0].ncol() == 4
        assert fpCorners[1].nrow() == 3
        assert fpCorners[1].ncol() == 4
        fpOrigins = forces.getTableMetaDataVectorMatrix("Origins")
        assert len(fpOrigins) == 2
        assert fpOrigins[0].nrow() == 3
        assert fpOrigins[0].ncol() == 1
        assert fpOrigins[1].nrow() == 3
        assert fpOrigins[1].ncol() == 1
        assert forces.getDependentsMetaDataString('units') == ('N', 'mm', 'Nmm',
                                                               'N', 'mm', 'Nmm')

        # Flatten forces data.
        forcesFlat = forces.flatten()
        assert forcesFlat.getNumRows()    == 8824
        assert forcesFlat.getNumColumns() == 6 * 3

        # Make sure flattenned forces data is writable/readable to/from file.
        forcesFilename = 'forces.sto'
        stoAdapter.write(forcesFlat, forcesFilename)
        forcesDouble = osim.TimeSeriesTable(forcesFilename)
        assert forcesDouble.getNumRows()    == 8824
        assert forcesDouble.getNumColumns() == 6 * 3

        # Clean up.
        os.remove(markersFilename)
        os.remove(forcesFilename)
