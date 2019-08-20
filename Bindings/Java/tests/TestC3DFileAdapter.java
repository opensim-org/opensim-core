import java.io.File;
import org.opensim.modeling.*;

class TestC3DFileAdapter {
    public static void test_C3DFileAdapter() {
        C3DFileAdapter c3dAdapter = new C3DFileAdapter();
        c3dAdapter.setLocationForForceExpression(C3DFileAdapter.ForceLocation.CenterOfPressure);
        StdMapStringAbstractDataTable tables =
            c3dAdapter.read("walking5.c3d");

        // Marker data read from C3D.
        TimeSeriesTableVec3 markerTable = c3dAdapter.getMarkersTable(tables);
        assert markerTable.getNumRows()    == 1103;
        assert markerTable.getNumColumns() == 40;
        assert markerTable.
               getTableMetaDataString("DataRate").equals("250.000000");
        assert markerTable.getTableMetaDataString("Units").equals("mm");

        // Flatten marker data.
        TimeSeriesTable markerTableFlat = markerTable.flatten();
        assert markerTableFlat.getNumRows()    == 1103;
        assert markerTableFlat.getNumColumns() == 40 * 3;

        // Make sure flattenned marker data is writable/readable to/from file.
        String markerFileName = new String("markers.mot");
        STOFileAdapter stoAdapter = new STOFileAdapter();
        stoAdapter.write(markerTableFlat, markerFileName);
        TimeSeriesTable markerTableDouble = new TimeSeriesTable(markerFileName);
        assert markerTableDouble.getNumRows()    == 1103;
        assert markerTableDouble.getNumColumns() == 40 * 3;

        // Forces data read from C3D.
        TimeSeriesTableVec3 forceTable = c3dAdapter.getForcesTable(tables);
        assert forceTable.getNumRows()    == 8824;
        assert forceTable.getNumColumns() == 6;
        assert forceTable.
               getTableMetaDataString("DataRate").equals("2000.000000");
        StdVectorUnsigned fpTypes =
            forceTable.getTableMetaDataVectorUnsigned("Types");
        assert fpTypes.size() == 2;
        assert fpTypes.get(0) == 2;
        assert fpTypes.get(1) == 2;
        StdVectorMatrix fpCalMatrices =
            forceTable.getTableMetaDataVectorMatrix("CalibrationMatrices");
        assert fpCalMatrices.size() == 2;
        Matrix calMatrix0 = fpCalMatrices.get(0);
        assert calMatrix0.nrow() == 6;
        assert calMatrix0.ncol() == 6;
        Matrix calMatrix1 = fpCalMatrices.get(1);
        assert calMatrix1.nrow() == 6;
        assert calMatrix1.ncol() == 6;
        StdVectorMatrix fpCorners =
            forceTable.getTableMetaDataVectorMatrix("Corners");
        assert fpCorners.size() == 2;
        Matrix corner0 = fpCorners.get(0);
        assert corner0.nrow() == 3;
        assert corner0.ncol() == 4;
        Matrix corner1 = fpCorners.get(1);
        assert corner1.nrow() == 3;
        assert corner1.ncol() == 4;
        StdVectorMatrix fpOrigins =
            forceTable.getTableMetaDataVectorMatrix("Origins");
        assert fpOrigins.size() == 2;
        Matrix origin0 = fpOrigins.get(0);
        assert origin0.nrow() == 3;
        assert origin0.ncol() == 1;
        Matrix origin1 = fpOrigins.get(1);
        assert origin1.nrow() == 3;
        assert origin1.ncol() == 1;
        StdVectorString units = forceTable.getDependentsMetaDataString("units");
        assert units.size() == 6;
        assert units.get(0).equals("N");
        assert units.get(1).equals("mm");
        assert units.get(2).equals("Nmm");
        assert units.get(3).equals("N");
        assert units.get(4).equals("mm");
        assert units.get(5).equals("Nmm");
            

        // Flatten forces data.
        TimeSeriesTable forceTableFlat = forceTable.flatten();
        assert forceTableFlat.getNumRows()    == 8824;
        assert forceTableFlat.getNumColumns() == 6 * 3;

        // Make sure flattenned forces data is writable/readable to/from file.
        String forceFileName = "forces.mot";
        stoAdapter = new STOFileAdapter();
        stoAdapter.write(forceTableFlat, forceFileName);
        TimeSeriesTable forceTableDouble = new TimeSeriesTable(forceFileName);
        assert forceTableDouble.getNumRows()    == 8824;
        assert forceTableDouble.getNumColumns() == 6 * 3;

        // Clean up.
        File file = new File(markerFileName);
        file.delete();
        file = new File(forceFileName);
        file.delete();
    }

    public static void main(String[] args) {
        test_C3DFileAdapter();
    }
};
