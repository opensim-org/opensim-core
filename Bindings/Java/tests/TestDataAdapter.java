import java.io.File;
import org.opensim.modeling.*;

class TestDataAdapter {
    public static void test_C3DFileAdapter() {
        C3DFileAdapter c3dAdapter = new C3DFileAdapter();
        StdMapStringTimeSeriesTableVec3 tables =
            c3dAdapter.read("../../../../OpenSim/Common/Test/walking5.c3d");


        TimeSeriesTableVec3 markerTable = tables.get("markers");
        assert markerTable.getNumRows()    == 1103;
        assert markerTable.getNumColumns() == 40;
        assert markerTable.
               getTableMetaDataString("DataRate").equals("250.000000");
        assert markerTable.getTableMetaDataString("Units").equals("mm");

        TimeSeriesTable markerTableFlat = markerTable.flatten();
        assert markerTableFlat.getNumRows()    == 1103;
        assert markerTableFlat.getNumColumns() == 40 * 3;

        String markerFileName = new String("markers.mot");
        STOFileAdapter stoAdapter = new STOFileAdapter();
        stoAdapter.write(markerTableFlat, markerFileName);

        TimeSeriesTable markerTableDouble = stoAdapter.read(markerFileName);
        assert markerTableDouble.getNumRows()    == 1103;
        assert markerTableDouble.getNumColumns() == 40 * 3;


        TimeSeriesTableVec3 forceTable = tables.get("forces");
        assert forceTable.getNumRows()    == 8824;
        assert forceTable.getNumColumns() == 6;
        assert forceTable.
               getTableMetaDataString("DataRate").equals("2000.000000");
        StdVectorString units = forceTable.getDependentsMetaDataString("units");
        assert units.size() == 6;
        assert units.get(0).equals("N");
        assert units.get(1).equals("Nmm");
        assert units.get(2).equals("mm");
        assert units.get(3).equals("N");
        assert units.get(4).equals("Nmm");
        assert units.get(5).equals("mm");

        TimeSeriesTable forceTableFlat = forceTable.flatten();
        assert forceTableFlat.getNumRows()    == 8824;
        assert forceTableFlat.getNumColumns() == 6 * 3;

        String forceFileName = new String("forces.mot");
        stoAdapter = new STOFileAdapter();
        stoAdapter.write(forceTableFlat, forceFileName);

        TimeSeriesTable forceTableDouble = stoAdapter.read(forceFileName);
        assert forceTableDouble.getNumRows()    == 8824;
        assert forceTableDouble.getNumColumns() == 6 * 3;


        File file = new File(markerFileName);
        file.delete();
        file = new File(forceFileName);
        file.delete();
    }

    public static void main(String[] args) {
        test_C3DFileAdapter();
    }
};
