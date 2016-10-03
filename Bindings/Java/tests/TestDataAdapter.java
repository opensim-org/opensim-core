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

        TimeSeriesTable markerTableFlat = markerTable.flatten();
        assert markerTableFlat.getNumRows()    == 1103;
        assert markerTableFlat.getNumColumns() == 40 * 3;

        String fileName = new String("testDataAdapter.mot");
        STOFileAdapter stoAdapter = new STOFileAdapter();
        stoAdapter.write(markerTableFlat, fileName);

        TimeSeriesTable markerTableDouble = stoAdapter.read(fileName);
        assert markerTableDouble.getNumRows()    == 1103;
        assert markerTableDouble.getNumColumns() == 40 * 3;

        File file = new File(fileName);
        file.delete();
    }

    public static void main(String[] args) {
        test_C3DFileAdapter();
    }
};
