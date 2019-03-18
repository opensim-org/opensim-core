import java.io.File;
import org.opensim.modeling.*;

class TestXsensDataReader {
    public static void test_XsensDataReader() {
        XsensDataReaderSettings settings = new XsensDataReaderSettings();
        ExperimentalSensor  nextSensor = new ExperimentalSensor("000_00B421AF", "shank");
        settings.append_ExperimentalSensors(nextSensor);
        settings.set_trial_prefix(0, "MT_012005D6_031-");
        XsensDataReader xsensDataReader = new XsensDataReader(settings);
        StdMapStringAbstractDataTable tables = xsensDataReader.extendRead("");
        TimeSeriesTableVec3 accelTableTyped = XsensDataReader.getLinearAccelerationsTable(tables);
        System.out.println("Number of Rows, columns, rate:", 
               accelTableTyped.getNumRows(), accelTableTyped.getNumColumns(), 
               accelTableTyped.getTableMetaDataString("DataRate"));
        assert accelTableTyped.getNumRows()    == 3369;
        assert accelTableTyped.getNumColumns() == 1;
        assert accelTableTyped.
               getTableMetaDataString("DataRate").equals("100.000000");
    }

    public static void main(String[] args) {
        test_XsensDataReader();
    }
};
