import java.io.File;
import org.opensim.modeling.*;

class TestXsensDataReader {
    public static void test_XsensDataReader() {

        // Test creation and population of XsensDataReaderSettings object 
        XsensDataReaderSettings settings = new XsensDataReaderSettings();
        ExperimentalSensor  nextSensor = new ExperimentalSensor("000_00B421AF",
            "shank");
        settings.append_ExperimentalSensors(nextSensor);
        settings.set_trial_prefix(0, "MT_012005D6_031-");

        // Test costruct XsensDataReader from a XsensDataReaderSettings object 
        XsensDataReader xsensDataReader = new XsensDataReader(settings);
        // Make sure types returned by the xsensDataReader are usable in Java
        StdMapStringAbstractDataTable tables = xsensDataReader.readSource("");

        // Check that custom accessors are available and return usable types
        // Only spot check of the table is done as actual testing of contents 
        // lives in the C++ tests
        TimeSeriesTableVec3 accelTableTyped = IMUDataReader.getLinearAccelerationsTable(tables);
        assert accelTableTyped.getNumRows()    == 3369;
        assert accelTableTyped.getNumColumns() == 1;
        assert accelTableTyped.
               getTableMetaDataString("DataRate").equals("100.000000");
    }

    public static void main(String[] args) {
        test_XsensDataReader();
    }
};
