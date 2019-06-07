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
        StdMapStringAbstractDataTable tables = xsensDataReader.read("");

        // Check that custom accessors are available and return usable types
        // Only spot check of the table is done as actual testing of contents 
        // lives in the C++ tests
        TimeSeriesTableVec3 accelTableTyped = IMUDataReader.getLinearAccelerationsTable(tables);
        assert accelTableTyped.getNumRows()    == 3369;
        assert accelTableTyped.getNumColumns() == 1;
        assert accelTableTyped.
               getTableMetaDataString("DataRate").equals("100.000000");

        TimeSeriesTableQuaternion quatsTable = IMUDataReader.getOrientationsTable(tables);
        TimeSeriesTableRotation rotsTable = OpenSenseUtilities.convertQuaternionsToRotations(quatsTable);

        // Test access to TimeSeriesTableRotation methods and individual Rotation matrices
        rotsTable.getNumColumns();
        rotsTable.getNumRows();
        Rotation rot = rotsTable.getRowAtIndex(0).getElt(0,0);
        System.out.println("Rotation Matrix at rotsTable(0,0)\n=================================");
        System.out.println(rot);
        Vec3 angles = rot.convertRotationToBodyFixedXYZ();
        System.out.println("Body-Fixed Euler Angles (X,Y,Z)\n===============================");
        System.out.println(angles);

        Rotation rot2 = rotsTable.getRowAtIndex(0).getElt(0, 1);
        Vec3 angles2 = rot2.convertRotationToBodyFixedXYZ();
        System.out.println("Body-Fixed Euler Angles (X,Y,Z)\n===============================");
        System.out.println(angles2);

        Rotation diff = rot.multiply(rot2.transpose());
        Vec4 errorAngleAxis = diff.convertRotationToAngleAxis();
        System.out.println("Error Angle Axis\n===============================");
        System.out.println(errorAngleAxis);

        //Verify that we can construct an OrientationsReference from a TimeSeriesTableRotation
        // Uncomment below to test.
        OrientationsReference oRefs = new OrientationsReference(rotsTable);
    }

    public static void main(String[] args) {
        test_XsensDataReader();
    }
};
