import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.io.File;
import org.opensim.modeling.*;

class TestTables {
    public static void test_DataTable() {
        DataTable table = new DataTable();
        // Set column labels.
        StdVectorString labels = new StdVectorString();
        labels.add("0"); labels.add("1"); labels.add("2"); labels.add("3");
        table.setColumnLabels(labels);
        assert table.getColumnLabels().size() == 4;
        assert table.getColumnLabel(0).equals("0");
        assert table.getColumnLabel(1).equals("1");
        assert table.getColumnLabel(2).equals("2");
        assert table.getColumnLabel(3).equals("3");
        assert table.hasColumn("2");
        assert !table.hasColumn("not-found");
        table.setColumnLabel(0, "zero");
        table.setColumnLabel(2, "two");
        assert table.getColumnLabel(0).equals("zero");
        assert table.getColumnLabel(2).equals("two");
        assert table.getColumnIndex("zero") == 0;
        assert table.getColumnIndex("two") == 2;
        // Append row to the table.
        RowVector row = new RowVector(4, 1);
        table.appendRow(0.1, row);
        assert table.getNumRows() == 1;
        assert table.getNumColumns() == 4;
        RowVectorView row0 = table.getRowAtIndex(0);
        assert row0.get(0) == 1 &&
               row0.get(1) == 1 &&
               row0.get(2) == 1 &&
               row0.get(3) == 1;
        System.out.println(table);
        // Append another row to table.
        row.set(0, 2); row.set(1, 2); row.set(2, 2); row.set(3, 2);
        table.appendRow(0.2, row);
        assert table.getNumRows() == 2;
        assert table.getNumColumns() == 4;
        RowVectorView row1 = table.getRow(0.2);
        assert row1.get(0) == 2 &&
               row1.get(1) == 2 &&
               row1.get(2) == 2 &&
               row1.get(3) == 2;
        System.out.println(table);
        // Append another row to table.
        row.set(0, 3); row.set(1, 3); row.set(2, 3); row.set(3, 3);
        table.appendRow(0.3, row);
        assert table.getNumRows() == 3;
        assert table.getNumColumns() == 4;
        RowVectorView row2 = table.getRow(0.3);
        assert row2.get(0) == 3 &&
               row2.get(1) == 3 &&
               row2.get(2) == 3 &&
               row2.get(3) == 3;
        System.out.println(table);
        // Get independent column.
        StdVectorDouble indCol = table.getIndependentColumn();
        assert indCol.get(0) == 0.1 &&
               indCol.get(1) == 0.2 &&
               indCol.get(2) == 0.3;
        // Get dependent column.
        VectorView col1 = table.getDependentColumnAtIndex(1);
        assert col1.get(0) == 1 &&
               col1.get(1) == 2 &&
               col1.get(2) == 3;
        VectorView col3 = table.getDependentColumnAtIndex(3);
        assert col3.get(0) == 1 &&
               col3.get(1) == 2 &&
               col3.get(2) == 3;
        assert table.hasColumn(0);
        assert table.hasColumn(2);
        // Edit rows of the table.
        row.set(0, 0); row.set(1, 1); row.set(2, 2); row.set(3, 3);
        table.setRowAtIndex(0, row);
        assert table.getRowAtIndex(0).get(0) == 0 &&
               table.getRowAtIndex(0).get(1) == 1 &&
               table.getRowAtIndex(0).get(2) == 2 &&
               table.getRowAtIndex(0).get(3) == 3;
        row0 = table.getRowAtIndex(0);
        row0.set(0, 10); row0.set(1, 10); row0.set(2, 10); row0.set(3, 10);
        assert table.getRowAtIndex(0).get(0) == 10 &&
               table.getRowAtIndex(0).get(1) == 10 &&
               table.getRowAtIndex(0).get(2) == 10 &&
               table.getRowAtIndex(0).get(3) == 10;
        row2 = table.getRow(0.3);
        row2.set(0, 20); row2.set(1, 20); row2.set(2, 20); row2.set(3, 20);
        assert table.getRowAtIndex(2).get(0) == 20 &&
               table.getRowAtIndex(2).get(1) == 20 &&
               table.getRowAtIndex(2).get(2) == 20 &&
               table.getRowAtIndex(2).get(3) == 20;
        System.out.println(table);
        // Edit columns of the table.
        VectorView col0 = table.getDependentColumnAtIndex(0);
        col0.set(0, 30); col0.set(1, 30); col0.set(2, 30);
        assert table.getDependentColumnAtIndex(0).get(0) == 30 &&
               table.getDependentColumnAtIndex(0).get(1) == 30 &&
               table.getDependentColumnAtIndex(0).get(2) == 30;
        col3 = table.getDependentColumn("3");
        col3.set(0, 40); col3.set(1, 40); col3.set(2, 40);
        assert table.getDependentColumn("3").get(0) == 40 &&
               table.getDependentColumn("3").get(1) == 40 &&
               table.getDependentColumn("3").get(2) == 40;
        System.out.println(table);
        // Append columns to the table.
        Vector col = new Vector(3, 1);
        table.appendColumn("4", col);
        table.appendColumn("5", col);
        assert table.getNumRows() == 3 && table.getNumColumns() == 6;
        // Add table metadata.
        table.addTableMetaDataString("subject-name", "Java");
        table.addTableMetaDataString("subject-yob" , "1995");
        assert table.getTableMetaDataString("subject-name").equals("Java");
        assert table.getTableMetaDataString("subject-yob" ).equals("1995");
        System.out.println(table);
        // Access element with index out of bounds. Exception expected.
        try {
            double shouldThrow = row0.get(4);
            assert false;
        } catch (java.lang.RuntimeException exc) {}
        try {
            double shouldThrow = col1.get(5);
            assert false;
        } catch (java.lang.RuntimeException exc) {}
        // Access row with index/time out of bounds. Exception expected.
        try {
            RowVectorView shouldThrow = table.getRowAtIndex(5);
            assert false;
        } catch (java.lang.RuntimeException exc) {}
        try {
            RowVectorView shouldThrow = table.getRow(5.5);
            assert false;
        } catch (java.lang.RuntimeException exc) {}
        // Access column with index/label out of bounds. Exception expected.
        try {
            VectorView shouldThrow = table.getDependentColumnAtIndex(6);
            assert false;
        } catch (java.lang.RuntimeException exc) {}
        try {
            VectorView shouldThrow = table.getDependentColumn("not-found");
            assert false;
        } catch (java.lang.RuntimeException exc) {}
        // Test pack-ing of columns of DataTable.
        table = new DataTable();
        labels = new StdVectorString();
        labels.add("col0_x"); labels.add("col0_y"); labels.add("col0_z");
        labels.add("col1_x"); labels.add("col1_y"); labels.add("col1_z");
        labels.add("col2_x"); labels.add("col2_y"); labels.add("col2_z");
        labels.add("col3_x"); labels.add("col3_y"); labels.add("col3_z");
        table.setColumnLabels(labels);
        row = new RowVector(12, 1);
        table.appendRow(1, row);
        row = new RowVector(12, 2);
        table.appendRow(2, row);
        row = new RowVector(12, 3);
        table.appendRow(3, row);
        assert table.getColumnLabels().size() == 12;
        assert table.getNumRows()             == 3;
        assert table.getNumColumns()          == 12;
        System.out.println(table);
        StdVectorString suffixes = new StdVectorString();
        suffixes.add("_x"); suffixes.add("_y"); suffixes.add("_z");
        DataTableVec3 tableVec3 = table.packVec3(suffixes);
        assert tableVec3.getColumnLabel(0).equals("col0");
        assert tableVec3.getColumnLabel(1).equals("col1");
        assert tableVec3.getColumnLabel(2).equals("col2");
        assert tableVec3.getColumnLabel(3).equals("col3");
        assert tableVec3.getNumRows()    == 3;
        assert tableVec3.getNumColumns() == 4;
        System.out.println(tableVec3);
        tableVec3 = table.packVec3();
        assert tableVec3.getColumnLabel(0).equals("col0");
        assert tableVec3.getColumnLabel(1).equals("col1");
        assert tableVec3.getColumnLabel(2).equals("col2");
        assert tableVec3.getColumnLabel(3).equals("col3");
        assert tableVec3.getNumRows()    == 3;
        assert tableVec3.getNumColumns() == 4;
        System.out.println(tableVec3);
        DataTable tableFlat = tableVec3.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col3_3");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
        DataTableUnitVec3 tableUnitVec3 = table.packUnitVec3();
        assert tableUnitVec3.getColumnLabel(0).equals("col0");
        assert tableUnitVec3.getColumnLabel(1).equals("col1");
        assert tableUnitVec3.getColumnLabel(2).equals("col2");
        assert tableUnitVec3.getColumnLabel(3).equals("col3");
        assert tableUnitVec3.getNumRows()    == 3;
        assert tableUnitVec3.getNumColumns() == 4;
        System.out.println(tableUnitVec3);
        tableFlat = tableUnitVec3.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col3_3");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
        labels = new StdVectorString();
        labels.add("col0.0"); labels.add("col0.1"); labels.add("col0.2");
        labels.add("col0.3"); labels.add("col1.0"); labels.add("col1.1");
        labels.add("col1.2"); labels.add("col1.3"); labels.add("col2.0");
        labels.add("col2.1"); labels.add("col2.2"); labels.add("col2.3");
        table.setColumnLabels(labels);
        DataTableQuaternion tableQuat = table.packQuaternion();
        assert tableQuat.getColumnLabel(0).equals("col0");
        assert tableQuat.getColumnLabel(1).equals("col1");
        assert tableQuat.getColumnLabel(2).equals("col2");
        assert tableQuat.getNumRows()    == 3;
        assert tableQuat.getNumColumns() == 3;
        System.out.println(tableQuat);
        tableFlat = tableQuat.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col2_4");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
        labels = new StdVectorString();
        labels.add("col0.0"); labels.add("col0.1"); labels.add("col0.2");
        labels.add("col0.3"); labels.add("col0.4"); labels.add("col0.5");
        labels.add("col1.0"); labels.add("col1.1"); labels.add("col1.2");
        labels.add("col1.3"); labels.add("col1.4"); labels.add("col1.5");
        table.setColumnLabels(labels);
        DataTableSpatialVec tableSVec = table.packSpatialVec();
        assert tableSVec.getColumnLabel(0).equals("col0");
        assert tableSVec.getColumnLabel(1).equals("col1");
        assert tableSVec.getNumRows()    == 3;
        assert tableSVec.getNumColumns() == 2;
        System.out.println(tableSVec);
        tableFlat = tableSVec.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col1_6");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
    }

    public static void test_DataTableVec3() {
        DataTableVec3 table = new DataTableVec3();
        // Set column labels.
        StdVectorString labels = new StdVectorString();
        labels.add("0"); labels.add("1"); labels.add("2"); labels.add("3");
        table.setColumnLabels(labels);
        // Append row to the table.
        Vec3 elem = new Vec3(1, 1, 1);
        StdVectorVec3 elems = new StdVectorVec3();
        elems.add(elem); elems.add(elem); elems.add(elem); elems.add(elem);
        RowVectorOfVec3 row = new RowVectorOfVec3(elems);
        table.appendRow(0.1, row);
        assert table.getNumRows() == 1;
        assert table.getNumColumns() == 4;
        RowVectorViewVec3 row0 = table.getRowAtIndex(0);
        assert row0.get(0).get(0) == 1 && row0.get(0).get(1) == 1 &&
               row0.get(0).get(2) == 1 && row0.get(1).get(0) == 1 &&
               row0.get(1).get(1) == 1 && row0.get(1).get(2) == 1 &&
               row0.get(2).get(0) == 1 && row0.get(2).get(1) == 1 &&
               row0.get(2).get(2) == 1 && row0.get(3).get(0) == 1 &&
               row0.get(3).get(1) == 1 && row0.get(3).get(2) == 1;
        System.out.println(table);
        // Append another row to the table.
        elem.set(0, 2); elem.set(1, 2); elem.set(2, 2);
        row.set(0, elem); row.set(1, elem); row.set(2, elem); row.set(3, elem);
        table.appendRow(0.2, row);
        assert table.getNumRows() == 2;
        assert table.getNumColumns() == 4;
        RowVectorViewVec3 row1 = table.getRowAtIndex(1);
        assert row1.get(0).get(0) == 2 && row1.get(0).get(1) == 2 &&
               row1.get(0).get(2) == 2 && row1.get(1).get(0) == 2 &&
               row1.get(1).get(1) == 2 && row1.get(1).get(2) == 2 &&
               row1.get(2).get(0) == 2 && row1.get(2).get(1) == 2 &&
               row1.get(2).get(2) == 2 && row1.get(3).get(0) == 2 &&
               row1.get(3).get(1) == 2 && row1.get(3).get(2) == 2;
        System.out.println(table);
        // Append another row to the table.
        elem.set(0, 3); elem.set(1, 3); elem.set(2, 3);
        row.set(0, elem); row.set(1, elem); row.set(2, elem); row.set(3, elem);
        table.appendRow(0.3, row);
        assert table.getNumRows() == 3;
        assert table.getNumColumns() == 4;
        RowVectorViewVec3 row2 = table.getRowAtIndex(2);
        assert row2.get(0).get(0) == 3 && row2.get(0).get(1) == 3 &&
               row2.get(0).get(2) == 3 && row2.get(1).get(0) == 3 &&
               row2.get(1).get(1) == 3 && row2.get(1).get(2) == 3 &&
               row2.get(2).get(0) == 3 && row2.get(2).get(1) == 3 &&
               row2.get(2).get(2) == 3 && row2.get(3).get(0) == 3 &&
               row2.get(3).get(1) == 3 && row2.get(3).get(2) == 3;
        System.out.println(table);
        // Get independent column.
        StdVectorDouble indCol = table.getIndependentColumn();
        assert indCol.get(0) == 0.1 &&
               indCol.get(1) == 0.2 &&
               indCol.get(2) == 0.3;
        // Get dependent column.
        VectorViewVec3 col1 = table.getDependentColumnAtIndex(1);
        assert col1.get(0).get(0) == 1 && col1.get(0).get(1) == 1 &&
               col1.get(0).get(2) == 1 && col1.get(1).get(0) == 2 &&
               col1.get(1).get(1) == 2 && col1.get(1).get(2) == 2 &&
               col1.get(2).get(0) == 3 && col1.get(2).get(1) == 3 &&
               col1.get(2).get(2) == 3;
        VectorViewVec3 col2 = table.getDependentColumn("2");
        assert col2.get(0).get(0) == 1 && col2.get(0).get(1) == 1 &&
               col2.get(0).get(2) == 1 && col2.get(1).get(0) == 2 &&
               col2.get(1).get(1) == 2 && col2.get(1).get(2) == 2 &&
               col2.get(2).get(0) == 3 && col2.get(2).get(1) == 3 &&
               col2.get(2).get(2) == 3;
        // Flatten table into table of doubles.
        DataTable tableDouble = table.flatten();
        assert tableDouble.getNumRows() == 3;
        assert tableDouble.getNumColumns() == 12;
        assert tableDouble.getColumnLabels().size() == 12;
        assert tableDouble.getColumnLabel( 0).equals("0_1");
        assert tableDouble.getColumnLabel( 1).equals("0_2");
        assert tableDouble.getColumnLabel( 2).equals("0_3");
        assert tableDouble.getColumnLabel( 3).equals("1_1");
        assert tableDouble.getColumnLabel( 4).equals("1_2");
        assert tableDouble.getColumnLabel( 5).equals("1_3");
        assert tableDouble.getColumnLabel( 6).equals("2_1");
        assert tableDouble.getColumnLabel( 7).equals("2_2");
        assert tableDouble.getColumnLabel( 8).equals("2_3");
        assert tableDouble.getColumnLabel( 9).equals("3_1");
        assert tableDouble.getColumnLabel(10).equals("3_2");
        assert tableDouble.getColumnLabel(11).equals("3_3");
        assert tableDouble.getRowAtIndex(0).get( 0) == 1;
        assert tableDouble.getRowAtIndex(0).get( 5) == 1;
        assert tableDouble.getRowAtIndex(0).get(11) == 1;
        assert tableDouble.getRowAtIndex(1).get( 0) == 2;
        assert tableDouble.getRowAtIndex(1).get( 5) == 2;
        assert tableDouble.getRowAtIndex(1).get(11) == 2;
        assert tableDouble.getRowAtIndex(2).get( 0) == 3;
        assert tableDouble.getRowAtIndex(2).get( 5) == 3;
        assert tableDouble.getRowAtIndex(2).get(11) == 3;
        System.out.println(tableDouble);

        StdVectorString suffixes = new StdVectorString();
        suffixes.add("_x"); suffixes.add("_y"); suffixes.add("_z");
        tableDouble = table.flatten(suffixes);
        assert tableDouble.getNumRows() == 3;
        assert tableDouble.getNumColumns() == 12;
        assert tableDouble.getColumnLabels().size() == 12;
        assert tableDouble.getColumnLabel( 0).equals("0_x");
        assert tableDouble.getColumnLabel( 1).equals("0_y");
        assert tableDouble.getColumnLabel( 2).equals("0_z");
        assert tableDouble.getColumnLabel( 3).equals("1_x");
        assert tableDouble.getColumnLabel( 4).equals("1_y");
        assert tableDouble.getColumnLabel( 5).equals("1_z");
        assert tableDouble.getColumnLabel( 6).equals("2_x");
        assert tableDouble.getColumnLabel( 7).equals("2_y");
        assert tableDouble.getColumnLabel( 8).equals("2_z");
        assert tableDouble.getColumnLabel( 9).equals("3_x");
        assert tableDouble.getColumnLabel(10).equals("3_y");
        assert tableDouble.getColumnLabel(11).equals("3_z");
        assert tableDouble.getRowAtIndex(0).get( 0) == 1;
        assert tableDouble.getRowAtIndex(0).get( 5) == 1;
        assert tableDouble.getRowAtIndex(0).get(11) == 1;
        assert tableDouble.getRowAtIndex(1).get( 0) == 2;
        assert tableDouble.getRowAtIndex(1).get( 5) == 2;
        assert tableDouble.getRowAtIndex(1).get(11) == 2;
        assert tableDouble.getRowAtIndex(2).get( 0) == 3;
        assert tableDouble.getRowAtIndex(2).get( 5) == 3;
        assert tableDouble.getRowAtIndex(2).get(11) == 3;
        System.out.println(tableDouble);
        // Edit rows of the table.
        row0 = table.getRowAtIndex(0);
        elem.set(0, 10); elem.set(1, 10); elem.set(2, 10);
        row0.set(0, elem); row0.set(1, elem);
        row0.set(2, elem); row0.set(3, elem);
        Vec3 elem0 = table.getRowAtIndex(0).get(0);
        Vec3 elem1 = table.getRowAtIndex(0).get(1);
        Vec3 elem2 = table.getRowAtIndex(0).get(2);
        Vec3 elem3 = table.getRowAtIndex(0).get(3);
        assert elem0.get(0) == 10 && elem0.get(1) == 10 && elem0.get(2) == 10 &&
               elem1.get(0) == 10 && elem1.get(1) == 10 && elem1.get(2) == 10 &&
               elem2.get(0) == 10 && elem2.get(1) == 10 && elem2.get(2) == 10 &&
               elem3.get(0) == 10 && elem3.get(1) == 10 && elem3.get(2) == 10;
        row2 = table.getRow(0.3);
        elem.set(0, 20); elem.set(1, 20); elem.set(2, 20);
        row2.set(0, elem); row2.set(1, elem);
        row2.set(2, elem); row2.set(3, elem);
        elem0 = table.getRow(0.3).get(0);
        elem1 = table.getRow(0.3).get(1);
        elem2 = table.getRow(0.3).get(2);
        elem3 = table.getRow(0.3).get(3);
        assert elem0.get(0) == 20 && elem0.get(1) == 20 && elem0.get(2) == 20 &&
               elem1.get(0) == 20 && elem1.get(1) == 20 && elem1.get(2) == 20 &&
               elem2.get(0) == 20 && elem2.get(1) == 20 && elem2.get(2) == 20 &&
               elem3.get(0) == 20 && elem3.get(1) == 20 && elem3.get(2) == 20;
        System.out.println(table);
        // Edit columns of the table.
        col1 = table.getDependentColumnAtIndex(1);
        elem.set(0, 30); elem.set(1, 30); elem.set(2, 30);
        col1.set(0, elem); col1.set(1, elem); col1.set(2, elem);
        elem0 = table.getDependentColumnAtIndex(1).get(0);
        elem1 = table.getDependentColumnAtIndex(1).get(1);
        elem2 = table.getDependentColumnAtIndex(1).get(2);
        assert elem0.get(0) == 30 && elem0.get(1) == 30 && elem0.get(2) == 30 &&
               elem1.get(0) == 30 && elem1.get(1) == 30 && elem1.get(2) == 30 &&
               elem2.get(0) == 30 && elem2.get(1) == 30 && elem2.get(2) == 30;
        col2 = table.getDependentColumn("2");
        elem.set(0, 40); elem.set(1, 40); elem.set(2, 40);
        col2.set(0, elem); col2.set(1, elem); col2.set(2, elem);
        elem0 = table.getDependentColumn("2").get(0);
        elem1 = table.getDependentColumn("2").get(1);
        elem2 = table.getDependentColumn("2").get(2);
        assert elem0.get(0) == 40 && elem0.get(1) == 40 && elem0.get(2) == 40 &&
               elem1.get(0) == 40 && elem1.get(1) == 40 && elem1.get(2) == 40 &&
               elem2.get(0) == 40 && elem2.get(1) == 40 && elem2.get(2) == 40;
        System.out.println(table);
    }

    public static void test_TimeSeriesTable() {
        TimeSeriesTable table = new TimeSeriesTable();
        StdVectorString labels = new StdVectorString();
        labels.add("0"); labels.add("1"); labels.add("2"); labels.add("3");
        table.setColumnLabels(labels);
        // Append a row to the table.
        RowVector row = new RowVector(4, 1);
        table.appendRow(0.1, row);
        assert table.getNumRows() == 1;
        assert table.getNumColumns() == 4;
        RowVectorView row0 = table.getRowAtIndex(0);
        assert row0.get(0) == 1 &&
               row0.get(1) == 1 &&
               row0.get(2) == 1 &&
               row0.get(3) == 1;
        System.out.println(table);
        // Append another row to the table.        
        row.set(0, 2); row.set(1, 2); row.set(2, 2); row.set(3, 2);
        table.appendRow(0.2, row);
        assert table.getNumRows() == 2;
        assert table.getNumColumns() == 4;
        RowVectorView row1 = table.getRow(0.2);
        assert row1.get(0) == 2 &&
               row1.get(1) == 2 &&
               row1.get(2) == 2 &&
               row1.get(3) == 2;
        System.out.println(table);
        // Append another row to the table with a timestamp
        // less than the previous one. Exception expected.
        try {
            table.appendRow(0.15, row);
            assert false;
        } catch(java.lang.RuntimeException exc) {}
        System.out.println(table);
        // Test pack-ing of columns of DataTable.
        table = new TimeSeriesTable();
        labels = new StdVectorString();
        labels.add("col0_x"); labels.add("col0_y"); labels.add("col0_z");
        labels.add("col1_x"); labels.add("col1_y"); labels.add("col1_z");
        labels.add("col2_x"); labels.add("col2_y"); labels.add("col2_z");
        labels.add("col3_x"); labels.add("col3_y"); labels.add("col3_z");
        table.setColumnLabels(labels);
        row = new RowVector(12, 1);
        table.appendRow(1, row);
        row = new RowVector(12, 2);
        table.appendRow(2, row);
        row = new RowVector(12, 3);
        table.appendRow(3, row);
        assert table.getColumnLabels().size() == 12;
        assert table.getNumRows()             == 3;
        assert table.getNumColumns()          == 12;
        System.out.println(table);
        RowVector avgRow = table.averageRow(1, 3);
        assert avgRow.ncol() == 12;
        assert Math.abs(avgRow.get( 0) - 2) < 1e-8/*epsilon*/;
        assert Math.abs(avgRow.get( 5) - 2) < 1e-8/*epsilon*/;
        assert Math.abs(avgRow.get(11) - 2) < 1e-8/*epsilon*/;
        RowVectorView nearRow = table.getNearestRow(1.1);
        assert nearRow.ncol()  == 12;
        assert nearRow.get( 0) == 1;
        assert nearRow.get( 5) == 1;
        assert nearRow.get(11) == 1;
        StdVectorString suffixes = new StdVectorString();
        suffixes.add("_x"); suffixes.add("_y"); suffixes.add("_z");
        TimeSeriesTableVec3 tableVec3 = table.packVec3(suffixes);
        assert tableVec3.getColumnLabel(0).equals("col0");
        assert tableVec3.getColumnLabel(1).equals("col1");
        assert tableVec3.getColumnLabel(2).equals("col2");
        assert tableVec3.getColumnLabel(3).equals("col3");
        assert tableVec3.getNumRows()    == 3;
        assert tableVec3.getNumColumns() == 4;
        System.out.println(tableVec3);
        TimeSeriesTable tableFlat = tableVec3.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col3_3");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
        tableVec3 = table.packVec3();
        assert tableVec3.getColumnLabel(0).equals("col0");
        assert tableVec3.getColumnLabel(1).equals("col1");
        assert tableVec3.getColumnLabel(2).equals("col2");
        assert tableVec3.getColumnLabel(3).equals("col3");
        assert tableVec3.getNumRows()    == 3;
        assert tableVec3.getNumColumns() == 4;
        System.out.println(tableVec3);
        tableFlat = tableVec3.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col3_3");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
        RowVectorOfVec3 avgRowVec3 = tableVec3.averageRow(1, 2);
        assert avgRowVec3.ncol() == 4;
        assert Math.abs(avgRowVec3.get(0).get(0) - 1.5) < 1e-8/*epsilon*/;
        assert Math.abs(avgRowVec3.get(3).get(2) - 1.5) < 1e-8/*epsilon*/;
        RowVectorViewVec3 nearRowVec3 = tableVec3.getNearestRow(1.1);
        assert nearRowVec3.ncol() == 4;
        assert nearRowVec3.get(0).get(0) == 1;
        assert nearRowVec3.get(3).get(2) == 1;
        TimeSeriesTableUnitVec3 tableUnitVec3 = table.packUnitVec3();
        assert tableUnitVec3.getColumnLabel(0).equals("col0");
        assert tableUnitVec3.getColumnLabel(1).equals("col1");
        assert tableUnitVec3.getColumnLabel(2).equals("col2");
        assert tableUnitVec3.getColumnLabel(3).equals("col3");
        assert tableUnitVec3.getNumRows()    == 3;
        assert tableUnitVec3.getNumColumns() == 4;
        System.out.println(tableUnitVec3);
        tableFlat = tableUnitVec3.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col3_3");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
        labels = new StdVectorString();
        labels.add("col0.0"); labels.add("col0.1"); labels.add("col0.2");
        labels.add("col0.3"); labels.add("col1.0"); labels.add("col1.1");
        labels.add("col1.2"); labels.add("col1.3"); labels.add("col2.0");
        labels.add("col2.1"); labels.add("col2.2"); labels.add("col2.3");
        table.setColumnLabels(labels);
        TimeSeriesTableQuaternion tableQuat = table.packQuaternion();
        assert tableQuat.getColumnLabel(0).equals("col0");
        assert tableQuat.getColumnLabel(1).equals("col1");
        assert tableQuat.getColumnLabel(2).equals("col2");
        assert tableQuat.getNumRows()    == 3;
        assert tableQuat.getNumColumns() == 3;
        System.out.println(tableQuat);
        // tableFlat = tableQuat.flatten();
        // assert tableFlat.getColumnLabels().size() == 12;
        // assert tableFlat.getColumnLabel( 0).equals("col0_1");
        // assert tableFlat.getColumnLabel(11).equals("col2_4");
        // assert tableFlat.getNumRows()             == 3;
        // assert tableFlat.getNumColumns()          == 12;
        // System.out.println(tableFlat);
        labels = new StdVectorString();
        labels.add("col0.0"); labels.add("col0.1"); labels.add("col0.2");
        labels.add("col0.3"); labels.add("col0.4"); labels.add("col0.5");
        labels.add("col1.0"); labels.add("col1.1"); labels.add("col1.2");
        labels.add("col1.3"); labels.add("col1.4"); labels.add("col1.5");
        table.setColumnLabels(labels);
        TimeSeriesTableSpatialVec tableSVec = table.packSpatialVec();
        assert tableSVec.getColumnLabel(0).equals("col0");
        assert tableSVec.getColumnLabel(1).equals("col1");
        assert tableSVec.getNumRows()    == 3;
        assert tableSVec.getNumColumns() == 2;
        System.out.println(tableSVec);
        tableFlat = tableSVec.flatten();
        assert tableFlat.getColumnLabels().size() == 12;
        assert tableFlat.getColumnLabel( 0).equals("col0_1");
        assert tableFlat.getColumnLabel(11).equals("col1_6");
        assert tableFlat.getNumRows()             == 3;
        assert tableFlat.getNumColumns()          == 12;
        System.out.println(tableFlat);
    }

    public static void test_TimeSeriesTableVec3() {
        TimeSeriesTableVec3 table = new TimeSeriesTableVec3();
        // Set column labels.
        StdVectorString labels = new StdVectorString();
        labels.add("0"); labels.add("1"); labels.add("2"); labels.add("3");
        table.setColumnLabels(labels);
        // Append row to the table.
        Vec3 elem = new Vec3(1, 1, 1);
        StdVectorVec3 elems = new StdVectorVec3();
        elems.add(elem); elems.add(elem); elems.add(elem); elems.add(elem);
        RowVectorOfVec3 row = new RowVectorOfVec3(elems);
        table.appendRow(0.1, row);
        assert table.getNumRows() == 1;
        assert table.getNumColumns() == 4;
        RowVectorViewVec3 row0 = table.getRowAtIndex(0);
        assert row0.get(0).get(0) == 1 && row0.get(0).get(1) == 1 &&
               row0.get(0).get(2) == 1 && row0.get(1).get(0) == 1 &&
               row0.get(1).get(1) == 1 && row0.get(1).get(2) == 1 &&
               row0.get(2).get(0) == 1 && row0.get(2).get(1) == 1 &&
               row0.get(2).get(2) == 1 && row0.get(3).get(0) == 1 &&
               row0.get(3).get(1) == 1 && row0.get(3).get(2) == 1;
        System.out.println(table);
        // Append another row to the table.
        elem.set(0, 2); elem.set(1, 2); elem.set(2, 2);
        row.set(0, elem); row.set(1, elem); row.set(2, elem); row.set(3, elem);
        table.appendRow(0.2, row);
        assert table.getNumRows() == 2;
        assert table.getNumColumns() == 4;
        RowVectorViewVec3 row1 = table.getRowAtIndex(1);
        assert row1.get(0).get(0) == 2 && row1.get(0).get(1) == 2 &&
               row1.get(0).get(2) == 2 && row1.get(1).get(0) == 2 &&
               row1.get(1).get(1) == 2 && row1.get(1).get(2) == 2 &&
               row1.get(2).get(0) == 2 && row1.get(2).get(1) == 2 &&
               row1.get(2).get(2) == 2 && row1.get(3).get(0) == 2 &&
               row1.get(3).get(1) == 2 && row1.get(3).get(2) == 2;
        System.out.println(table);
        // Append another row to the table with a timestamp
        // less than the previous one. Exception expected.
        try {
            table.appendRow(0.15, row);
            assert false;
        } catch(java.lang.RuntimeException exc) {}
        System.out.println(table);
        // Average row.
        RowVectorOfVec3 avgRow = table.averageRow(0.1, 0.2);
        assert avgRow.ncol() == 4;
        assert Math.abs(avgRow.get(0).get(0) - 1.5) < 1e-8/*epsilon*/;
        assert Math.abs(avgRow.get(3).get(2) - 1.5) < 1e-8/*epsilon*/;
        // Nearest row.
        RowVectorViewVec3 nearRow = table.getNearestRow(0.13);
        assert nearRow.ncol() == 4;
        assert nearRow.get(0).get(0) == 1;
        assert nearRow.get(3).get(2) == 1;
        // Flatten table into table of doubles.
        TimeSeriesTable tableDouble = table.flatten();
        assert tableDouble.getNumRows() == 2;
        assert tableDouble.getNumColumns() == 12;
        assert tableDouble.getColumnLabels().size() == 12;
        assert tableDouble.getColumnLabel( 0).equals("0_1");
        assert tableDouble.getColumnLabel( 1).equals("0_2");
        assert tableDouble.getColumnLabel( 2).equals("0_3");
        assert tableDouble.getColumnLabel( 3).equals("1_1");
        assert tableDouble.getColumnLabel( 4).equals("1_2");
        assert tableDouble.getColumnLabel( 5).equals("1_3");
        assert tableDouble.getColumnLabel( 6).equals("2_1");
        assert tableDouble.getColumnLabel( 7).equals("2_2");
        assert tableDouble.getColumnLabel( 8).equals("2_3");
        assert tableDouble.getColumnLabel( 9).equals("3_1");
        assert tableDouble.getColumnLabel(10).equals("3_2");
        assert tableDouble.getColumnLabel(11).equals("3_3");
        assert tableDouble.getRowAtIndex(0).get( 0) == 1;
        assert tableDouble.getRowAtIndex(0).get( 5) == 1;
        assert tableDouble.getRowAtIndex(0).get(11) == 1;
        assert tableDouble.getRowAtIndex(1).get( 0) == 2;
        assert tableDouble.getRowAtIndex(1).get( 5) == 2;
        assert tableDouble.getRowAtIndex(1).get(11) == 2;
        System.out.println(tableDouble);

        StdVectorString suffixes = new StdVectorString();
        suffixes.add("_x"); suffixes.add("_y"); suffixes.add("_z");
        tableDouble = table.flatten(suffixes);
        assert tableDouble.getNumRows() == 2;
        assert tableDouble.getNumColumns() == 12;
        assert tableDouble.getColumnLabels().size() == 12;
        assert tableDouble.getColumnLabel( 0).equals("0_x");
        assert tableDouble.getColumnLabel( 1).equals("0_y");
        assert tableDouble.getColumnLabel( 2).equals("0_z");
        assert tableDouble.getColumnLabel( 3).equals("1_x");
        assert tableDouble.getColumnLabel( 4).equals("1_y");
        assert tableDouble.getColumnLabel( 5).equals("1_z");
        assert tableDouble.getColumnLabel( 6).equals("2_x");
        assert tableDouble.getColumnLabel( 7).equals("2_y");
        assert tableDouble.getColumnLabel( 8).equals("2_z");
        assert tableDouble.getColumnLabel( 9).equals("3_x");
        assert tableDouble.getColumnLabel(10).equals("3_y");
        assert tableDouble.getColumnLabel(11).equals("3_z");
        assert tableDouble.getRowAtIndex(0).get( 0) == 1;
        assert tableDouble.getRowAtIndex(0).get( 5) == 1;
        assert tableDouble.getRowAtIndex(0).get(11) == 1;
        assert tableDouble.getRowAtIndex(1).get( 0) == 2;
        assert tableDouble.getRowAtIndex(1).get( 5) == 2;
        assert tableDouble.getRowAtIndex(1).get(11) == 2;
        System.out.println(tableDouble);
    }

    public static void test_FlattenWithIK() throws java.io.IOException {
        String setupFileName = new String("subject01_Setup_IK_generic.xml");
        String markerFileName = new String("walk_free_01.trc");
        String modelFileName = new String("subject01_gait2392_scaled.osim");

        TRCFileAdapter trcAdapter = new TRCFileAdapter();
        TimeSeriesTableVec3 markerTable = trcAdapter.read(markerFileName);
        System.out.println(markerTable);
        StdVectorString suffixes = new StdVectorString();
        suffixes.add(".x"); suffixes.add(".y"); suffixes.add(".z");
        TimeSeriesTable markerTableFlat = markerTable.flatten(suffixes);
        System.out.println(markerTableFlat);
        STOFileAdapter stoAdapter = new STOFileAdapter();
        markerFileName = "walk_free_01.sto";
        stoAdapter.write(markerTableFlat, markerFileName);

        InverseKinematicsTool ikTool = new InverseKinematicsTool(setupFileName);
        ikTool.setName("ik_test");
        ikTool.setModel(new Model(modelFileName));
        ikTool.setMarkerDataFileName(markerFileName);
        StdVectorDouble timeColumn = markerTable.getIndependentColumn();
        ikTool.setStartTime(timeColumn.get(0));
        ikTool.setEndTime(timeColumn.get((int)(timeColumn.size() - 1)));
        String ikResultsFileName = new String("ik_results.mot");
        ikTool.setOutputMotionFileName(ikResultsFileName);

        ikTool.run();
    }

    public static void test_vector_rowvector() {
        {
        System.out.println("Test transpose RowVector to Vector.");
        RowVector rowVec = new RowVector(4);
        for(int i = 0; i < 4; ++i)
            rowVec.set(i, i);
        Vector colVec = rowVec.transpose();
        assert colVec.size() == 4;
        for(int i = 0; i < 4; ++i)
            assert colVec.get(i) == i;
        System.out.println("Test transpose Vector to RowVector.");
        RowVector rowVec_copy = colVec.transpose();
        assert rowVec_copy.size() == 4;
        for(int i = 0; i < 4; ++i)
            assert rowVec_copy.get(i) == i;
        }
        {
        System.out.println("Test transpose RowVectorOfVec3 to VectorOfVec3.");
        StdVectorVec3 elems = new StdVectorVec3();
        for(int i = 0; i < 4; ++i)
            elems.add(new Vec3(i, i+1, i+2)); 
        RowVectorOfVec3 rowVec = new RowVectorOfVec3(elems);
        VectorOfVec3 colVec = rowVec.transpose();
        assert colVec.size() == 4;
        for(int i = 0; i < 4; ++i)
            assert colVec.get(i).get(0) == i &&
                   colVec.get(i).get(1) == i+1 &&
                   colVec.get(i).get(2) == i+2;
        System.out.println("Test transpose VectorOfVec3 to RowVectorOfVec3.");
        RowVectorOfVec3 rowVec_copy = colVec.transpose();
        assert rowVec_copy.size() == 4;
        for(int i = 0; i < 4; ++i)
            assert rowVec_copy.get(i).get(0) == i &&
                   rowVec_copy.get(i).get(1) == i+1 &&
                   rowVec_copy.get(i).get(2) == i+2;
        }
    }

    public static void main(String[] args)  throws java.io.IOException {
        test_DataTable();
        test_DataTableVec3();
        test_TimeSeriesTable();
        test_TimeSeriesTableVec3();
        test_FlattenWithIK();
        test_vector_rowvector();
    }
}
