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
        // Access element with index out of bounds. Exception expected.
        try {
            double shouldThrow = row0.get(4);
            assert false;
        } catch (java.lang.RuntimeException exc) {}
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
        // Append another row to the table with a timestamp
        // less than the previous one. Exception expected.
        try {
            table.appendRow(0.15, row);
            assert false;
        } catch(java.lang.RuntimeException exc) {}
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
        // Append another row to the table with a timestamp
        // less than the previous one. Exception expected.
        try {
            table.appendRow(0.15, row);
            assert false;
        } catch(java.lang.RuntimeException exc) {}
    }

    public static void main(String[] args) {
        test_DataTable();
        test_DataTableVec3();
        test_TimeSeriesTable();
        test_TimeSeriesTableVec3();
    }
}
