import org.opensim.modeling.*;

class TestReporter {
    public static void test_TableReporter() throws java.io.IOException {
        // Prepare the table for table-source.
        TimeSeriesTable table = new TimeSeriesTable();
        StdVectorString labels = new StdVectorString();
        labels.add("col0"); labels.add("col1");
        labels.add("col2"); labels.add("col3");
        table.setColumnLabels(labels);
        RowVector row = new RowVector(4, 1);
        table.appendRow(0.1, row);
        row.set(0, 2); row.set(1, 2); row.set(2, 2); row.set(3, 2);
        table.appendRow(0.2, row);
        row.set(0, 3); row.set(1, 3); row.set(2, 3); row.set(3, 3);
        table.appendRow(0.3, row);

        TableSource tableSource = new TableSource();
        tableSource.setName("table_source");
        tableSource.setTable(table);

        TableReporter tableReporter = new TableReporter();
        tableReporter.setName("table_reporter");

        Model model = new Model();
        model.setName("model");
        model.addComponent(tableSource);
        model.addComponent(tableReporter);

        tableReporter.updInput("inputs").
                      connect(tableSource.getOutput("column").
                                          getChannel("col1"));
        tableReporter.updInput("inputs").
                      connect(tableSource.getOutput("column").
                                          getChannel("col2"));

        State state = model.initSystem();
        state.setTime(0.1);
        model.realizeReport(state);
        state.setTime(0.2);
        model.realizeReport(state);
        state.setTime(0.3);
        model.realizeReport(state);

        assert tableReporter.getTable().getColumnLabels().size()      == 2;
        assert tableReporter.getTable().getColumnLabel(0).
                                      equals("/model/table_source/column:col1");
        assert tableReporter.getTable().getColumnLabel(1).
                                      equals("/model/table_source/column:col2");
        assert tableReporter.getTable().getNumRows()                  == 3;
        assert tableReporter.getTable().getNumColumns()               == 2;
        assert tableReporter.getTable().getRowAtIndex(0).get(0)       == 1;
        assert tableReporter.getTable().getRowAtIndex(0).get(1)       == 1;
        assert tableReporter.getTable().getRowAtIndex(1).get(0)       == 2;
        assert tableReporter.getTable().getRowAtIndex(1).get(1)       == 2;
        assert tableReporter.getTable().getRowAtIndex(2).get(0)       == 3;
        assert tableReporter.getTable().getRowAtIndex(2).get(1)       == 3;
        assert tableReporter.getTable().getIndependentColumn().get(0) == 0.1;
        assert tableReporter.getTable().getIndependentColumn().get(1) == 0.2;
        assert tableReporter.getTable().getIndependentColumn().get(2) == 0.3;

        for(int iteration = 0; iteration < 4; ++iteration) {
            tableReporter.clearTable();
            assert tableReporter.getTable().getNumRows()                  == 0;
            assert tableReporter.getTable().getNumColumns()               == 0;
            assert tableReporter.getTable().getColumnLabels().size()      == 2;
            assert tableReporter.getTable().getColumnLabel(0).
                                      equals("/model/table_source/column:col1");
            assert tableReporter.getTable().getColumnLabel(1).
                                      equals("/model/table_source/column:col2");
        
            state = model.initSystem();
            state.setTime(0.1);
            model.realizeReport(state);
            state.setTime(0.2);
            model.realizeReport(state);
            state.setTime(0.3);
            model.realizeReport(state);

            assert tableReporter.getTable().getColumnLabels().size()      == 2;
            assert tableReporter.getTable().getColumnLabel(0).
                                      equals("/model/table_source/column:col1");
            assert tableReporter.getTable().getColumnLabel(1).
                                      equals("/model/table_source/column:col2");
            assert tableReporter.getTable().getNumRows()                  == 3;
            assert tableReporter.getTable().getNumColumns()               == 2;
            assert tableReporter.getTable().getRowAtIndex(0).get(0)       == 1;
            assert tableReporter.getTable().getRowAtIndex(0).get(1)       == 1;
            assert tableReporter.getTable().getRowAtIndex(1).get(0)       == 2;
            assert tableReporter.getTable().getRowAtIndex(1).get(1)       == 2;
            assert tableReporter.getTable().getRowAtIndex(2).get(0)       == 3;
            assert tableReporter.getTable().getRowAtIndex(2).get(1)       == 3;
            assert tableReporter.getTable().
                                 getIndependentColumn().get(0) == 0.1;
            assert tableReporter.getTable().
                                 getIndependentColumn().get(1) == 0.2;
            assert tableReporter.getTable().
                                 getIndependentColumn().get(2) == 0.3;
        }
    }

    public static void main(String[] args) throws java.io.IOException {
        test_TableReporter();
    }
};
