import org.opensim.modeling.*;

class TestReporter {
    public static void test_TableReporter_1() throws java.io.IOException {
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
        tableReporter.addToReport(tableSource.getOutput("column").
                                          getChannel("col2"));

        State state = model.initSystem();
        state.setTime(0.1);
        model.realizeReport(state);
        state.setTime(0.2);
        model.realizeReport(state);
        state.setTime(0.3);
        model.realizeReport(state);

        table = tableReporter.getTable();

        assert table.getColumnLabels().size()          == 2;
        assert table.getColumnLabel(0).
                                      equals("/table_source|column:col1");
        assert table.getColumnLabel(1).
                                      equals("/table_source|column:col2");
        assert table.getNumRows()                      == 3;
        assert table.getNumColumns()                   == 2;
        assert table.getRowAtIndex(0).get(0)           == 1;
        assert table.getRowAtIndex(0).get(1)           == 1;
        assert table.getRowAtIndex(1).get(0)           == 2;
        assert table.getRowAtIndex(1).get(1)           == 2;
        assert table.getRowAtIndex(2).get(0)           == 3;
        assert table.getRowAtIndex(2).get(1)           == 3;
        assert table.getIndependentColumn().get(0)     == 0.1;
        assert table.getIndependentColumn().get(1)     == 0.2;
        assert table.getIndependentColumn().get(2)     == 0.3;

        for(int iteration = 0; iteration < 4; ++iteration) {
            tableReporter.clearTable();
            assert table.getNumRows()                  == 0;
            assert table.getNumColumns()               == 0;
            assert table.getColumnLabels().size()      == 2;
            assert table.getColumnLabel(0).
                                      equals("/table_source|column:col1");
            assert table.getColumnLabel(1).
                                      equals("/table_source|column:col2");
            // Make sure the table reference is still valid.
            assert table.getNumRows()                  == 0;
            assert table.getNumColumns()               == 0;
            assert table.getColumnLabels().size()      == 2;
            assert table.getColumnLabel(0).
                                      equals("/table_source|column:col1");
            assert table.getColumnLabel(1).
                                      equals("/table_source|column:col2");
        
            state = model.initSystem();
            state.setTime(0.1);
            model.realizeReport(state);
            state.setTime(0.2);
            model.realizeReport(state);
            state.setTime(0.3);
            model.realizeReport(state);

            assert table.getColumnLabels().size()      == 2;
            assert table.getColumnLabel(0).
                                      equals("/table_source|column:col1");
            assert table.getColumnLabel(1).
                                      equals("/table_source|column:col2");
            assert table.getNumRows()                  == 3;
            assert table.getNumColumns()               == 2;
            assert table.getRowAtIndex(0).get(0)       == 1;
            assert table.getRowAtIndex(0).get(1)       == 1;
            assert table.getRowAtIndex(1).get(0)       == 2;
            assert table.getRowAtIndex(1).get(1)       == 2;
            assert table.getRowAtIndex(2).get(0)       == 3;
            assert table.getRowAtIndex(2).get(1)       == 3;
            assert table.getIndependentColumn().get(0) == 0.1;
            assert table.getIndependentColumn().get(1) == 0.2;
            assert table.getIndependentColumn().get(2) == 0.3;
        }
    }

    public static void test_TableReporter_2() throws java.io.IOException {
        final double timeInterval = 0.1;
        
        String modelFileName = "double_pendulum_markers.osim";
        Model model = new Model(modelFileName);

        ConsoleReporter consoleReporter = new ConsoleReporter();
        consoleReporter.set_report_time_interval(timeInterval);
        consoleReporter.
            addToReport(model.getCoordinateSet().get(0).getOutput("value"),
                    "pin1_angle");
        consoleReporter.
            updInput("inputs").
            connect(model.getCoordinateSet().get(1).getOutput("value"),
                    "q2");
        model.addComponent(consoleReporter);

        TableReporter tableReporter = new TableReporter();
        tableReporter.set_report_time_interval(timeInterval);
        tableReporter.
            updInput("inputs").
            connect(model.getCoordinateSet().get(0).getOutput("value"),
                    "q1");
        tableReporter.
            addToReport(model.getCoordinateSet().get(1).getOutput("value"),
                    "q2");
        model.addComponent(tableReporter);

        STOFileAdapter stoAdapter = new STOFileAdapter();

        assert !tableReporter.getTable().hasColumnLabels();

        for(int n = 1; n <= 10; ++n) {
            assert tableReporter.getTable().getNumRows()    == 0;
            assert tableReporter.getTable().getNumColumns() == 0;
            
            State state = model.initSystem();
            Manager manager = new Manager(model);
            state.setTime(0);
            manager.initialize(state);
            state = manager.integrate(n);

            TimeSeriesTable table = tableReporter.getTable();
            stoAdapter.write(table, "pendulum_coordinates.sto");

            assert table.getColumnLabels().size() == 2;
            assert table.getColumnLabel(0).equals("q1");
            assert table.getColumnLabel(1).equals("q2");
            assert table.getNumRows()    == 1 + (n / timeInterval);
            assert table.getNumColumns() == 2;

            tableReporter.clearTable();

            assert table.getColumnLabels().size() == 2;
            assert table.getColumnLabel(0).equals("q1");
            assert table.getColumnLabel(1).equals("q2");
            assert table.getNumRows()    == 0;
            assert table.getNumColumns() == 0;
        }
    }

    public static void main(String[] args) throws java.io.IOException {
        test_TableReporter_1();
        test_TableReporter_2();
    }
};
