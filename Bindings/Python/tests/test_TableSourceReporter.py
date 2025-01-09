import os
import unittest

import opensim as osim

test_dir = os.path.join(os.path.dirname(os.path.abspath(osim.__file__)),
                        'tests')

# Tests TableSource and Reporter (including TableReporter) classes.
class TestTableSourceReporter(unittest.TestCase):
    def test_TableSourceReporter(self):
        m = osim.Model()

        # Source.
        source = osim.TableSource()
        source.setName("source")

        table = osim.TimeSeriesTable()
        table.setColumnLabels(('col1', 'col2', 'col3', 'col4'))
        row = osim.RowVector([1, 2, 3, 4])
        table.appendRow(0.0, row)
        row = osim.RowVector([2, 3, 4, 5])
        table.appendRow(1.0, row)

        source.setTable(table)

        # Reporter.
        c_rep = osim.ConsoleReporter()
        c_rep.setName("c_rep")

        t_rep = osim.TableReporter()
        t_rep.setName("t_rep")

        # Add.
        m.addComponent(source)
        m.addComponent(c_rep)
        m.addComponent(t_rep)

        # Connect.
        c_rep.addToReport(source.getOutput("column").getChannel("col1"))
        t_rep.addToReport(source.getOutput("column").getChannel("col2"))

        # Realize.
        s = m.initSystem()
        s.setTime(0.5)
        m.realizeReport(s)

        # Test.
        # This value is the average of col2 (2.0 and 3.0).
        assert t_rep.getTable().getRowAtIndex(0)[0] == 2.5


