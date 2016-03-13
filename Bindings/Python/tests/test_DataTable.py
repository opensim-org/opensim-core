"""
Test DataTable interface.
"""
import os, unittest
import opensim as osim

class TestDataTable(unittest.TestCase):
    def test_DataTable(self):
        table = osim.DataTable()
        # Set column labels.
        table.setColumnLabels(['0', '1', '2', '3'])
        assert table.getColumnLabels() == ('0', '1', '2', '3')
        # Append a row to the table.
        row = osim.RowVector([1, 2, 3, 4])
        table.appendRow(0.1, row)
        assert table.getNumRows() == 1
        assert table.getNumColumns() == 4
        row0 = table.getRowAtIndex(0)
        assert (row0[0] == row[0] and
                row0[1] == row[1] and
                row0[2] == row[2] and
                row0[3] == row[3])
        # Append another row to the table.
        row[0] *= 2
        row[1] *= 2
        row[2] *= 2
        row[3] *= 2
        table.appendRow(0.2, row)
        assert table.getNumRows() == 2
        assert table.getNumColumns() == 4
        row1 = table.getRow(0.2)
        assert (row1[0] == row[0] and
                row1[1] == row[1] and
                row1[2] == row[2] and
                row1[3] == row[3])
        # Append another row to the table.
        row[0] *= 2
        row[1] *= 2
        row[2] *= 2
        row[3] *= 2
        table.appendRow(0.3, row)
        assert table.getNumRows() == 3
        assert table.getNumColumns() == 4
        row1 = table.getRow(0.3)
        assert (row1[0] == row[0] and
                row1[1] == row[1] and
                row1[2] == row[2] and
                row1[3] == row[3])
        # Retrieve independent column.
        assert table.getIndependentColumn() == (0.1, 0.2, 0.3)
        # Retrieve dependent columns.
        col1 = table.getDependentColumnAtIndex(1)
        assert (col1[0] == 2 and
                col1[1] == 4 and
                col1[2] == 8)
        col3 = table.getDependentColumn('3')
        assert (col3[0] == 4 and
                col3[1] == 8 and
                col3[2] == 16)

    def test_TimeSeriesTable(self):
        table = osim.TimeSeriesTable()
        # Append a row to the table.
        row = osim.RowVector([1, 2, 3, 4])
        table.appendRow(0.1, row)
        assert table.getNumRows() == 1
        assert table.getNumColumns() == 4
        row0 = table.getRowAtIndex(0)
        assert (row0[0] == row[0] and
                row0[1] == row[1] and
                row0[2] == row[2] and
                row0[3] == row[3])
        # Append another row to the table.
        row[0] *= 2
        row[1] *= 2
        row[2] *= 2
        row[3] *= 2
        table.appendRow(0.2, row)
        assert table.getNumRows() == 2
        assert table.getNumColumns() == 4
        row1 = table.getRow(0.2)
        assert (row1[0] == row[0] and
                row1[1] == row[1] and
                row1[2] == row[2] and
                row1[3] == row[3])
        # Append another row to the table with a timestamp
        # less than the previous one. Exception expected.
        try:
            table.appendRow(0.15, row)
            assert False
        except RuntimeError:
            pass
            
