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
        assert table.hasColumn('0')
        assert table.hasColumn('2')
        assert not table.hasColumn('not-found')
        table.setColumnLabel(0, 'zero')
        table.setColumnLabel(2, 'two')
        assert table.getColumnLabel(0) == 'zero'
        assert table.getColumnLabel(2) == 'two'
        assert table.getColumnIndex('zero') == 0
        assert table.getColumnIndex('two')  == 2
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
        row2 = table.getRow(0.3)
        assert (row2[0] == row[0] and
                row2[1] == row[1] and
                row2[2] == row[2] and
                row2[3] == row[3])
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
        assert table.hasColumn(0)
        assert table.hasColumn(2)
        # Edit rows of the table.
        row0 = table.getRowAtIndex(0)
        row0[0] = 10
        row0[1] = 10
        row0[2] = 10
        row0[3] = 10
        row0 = table.getRowAtIndex(0)
        assert (row0[0] == 10 and
                row0[1] == 10 and
                row0[2] == 10 and
                row0[3] == 10)
        row2 = table.getRow(0.3)
        row2[0] = 20
        row2[1] = 20
        row2[2] = 20
        row2[3] = 20
        row2 = table.getRow(0.3)
        assert (row2[0] == 20 and
                row2[1] == 20 and
                row2[2] == 20 and
                row2[3] == 20)
        # Edit columns of the table.
        col1 = table.getDependentColumnAtIndex(1)
        col1[0] = 30
        col1[1] = 30
        col1[2] = 30
        col1 = table.getDependentColumnAtIndex(1)
        assert (col1[0] == 30 and
                col1[1] == 30 and
                col1[2] == 30)
        col3 = table.getDependentColumn('3')
        col3[0] = 40
        col3[1] = 40
        col3[2] = 40
        col3 = table.getDependentColumn('3')
        assert (col3[0] == 40 and
                col3[1] == 40 and
                col3[2] == 40)
        # Access eleemnt with index out of bounds. Exception expected.
        try:
            shouldThrow = row0[5]
            assert false
        except RuntimeError:
            pass
        try:
            shouldThrow = col1[5]
            assert false
        except RuntimeError:
            pass
        # Access row with index out of bounds. Exception expected.
        try:
            shouldThrow = table.getRowAtIndex(5)
            assert false
        except RuntimeError:
            pass
        # Access row with timestamp that does not exist. Exception expected.
        try:
            shouldThrow = table.getRow(5.5)
            assert false
        except RuntimeError:
            pass
        # Access column with index out of bounds. Exception expected.
        try:
            shouldThrow = table.getDependentColumnAtIndex(5)
            assert false
        except RuntimeError:
            pass
        # Access column with label that does not exist. Exception expected.
        try:
            shouldThrow = table.getDependentColumn('not-found')
            assert false
        except RuntimeError:
            pass

    def test_TimeSeriesTable(self):
        table = osim.TimeSeriesTable()
        table.setColumnLabels(('col1', 'col2', 'col3', 'col4'))
        assert(table.getColumnLabels() == ('col1', 'col2', 'col3', 'col4'))
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

    def test_DataTableVec3(self):
        table = osim.DataTableVec3()
        # Set columns labels.
        table.setColumnLabels(['0', '1', '2'])
        assert table.getColumnLabels() == ('0', '1', '2')
        # Append a row to the table.
        row = osim.RowVectorOfVec3([osim.Vec3(1, 2, 3), 
                                    osim.Vec3(4, 5, 6),
                                    osim.Vec3(7, 8, 9)])
        table.appendRow(0.1, row)
        assert table.getNumRows() == 1
        assert table.getNumColumns() == 3
        row0 = table.getRowAtIndex(0)
        assert (str(row0[0]) == str(row[0]) and
                str(row0[1]) == str(row[1]) and
                str(row0[2]) == str(row[2]))
        # Append another row to the table.
        row = osim.RowVectorOfVec3([osim.Vec3( 2,  4,  6), 
                                    osim.Vec3( 8, 10, 12),
                                    osim.Vec3(14, 16, 18)])
        table.appendRow(0.2, row)
        assert table.getNumRows() == 2
        assert table.getNumColumns() == 3
        row1 = table.getRow(0.2)
        assert (str(row1[0]) == str(row[0]) and
                str(row1[1]) == str(row[1]) and
                str(row1[2]) == str(row[2]))
        # Append another row to the table.
        row = osim.RowVectorOfVec3([osim.Vec3( 4,  8, 12), 
                                    osim.Vec3(16, 20, 24),
                                    osim.Vec3(28, 32, 36)])
        table.appendRow(0.3, row)
        assert table.getNumRows() == 3
        assert table.getNumColumns() == 3
        row2 = table.getRow(0.3)
        assert (str(row2[0]) == str(row[0]) and
                str(row2[1]) == str(row[1]) and
                str(row2[2]) == str(row[2]))
        # Retrieve independent column.
        assert table.getIndependentColumn() == (0.1, 0.2, 0.3)
        # Retrieve dependent columns.
        col1 = table.getDependentColumnAtIndex(1)
        assert (str(col1[0]) == str(osim.Vec3( 4,  5,  6)) and
                str(col1[1]) == str(osim.Vec3( 8, 10, 12)) and
                str(col1[2]) == str(osim.Vec3(16, 20, 24)))
        col2 = table.getDependentColumn('2')
        assert (str(col2[0]) == str(osim.Vec3( 7,  8,  9)) and
                str(col2[1]) == str(osim.Vec3(14, 16, 18)) and
                str(col2[2]) == str(osim.Vec3(28, 32, 36)))
        # Flatten the table into table of doubles.
        tableDouble = table.flatten()
        assert tableDouble.getNumRows() == 3
        assert tableDouble.getNumColumns() == 9
        assert len(tableDouble.getColumnLabels()) == 9
        assert tableDouble.getColumnLabels() == ('0_1', '0_2', '0_3',
                                                 '1_1', '1_2', '1_3',
                                                 '2_1', '2_2', '2_3')
        assert tableDouble.getRowAtIndex(0)[0] == 1
        assert tableDouble.getRowAtIndex(1)[0] == 2
        assert tableDouble.getRowAtIndex(2)[0] == 4
        assert tableDouble.getRowAtIndex(0)[8] == 9
        assert tableDouble.getRowAtIndex(1)[8] == 18
        assert tableDouble.getRowAtIndex(2)[8] == 36

        tableDouble = table.flatten(['_x', '_y', '_z'])
        assert tableDouble.getColumnLabels() == ('0_x', '0_y', '0_z',
                                                 '1_x', '1_y', '1_z',
                                                 '2_x', '2_y', '2_z')
        # Edit rows of the table.
        row0 = table.getRowAtIndex(0)
        row0[0] = osim.Vec3(10, 10, 10)
        row0[1] = osim.Vec3(10, 10, 10)
        row0[2] = osim.Vec3(10, 10, 10)
        row0 = table.getRowAtIndex(0)
        assert (str(row0[0]) == str(osim.Vec3(10, 10, 10)) and
                str(row0[1]) == str(osim.Vec3(10, 10, 10)) and
                str(row0[2]) == str(osim.Vec3(10, 10, 10)))
        row2 = table.getRow(0.3)
        row2[0] = osim.Vec3(20, 20, 20)
        row2[1] = osim.Vec3(20, 20, 20)
        row2[2] = osim.Vec3(20, 20, 20)
        row2 = table.getRow(0.3)
        assert (str(row2[0]) == str(osim.Vec3(20, 20, 20)) and
                str(row2[1]) == str(osim.Vec3(20, 20, 20)) and
                str(row2[2]) == str(osim.Vec3(20, 20, 20)))
        # Edit columns of the table.
        col1 = table.getDependentColumnAtIndex(1)
        col1[0] = osim.Vec3(30, 30, 30)
        col1[1] = osim.Vec3(30, 30, 30)
        col1[2] = osim.Vec3(30, 30, 30)
        col1 = table.getDependentColumnAtIndex(1)
        assert (str(col1[0]) == str(osim.Vec3(30, 30, 30)) and
                str(col1[1]) == str(osim.Vec3(30, 30, 30)) and
                str(col1[2]) == str(osim.Vec3(30, 30, 30)))
        col2 = table.getDependentColumn('2')
        col2[0] = osim.Vec3(40, 40, 40)
        col2[1] = osim.Vec3(40, 40, 40)
        col2[2] = osim.Vec3(40, 40, 40)
        col2 = table.getDependentColumn('2')
        assert (str(col2[0]) == str(osim.Vec3(40, 40, 40)) and
                str(col2[1]) == str(osim.Vec3(40, 40, 40)) and
                str(col2[2]) == str(osim.Vec3(40, 40, 40)))


    def test_TimeSeriesTableVec3(self):
        table = osim.TimeSeriesTableVec3()
        # Set columns labels.
        table.setColumnLabels(['0', '1', '2'])
        assert table.getColumnLabels() == ('0', '1', '2')
        # Append a row to the table.
        row = osim.RowVectorOfVec3([osim.Vec3(1, 2, 3), 
                                    osim.Vec3(4, 5, 6),
                                    osim.Vec3(7, 8, 9)])
        table.appendRow(0.1, row)
        assert table.getNumRows() == 1
        assert table.getNumColumns() == 3
        row0 = table.getRowAtIndex(0)
        assert (str(row0[0]) == str(row[0]) and
                str(row0[1]) == str(row[1]) and
                str(row0[2]) == str(row[2]))
        # Append another row to the table.
        row = osim.RowVectorOfVec3([osim.Vec3( 2,  4,  6), 
                                    osim.Vec3( 8, 10, 12),
                                    osim.Vec3(14, 16, 18)])
        table.appendRow(0.2, row)
        assert table.getNumRows() == 2
        assert table.getNumColumns() == 3
        row1 = table.getRow(0.2)
        assert (str(row1[0]) == str(row[0]) and
                str(row1[1]) == str(row[1]) and
                str(row1[2]) == str(row[2]))
        # Append another row to the table with a timestamp
        # less than the previous one. Exception expected.
        try:
            table.appendRow(0.15, row)
            assert False
        except RuntimeError:
            pass

        tableDouble = table.flatten()
        assert tableDouble.getNumRows() == 2
        assert tableDouble.getNumColumns() == 9
        assert len(tableDouble.getColumnLabels()) == 9
        assert tableDouble.getRowAtIndex(0)[0] == 1
        assert tableDouble.getRowAtIndex(1)[0] == 2
        assert tableDouble.getRowAtIndex(0)[8] == 9
        assert tableDouble.getRowAtIndex(1)[8] == 18

        tableDouble = table.flatten(['_x', '_y', '_z'])
        assert tableDouble.getColumnLabels() == ('0_x', '0_y', '0_z',
                                                 '1_x', '1_y', '1_z',
                                                 '2_x', '2_y', '2_z')
        
