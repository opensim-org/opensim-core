"""
Test DataTable interface.
"""
import os, unittest
import opensim as osim

class TestDataTable(unittest.TestCase):
    def test_clone(self):
        # Make sure the clone() method works (we have to implement this in a
        # SWIG interface file).
        dt = osim.DataTable()
        c = dt.clone()
        assert c
        dt = osim.DataTableVec3()
        c = dt.clone()
        assert c
        dt = osim.DataTableUnitVec3()
        c = dt.clone()
        assert c
        dt = osim.DataTableQuaternion()
        c = dt.clone()
        assert c
        dt = osim.DataTableVec6()
        c = dt.clone()
        assert c
        dt = osim.DataTableSpatialVec()
        c = dt.clone()
        assert c
    def test_vector_rowvector(self):
        print()
        print('Test transpose RowVector to Vector.')
        row = osim.RowVector([1, 2, 3, 4])
        col = row.transpose()
        assert (col[0] == row[0] and
                col[1] == row[1] and
                col[2] == row[2] and
                col[3] == row[3])
        print('Test transpose Vector to RowVector.')
        row_copy = col.transpose()
        assert (row_copy[0] == row[0] and
                row_copy[1] == row[1] and
                row_copy[2] == row[2] and
                row_copy[3] == row[3])
        print('Test transpose RowVectorVec3 to VectorVec3.')
        row = osim.RowVectorVec3([osim.Vec3(1, 2, 3), 
                                    osim.Vec3(4, 5, 6),
                                    osim.Vec3(7, 8, 9)])
        col = row.transpose()
        assert (str(col[0]) == str(row[0]) and
                str(col[1]) == str(row[1]) and
                str(col[2]) == str(row[2]))
        print('Test transpose VectorVec3 to RowVectorVec3.')
        row_copy = col.transpose()
        assert (str(row_copy[0]) == str(row[0]) and
                str(row_copy[1]) == str(row[1]) and
                str(row_copy[2]) == str(row[2]))
    
    def test_DataTable(self):
        print()
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
        print(table)
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
        print(table)
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
        print(table)
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
        row = osim.RowVector([100, 200, 300, 400])
        table.setRowAtIndex(0, row)
        row = table.getRowAtIndex(0)
        assert (row[0] == 100 and
                row[1] == 200 and
                row[2] == 300 and
                row[3] == 400)
        row0 = table.updRowAtIndex(0)
        row0[0] = 10
        row0[1] = 10
        row0[2] = 10
        row0[3] = 10
        row0 = table.getRowAtIndex(0)
        assert (row0[0] == 10 and
                row0[1] == 10 and
                row0[2] == 10 and
                row0[3] == 10)
        row2 = table.updRow(0.3)
        row2[0] = 20
        row2[1] = 20
        row2[2] = 20
        row2[3] = 20
        row2 = table.getRow(0.3)
        assert (row2[0] == 20 and
                row2[1] == 20 and
                row2[2] == 20 and
                row2[3] == 20)
        print(table)
        # Edit columns of the table.
        col1 = table.updDependentColumnAtIndex(1)
        col1[0] = 30
        col1[1] = 30
        col1[2] = 30
        col1 = table.getDependentColumnAtIndex(1)
        assert (col1[0] == 30 and
                col1[1] == 30 and
                col1[2] == 30)
        col3 = table.updDependentColumn('3')
        col3[0] = 40
        col3[1] = 40
        col3[2] = 40
        col3 = table.getDependentColumn('3')
        assert (col3[0] == 40 and
                col3[1] == 40 and
                col3[2] == 40)
        print(table)
        # Append columns to table.
        col = osim.Vector([1, 2, 3])
        table.appendColumn("4", col)
        table.appendColumn("5", col)
        assert (table.getNumRows()    == 3 and
                table.getNumColumns() == 6)
        # Add table metadata.
        table.addTableMetaDataString('subject-name', 'Python')
        table.addTableMetaDataString('subject-yob' , '1991')
        assert table.getTableMetaDataString('subject-name') == 'Python'
        assert table.getTableMetaDataString('subject-yob' ) == '1991'
        print(table)
        # Access eleemnt with index out of bounds. Exception expected.
        try:
            shouldThrow = row0[6]
            assert False
        except RuntimeError:
            pass
        try:
            shouldThrow = col1[5]
            assert False
        except RuntimeError:
            pass
        # Access row with index out of bounds. Exception expected.
        try:
            shouldThrow = table.getRowAtIndex(5)
            assert False
        except RuntimeError:
            pass
        # Access row with timestamp that does not exist. Exception expected.
        try:
            shouldThrow = table.getRow(5.5)
            assert False
        except RuntimeError:
            pass
        # Access column with index out of bounds. Exception expected.
        try:
            shouldThrow = table.getDependentColumnAtIndex(6)
            assert False
        except RuntimeError:
            pass
        # Access column with label that does not exist. Exception expected.
        try:
            shouldThrow = table.getDependentColumn('not-found')
            assert False
        except RuntimeError:
            pass
        # Test pack-ing of columns of DataTable.
        table = osim.DataTable()
        table.setColumnLabels(('col0_x', 'col0_y', 'col0_z',
                               'col1_x', 'col1_y', 'col1_z',
                               'col2_x', 'col2_y', 'col2_z',
                               'col3_x', 'col3_y', 'col3_z'))
        row = osim.RowVector([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        table.appendRow(1, row)
        row = osim.RowVector([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
        table.appendRow(2, row)
        row = osim.RowVector([3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])
        table.appendRow(3, row)
        assert len(table.getColumnLabels()) == 12
        assert table.getNumRows()           == 3
        assert table.getNumColumns()        == 12
        print(table)
        tableVec3 = table.packVec3(('_x', '_y', '_z'))
        tableVec3.getColumnLabels() == ('col0', 'col1', 'col2', 'col3')
        tableVec3.getNumRows()    == 3
        tableVec3.getNumColumns() == 4
        print(tableVec3)
        tableFlat = tableVec3.flatten()
        assert len(tableFlat.getColumnLabels()) == 12
        assert tableFlat.getColumnLabel( 0) == 'col0_1'
        assert tableFlat.getColumnLabel(11) == 'col3_3'
        assert tableFlat.getNumRows()           == 3
        assert tableFlat.getNumColumns()        == 12
        print(tableFlat)
        tableVec3 = table.packVec3()
        tableVec3.getColumnLabels() == ('col0', 'col1', 'col2', 'col3')
        tableVec3.getNumRows()    == 3
        tableVec3.getNumColumns() == 4
        print(tableVec3)
        tableFlat = tableVec3.flatten()
        assert len(tableFlat.getColumnLabels()) == 12
        assert tableFlat.getColumnLabel( 0) == 'col0_1'
        assert tableFlat.getColumnLabel(11) == 'col3_3'
        assert tableFlat.getNumRows()           == 3
        assert tableFlat.getNumColumns()        == 12
        print(tableFlat)
        tableUnitVec3 = table.packUnitVec3()
        tableUnitVec3.getColumnLabels() == ('col0', 'col1', 'col2', 'col3')
        tableUnitVec3.getNumRows()    == 3
        tableUnitVec3.getNumColumns() == 4
        print(tableUnitVec3)
        tableFlat = tableUnitVec3.flatten()
        assert len(tableFlat.getColumnLabels()) == 12
        assert tableFlat.getColumnLabel( 0) == 'col0_1'
        assert tableFlat.getColumnLabel(11) == 'col3_3'
        assert tableFlat.getNumRows()           == 3
        assert tableFlat.getNumColumns()        == 12
        print(tableFlat)
        table.setColumnLabels(('col0.0', 'col0.1', 'col0.2', 'col0.3',
                               'col1.0', 'col1.1', 'col1.2', 'col1.3',
                               'col2.0', 'col2.1', 'col2.2', 'col2.3'))
        tableQuat = table.packQuaternion()
        tableQuat.getColumnLabels() == ('col0', 'col1', 'col2')
        tableQuat.getNumRows()    == 3
        tableQuat.getNumColumns() == 3
        print(tableQuat)
        tableFlat = tableQuat.flatten()
        assert len(tableFlat.getColumnLabels()) == 12
        assert tableFlat.getColumnLabel( 0) == 'col0_1'
        assert tableFlat.getColumnLabel(11) == 'col2_4'
        assert tableFlat.getNumRows()           == 3
        assert tableFlat.getNumColumns()        == 12
        print(tableFlat)
        table.setColumnLabels(('col0_0', 'col0_1', 'col0_2',
                               'col0_3', 'col0_4', 'col0_5',
                               'col1_0', 'col1_1', 'col1_2',
                               'col1_3', 'col1_4', 'col1_5'))
        tableSVec = table.packSpatialVec()
        tableSVec.getColumnLabels() == ('col0', 'col1')
        tableSVec.getNumRows()    == 3
        tableSVec.getNumColumns() == 2
        print(tableSVec)
        tableFlat = tableSVec.flatten()
        assert len(tableFlat.getColumnLabels()) == 12
        assert tableFlat.getColumnLabel( 0) == 'col0_1'
        assert tableFlat.getColumnLabel(11) == 'col1_6'
        assert tableFlat.getNumRows()           == 3
        assert tableFlat.getNumColumns()        == 12
        print(tableFlat)
        table = osim.DataTable()
        table.setColumnLabels(('col0_x', 'col0_y', 'col0_z',
                               'col1_x', 'col1_y', 'col1_z',
                               'col2_x', 'col2_y', 'col2_z',
                               'col3_x', 'col3_y', 'col3_z',
                               'col4_x', 'col4_y', 'col4_z',
                               'col5_x', 'col5_y', 'col5_z'))
        row = osim.RowVector([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        table.appendRow(1, row)
        row = osim.RowVector([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
        table.appendRow(2, row)
        row = osim.RowVector([3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])
        table.appendRow(3, row)
        assert len(table.getColumnLabels()) == 18
        assert table.getNumRows()           == 3
        assert table.getNumColumns()        == 18
        table.setColumnLabels(('col0_0', 'col0_1', 'col0_2',
                               'col0_3', 'col0_4', 'col0_5',
                               'col0_6', 'col0_7', 'col0_8',
                               'col1_0', 'col1_1', 'col1_2',
                               'col1_3', 'col1_4', 'col1_5',
                               'col1_6', 'col1_7', 'col1_8'))
        tableRot = table.packRotation()
        tableRot.getColumnLabels() == ('col0', 'col1')
        tableRot.getNumRows()    == 3
        tableRot.getNumColumns() == 2
        print(tableRot)
        tableFlat = tableRot.flatten()
        assert len(tableFlat.getColumnLabels()) == 18
        assert tableFlat.getColumnLabel( 0) == 'col0_1'
        assert tableFlat.getColumnLabel(15) == 'col1_7'
        assert tableFlat.getNumRows()           == 3
        assert tableFlat.getNumColumns()        == 18
        print(tableFlat)

    def test_TimeSeriesTable(self):
        print()
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
        # Test pack-ing of columns of TimeSeriesTable.
        table = osim.TimeSeriesTable()
        table.setColumnLabels(('col0_x', 'col0_y', 'col0_z',
                               'col1_x', 'col1_y', 'col1_z',
                               'col2_x', 'col2_y', 'col2_z',
                               'col3_x', 'col3_y', 'col3_z'))
        row = osim.RowVector([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1])
        table.appendRow(1, row)
        row = osim.RowVector([2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2])
        table.appendRow(2, row)
        row = osim.RowVector([3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3])
        table.appendRow(3, row)
        assert len(table.getColumnLabels()) == 12
        assert table.getNumRows()           == 3
        assert table.getNumColumns()        == 12
        print(table)
        avgRow = table.averageRow(1, 3)
        assert avgRow.ncol() == 12
        assert abs(avgRow[ 0] - 2) < 1e-8#epsilon
        assert abs(avgRow[11] - 2) < 1e-8#epsilon
        nearRow = table.getNearestRow(1.1)
        assert nearRow.ncol() == 12
        assert nearRow[ 0] == 1
        assert nearRow[11] == 1
        tableVec3 = table.packVec3(('_x', '_y', '_z'))
        tableVec3.getColumnLabels() == ('col0', 'col1', 'col2', 'col3')
        tableVec3.getNumRows()    == 3
        tableVec3.getNumColumns() == 4
        print(tableVec3)
        tableVec3 = table.packVec3()
        tableVec3.getColumnLabels() == ('col0', 'col1', 'col2', 'col3')
        tableVec3.getNumRows()    == 3
        tableVec3.getNumColumns() == 4
        print(tableVec3)
        avgRow = tableVec3.averageRow(1, 2)
        assert avgRow.ncol() == 4
        assert abs(avgRow[0][0] - 1.5) < 1e-8#epsilon
        assert abs(avgRow[3][2] - 1.5) < 1e-8#epsilon
        nearRow = tableVec3.getNearestRow(1.1)
        assert nearRow.ncol() == 4
        assert nearRow[0][0] == 1
        assert nearRow[3][2] == 1
        tableUnitVec3 = table.packUnitVec3()
        tableUnitVec3.getColumnLabels() == ('col0', 'col1', 'col2', 'col3')
        tableUnitVec3.getNumRows()    == 3
        tableUnitVec3.getNumColumns() == 4
        print(tableUnitVec3)
        table.setColumnLabels(('col0.0', 'col0.1', 'col0.2', 'col0.3',
                               'col1.0', 'col1.1', 'col1.2', 'col1.3',
                               'col2.0', 'col2.1', 'col2.2', 'col2.3'))
        tableQuat = table.packQuaternion()
        tableQuat.getColumnLabels() == ('col0', 'col1', 'col2')
        tableQuat.getNumRows()    == 3
        tableQuat.getNumColumns() == 3
        print(tableQuat)
        table.setColumnLabels(('col0_0', 'col0_1', 'col0_2',
                               'col0_3', 'col0_4', 'col0_5',
                               'col1_0', 'col1_1', 'col1_2',
                               'col1_3', 'col1_4', 'col1_5'))
        tableSVec = table.packSpatialVec()
        tableSVec.getColumnLabels() == ('col0', 'col1')
        tableSVec.getNumRows()    == 3
        tableSVec.getNumColumns() == 2
        print(tableSVec)

    def test_DataTableVec3(self):
        table = osim.DataTableVec3()
        # Set columns labels.
        table.setColumnLabels(['0', '1', '2'])
        assert table.getColumnLabels() == ('0', '1', '2')
        # Append a row to the table.
        row = osim.RowVectorVec3([osim.Vec3(1, 2, 3), 
                                    osim.Vec3(4, 5, 6),
                                    osim.Vec3(7, 8, 9)])
        table.appendRow(0.1, row)
        assert table.getNumRows() == 1
        assert table.getNumColumns() == 3
        row0 = table.getRowAtIndex(0)
        assert (str(row0[0]) == str(row[0]) and
                str(row0[1]) == str(row[1]) and
                str(row0[2]) == str(row[2]))
        print(table)
        # Append another row to the table.
        row = osim.RowVectorVec3([osim.Vec3( 2,  4,  6), 
                                    osim.Vec3( 8, 10, 12),
                                    osim.Vec3(14, 16, 18)])
        table.appendRow(0.2, row)
        assert table.getNumRows() == 2
        assert table.getNumColumns() == 3
        row1 = table.getRow(0.2)
        assert (str(row1[0]) == str(row[0]) and
                str(row1[1]) == str(row[1]) and
                str(row1[2]) == str(row[2]))
        print(table)
        # Append another row to the table.
        row = osim.RowVectorVec3([osim.Vec3( 4,  8, 12), 
                                    osim.Vec3(16, 20, 24),
                                    osim.Vec3(28, 32, 36)])
        table.appendRow(0.3, row)
        assert table.getNumRows() == 3
        assert table.getNumColumns() == 3
        row2 = table.getRow(0.3)
        assert (str(row2[0]) == str(row[0]) and
                str(row2[1]) == str(row[1]) and
                str(row2[2]) == str(row[2]))
        print(table)
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
        print(tableDouble)
        
        tableDouble = table.flatten(['_x', '_y', '_z'])
        assert tableDouble.getColumnLabels() == ('0_x', '0_y', '0_z',
                                                 '1_x', '1_y', '1_z',
                                                 '2_x', '2_y', '2_z')
        print(tableDouble)
        
        # Edit rows of the table.
        row0 = table.updRowAtIndex(0)
        row0[0] = osim.Vec3(10, 10, 10)
        row0[1] = osim.Vec3(10, 10, 10)
        row0[2] = osim.Vec3(10, 10, 10)
        row0 = table.getRowAtIndex(0)
        assert (str(row0[0]) == str(osim.Vec3(10, 10, 10)) and
                str(row0[1]) == str(osim.Vec3(10, 10, 10)) and
                str(row0[2]) == str(osim.Vec3(10, 10, 10)))
        row2 = table.updRow(0.3)
        row2[0] = osim.Vec3(20, 20, 20)
        row2[1] = osim.Vec3(20, 20, 20)
        row2[2] = osim.Vec3(20, 20, 20)
        row2 = table.getRow(0.3)
        assert (str(row2[0]) == str(osim.Vec3(20, 20, 20)) and
                str(row2[1]) == str(osim.Vec3(20, 20, 20)) and
                str(row2[2]) == str(osim.Vec3(20, 20, 20)))
        print(table)
        # Edit columns of the table.
        col1 = table.updDependentColumnAtIndex(1)
        col1[0] = osim.Vec3(30, 30, 30)
        col1[1] = osim.Vec3(30, 30, 30)
        col1[2] = osim.Vec3(30, 30, 30)
        col1 = table.getDependentColumnAtIndex(1)
        assert (str(col1[0]) == str(osim.Vec3(30, 30, 30)) and
                str(col1[1]) == str(osim.Vec3(30, 30, 30)) and
                str(col1[2]) == str(osim.Vec3(30, 30, 30)))
        col2 = table.updDependentColumn('2')
        col2[0] = osim.Vec3(40, 40, 40)
        col2[1] = osim.Vec3(40, 40, 40)
        col2[2] = osim.Vec3(40, 40, 40)
        col2 = table.getDependentColumn('2')
        assert (str(col2[0]) == str(osim.Vec3(40, 40, 40)) and
                str(col2[1]) == str(osim.Vec3(40, 40, 40)) and
                str(col2[2]) == str(osim.Vec3(40, 40, 40)))
        print(table)


    def test_TimeSeriesTableVec3(self):
        table = osim.TimeSeriesTableVec3()
        # Set columns labels.
        table.setColumnLabels(['0', '1', '2'])
        assert table.getColumnLabels() == ('0', '1', '2')
        # Append a row to the table.
        row = osim.RowVectorVec3([osim.Vec3(1, 2, 3), 
                                    osim.Vec3(4, 5, 6),
                                    osim.Vec3(7, 8, 9)])
        table.appendRow(0.1, row)
        assert table.getNumRows() == 1
        assert table.getNumColumns() == 3
        row0 = table.getRowAtIndex(0)
        assert (str(row0[0]) == str(row[0]) and
                str(row0[1]) == str(row[1]) and
                str(row0[2]) == str(row[2]))
        print(table)
        # Append another row to the table.
        row = osim.RowVectorVec3([osim.Vec3( 2,  4,  6), 
                                    osim.Vec3( 8, 10, 12),
                                    osim.Vec3(14, 16, 18)])
        table.appendRow(0.2, row)
        assert table.getNumRows() == 2
        assert table.getNumColumns() == 3
        row1 = table.getRow(0.2)
        assert (str(row1[0]) == str(row[0]) and
                str(row1[1]) == str(row[1]) and
                str(row1[2]) == str(row[2]))
        print(table)
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
        print(tableDouble)

        tableDouble = table.flatten(['_x', '_y', '_z'])
        assert tableDouble.getColumnLabels() == ('0_x', '0_y', '0_z',
                                                 '1_x', '1_y', '1_z',
                                                 '2_x', '2_y', '2_z')
        print(tableDouble)
        
