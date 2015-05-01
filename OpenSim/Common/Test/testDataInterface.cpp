/* -------------------------------------------------------------------------- *
 *                OpenSim:  testComponentInterface.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Common/TimeSeriesData.h>
#include <OpenSim/Common/CSVFileAdapter.h>

using namespace OpenSim;
using namespace std;
using namespace SimTK;

template <> struct Object_GetClassName<SimTK::Mat22>
{
    static const std::string name() { return "Mat22"; }
};

int main() {

    try {
        DataAdapter::registerDataAdpater("streamWriter", 
            StreamWriter<SimTK::Real>());

        DataTable_<Vec3> markers(1,4);
        RowVector_<Vec3> r0(4, Vec3(1.0, 1.0, 1.0));
        RowVector_<Vec3> r1(4, Vec3(2.0, 2.0, 2.0));
        RowVector_<Vec3> r2(4, Vec3(3.0, 3.0, 3.0));
        // Update the table by row
        markers.updRow(0) = r0;
        // Append more rows.
        markers.appendRow(r1);
        markers.appendRow(r2);

        Array<string> labels;
        labels.append("mark1"); labels.append("mark2"); 
        labels.append("mark3"); labels.append("mark4");

        markers.updColumnLabels() = labels;

        size_t nr = markers.getNumRows();
        size_t nc = markers.getNumCols();

        cout << "Markers has " << nr << " rows, " << nc << " columns." << endl;
        cout << "Marker columns are " << markers.getColumnLabels() << endl;

        const RowVector_<Vec3> row = markers.getRow(2);
        row.dump("Row 2 markers");

        const Vector_<Vec3> column = markers.getColumn(3);
        column.dump("Column 3 marker");

        cout << "Element(1,3) = " << markers.getElement(1, 3) << endl;

        cout << "Markers Table:" << endl;
        cout << markers << endl;

        DataTable table;
        // Update the table by row
        table.appendRow(RowVector(5, 0.1));
        table.appendRow(RowVector(5, 0.2));
        // Grow the table by appending row
        table.appendRow(RowVector(5, 0.3));

        nr = table.getNumRows();
        nc = table.getNumCols();

        cout << "\nTable of values has " << nr << " rows, " << nc << " columns." << endl;

        labels.setSize(0);
        labels.append("time");
        for (int i = 1; i < nc; ++i){
            stringstream label;
            label << "col" << i;
            labels.append(label.str());
        }

        table.updColumnLabels() = labels;

        cout << "Table columns are " << table.getColumnLabels() << endl;

        auto row1 = table.getRow(1);
        cout << "Table Row 1 values:\n" << row1 << endl;

        cout << "Element(0,4) = " << table.getElement(0, 4) << endl;

        cout << "Double Valued Table:" << endl;
        cout << table << endl;

        const RowVector rv = table.getRow(0);
        table.appendRow(4.0*rv);

        size_t nr2 = table.getNumRows();

        ASSERT(nr + 1 == nr2);
        cout << table << endl;

        // Make a copy of the table
        DataTable table2(table);
        VectorView vcol = table2.getColumn(0);
        // Verify that VectorView from getColumn is not writable.
        //ASSERT_THROW(std::exception, vcol *= 5.0);
        // And that updColumn is writable.
        table2.updColumn(0) *= 5.0;
        table.appendDataTable(table2);

        size_t nr3 = table.getNumRows();
        ASSERT(2*nr2 == nr3);
        cout << table << endl;

        // If no adapter is registered with an identifier it should
        // throw and exception that the 
//        ASSERT_THROW(OpenSim::Exception, DataAdapter::createAdapter("stream"));

        DataAdapter* adapter = DataAdapter::createAdapter("streamWriter");
        adapter->prepareForWriting(table);
        adapter->writeOutTable(table);

        // Write out contents to cout
        cout << endl;

        std::stringstream iobuffer;
        StreamWriter<SimTK::Vec3>  markerWriter(iobuffer);
        markerWriter.prepareForWriting(markers);
        markerWriter.writeOutTable(markers);
        cout << iobuffer.str() << endl;

        StreamReader<> reader(iobuffer);
        DataTable readTable;

        reader.prepareForReading(readTable);
        reader.readInTable(readTable);
        readTable.getNumRows();
        size_t nrows = readTable.getNumCols();

        iobuffer.clear();
        DataTable_<SimTK::Mat22> matTable;
        matTable.appendRow(RowVector_<Mat22>(3, Mat22(1.0) ) );
        matTable.appendRow(RowVector_<Mat22>(3, Mat22(2.0) ) );
        matTable.appendRow(RowVector_<Mat22>(3, Mat22(3.0) ) );
        matTable.appendRow(RowVector_<Mat22>(3, Mat22(4.0) ) );
        Array<string> matlabels;
        matlabels.append("mat_1"); 
        matlabels.append("mat_2");
        matlabels.append("mat_3");
        matTable.updColumnLabels() = matlabels;

        StreamWriter<Mat22>  matWriter(iobuffer);
        matWriter.prepareForWriting(matTable);
        matWriter.writeOutTable(matTable);
        cout << iobuffer.str() << endl;

        TimeSeriesData tsd(table);
        tsd.dumpToStream(cout);

        // Time column in data table should not be part of the data in
        // time series data
        ASSERT((table.getNumCols() - 1) == tsd.getNumCols());

        cout << "\nCopy construct TimeSeriesData" << endl;
        TimeSeriesData tsd2(tsd);
        tsd2.dumpToStream(cout);

        DataTable* dt_tsd = &tsd;
        cout << "\nCopy construct TimeSeriesData from DataTable reference to TimeSeriesData" << endl;
        TimeSeriesData tsd3(*dt_tsd);
        tsd3.dumpToStream(cout);

        CSVFileReader csvReader("dataTestFile.csv");
        ASSERT(csvReader.isWriteAccess() == false);
        ASSERT(csvReader.isReadAccess() == true);
        DataTable csvTable;
        csvReader.prepareForReading(csvTable);
        csvReader.readColumnLabels(csvTable);
        csvReader.readInTable(csvTable);

        cout << endl;
        cout << "CSV Comments: " << csvTable.getMetaData() << endl;
        cout << "CSV Labels: " << csvTable.getColumnLabels() << endl;
        cout << "CSV nrows = " << csvTable.getNumRows();
        cout << "    ncols = " << csvTable.getNumCols() << endl;

        //Convert a DataTable to a TimeSeriesData table
        TimeSeriesData tcsvData(csvTable);
        tcsvData.dumpToStream(cout);
    }
    catch (const std::exception& e) {
        cout << e.what() <<endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
