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

template <> struct Object_GetClassName<SimTK::Mat22>
{
    static const std::string name() { return "Mat22"; }
};


//=============================================================================
/**
* StreamWriter is a concrete class defining an interface for writing
* out the contents of a DataTable to a stream. It controls access to a stream.
*/
template<typename DataType = SimTK::Real>
class StreamWriter : public DataAdapter {

public:
    StreamWriter() : DataAdapter() {
        setAccessMode(std::ios_base::out);
    }

    virtual ~StreamWriter() {}

    StreamWriter(std::ostream& out) : _outputstream(&out) {
        setAccessMode(std::ios_base::out);
    }

    StreamWriter* clone() const override { return new StreamWriter(*this); }

protected:

    bool extendOpenDataSource() override {
        if (_outputstream.empty()){
            std::cout << "StreamWriter::extendOpenDataSrouce() "
                << "No output stream open.\n"
                << " Assuming std::cout." << std::endl;
            _outputstream = std::cout;
        }
        return true;
    }

    bool extendCloseDataSource() override {
        if (_outputstream.empty()){
            _outputstream.clear();
        }
        return true;
    }

    void extendPrepareForWriting(const AbstractDataTable& dt) override {
        _outTable = &(DataTable_<DataType>::downcast(dt));
        *_outputstream << dt.getDataTypeInfo().name() << "\n";
        *_outputstream << dt.getColumnLabels() << "\n";
    }

    /** extend the writing capability of the DataAdapter to write out
    the data to a std::ostream */
    bool extendWrite() override {
        for (size_t rix = 0; rix < _outTable->getNumRows(); ++rix) {
            *_outputstream << _outTable->getRow(rix) << std::endl;
        }
        return true;
    }

private:
    SimTK::ReferencePtr<std::ostream> _outputstream;
    SimTK::ReferencePtr<const DataTable_<DataType> > _outTable;

    //=============================================================================
};  // END of class StreamWriter


//=============================================================================
/**
* StreamReader is a concrete class defining an interface for reading in
* the contents of a DataTable from a stream. It controls access to a stream.
*/
template<typename DataType = SimTK::Real>
class StreamReader : public DataAdapter {
    //    OpenSim_DECLARE_CONCRETE_OBJECT_T(StreamReader, DataType, DataAdapter);

public:
    StreamReader() : DataAdapter() { setAccessMode(std::ios_base::out); }
    virtual ~StreamReader() {}

    StreamReader(std::istream& in) : _inputstream(&in) {
        setAccessMode(std::ios_base::in);
    }

    StreamReader* clone() const override { return new StreamReader(*this); }

    bool extendOpenDataSource() override {
        if (_inputstream.empty()){
            throw std::runtime_error(
                "StreamReader::extendPrepareForReading(): "
                "No input stream specified.\n"
                "Assuming std::cin.");
        }
        return true;
    }

    bool extendCloseDataSource() override {
        if (_inputstream.empty()){
            _inputstream.clear();
        }
        return true;
    }

    void extendPrepareForReading(AbstractDataTable& dt) const override {
        _inTable = &DataTable_<DataType>::downcast(dt);
    }

    /** extend the reading capability of the DataAdapter to write out table
        to a std::istream. Currently implementation is for demonstration only
        and does not parse the row into valid values. */
    bool extendRead() const override {
        std::string rowString;
        std::getline(*_inputstream, rowString);
        std::cout << "StreamReader::readNextRow() input stream contains:";
        std::cout << rowString << std::endl;
        SimTK::RowVector_<DataType> row(int(_inTable->getNumCols()), DataType(0));
        _inTable->appendRow(row);
        return !_inputstream->eof();
    }

private:
    SimTK::ReferencePtr<std::istream> _inputstream;
    mutable SimTK::ReferencePtr<DataTable_<DataType> > _inTable;
};

using namespace std;
using namespace SimTK;

int main() {

    try {
        DataAdapter::registerDataAdapter("streamWriter",
            StreamWriter<SimTK::Real>());
        DataAdapter::registerDataAdapter("csv", CSVFileReader());

        // create a data table of Vec3's, which would be like marker data
        DataTable_<Vec3> markers(1,4);
        RowVector_<Vec3> r0(4, Vec3(1.0, 1.0, 1.0));
        RowVector_<Vec3> r1(4, Vec3(2.0, 2.0, 2.0));
        RowVector_<Vec3> r2(4, Vec3(3.0, 3.0, 3.0));
        
        // Update the table by row
        markers.updRow(0) = r0;
        // TODO updating the next row should throw an exception since markers
        // should only have 1 row. Currently aborts in release. SimTK::Matrix
        // checks in debug.
        //ASSERT_THROW(std::exception, markers.updRow(1) = r1);

        // Append more rows to grow the table.
        markers.appendRow(r1);
        markers.appendRow(r2);
        cout << "HERE 2" << endl;
        ASSERT_THROW(std::exception,
             markers.appendRow(RowVector_<Vec3>(3, Vec3(4.0, 4.0, 4.0))) );

        // create and update column labels for the table of marker data
        Array<string> labels;
        labels.append("mark1"); labels.append("mark2"); 
        labels.append("mark3"); labels.append("mark4");
        markers.updColumnLabels() = labels;

        // demonstrate row and column access of the data table
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
        // exercise DataTable's output operator
        cout << markers << endl;

        // DataTable is our typical table of Real numbers
        DataTable table;
        // Build up the table by row
        table.appendRow(RowVector(5, 0.1));
        table.appendRow(RowVector(5, 0.2));
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

        // If no adapter is registered with a given identifier, the base should
        // throw an exception to that effect.
        ASSERT_THROW(OpenSim::Exception, DataAdapter::createAdapter("stream"));

        auto* adapter = DataAdapter::createAdapter("streamWriter");
        adapter->prepareForWriting(table);
        adapter->write();

        // Write out contents to cout
        cout << endl;

        std::stringstream iobuffer;
        StreamWriter<SimTK::Vec3>  markerWriter(iobuffer);
        markerWriter.prepareForWriting(markers);
        markerWriter.write();
        cout << iobuffer.str() << endl;

        StreamReader<> reader(iobuffer);
        DataTable readTable;

        reader.prepareForReading(readTable);
        reader.read();
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
        matWriter.write();
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
        csvReader.read();

        cout << endl;
        cout << "CSV Comments: " << csvTable.getMetaData() << endl;
        cout << "CSV Labels: " << csvTable.getColumnLabels() << endl;
        cout << "CSV nrows = " << csvTable.getNumRows();
        cout << "    ncols = " << csvTable.getNumCols() << endl;

        DataTable csvTable2("dataTestFile.csv");
        cout << "DataTable csvTable2('dataTestFile.csv')" << endl;
        csvTable2.dumpToStream(cout);

        // Try to read the table in again. Should throw an exception
        // about no more lines to read from file.
        ASSERT_THROW(std::exception, csvReader.read());
        // If we set the file to be loaded it must close and open the new.
        csvReader.setFilename("dataTestFile.csv");
        // Should now fail because the reader needs a table to fill */
        ASSERT_THROW(std::exception, csvReader.read());
        
        //prepare the reader for the given table
        csvReader.prepareForReading(csvTable2);
        csvReader.read(); 

        cout << endl;
        cout << "2nd read of CSV file: nrows = " << csvTable2.getNumRows();
        cout << "    ncols = " << csvTable2.getNumCols() << endl;
        cout << csvTable2 << endl;

        // Convert a DataTable to a TimeSeriesData table
        // Should complain that time column is not sequential since
        // we just read in the data into the csvTable twice!
        ASSERT_THROW(std::exception, TimeSeriesData tcsvData(csvTable2));

        // create a table of time series data from the csvTable
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