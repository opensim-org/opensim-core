#ifndef OPENSIM_CSV_FILE_ADAPTER_H_
#define OPENSIM_CSV_FILE_ADAPTER_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  CSVFileAdapter.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 * Contributors: Michael Sherman, Chris Dembia                                *
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

/** @file
 * This file defines concrete CSVFileReader and CSVFileWriter classes, which
 * implements the  OpenSim::DataAdpater interface for reading or writing 
 * CSV data files.
*/
#include "FileAdapter.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/** CSVFileReader reads a Comma Separated Values file (.csv). 
    First uncommented line is expected to be the column labels, also separated 
    by commas. After that each row has the same number of numerical entries,
    separated by commas. No blank lines or extra spaces are allowed. Here is an example:
<pre>
time,x,y,z,theta,phi,psi
0,1.2,-3,2.3,.02,1.03,.9
.1,1.24,-2.9,2.8,.4,.99,.8
1,.9,-2.3,2.2,.7,.08,.77
</pre>
**/

template<typename DataType = SimTK::Real>
class CSVFileReader_ : public FileAdapter {

public:

    CSVFileReader_(const std::string& filename = "", const std::string& delim = ",",
        const std::string& commentTag = "#") : FileAdapter(filename), 
            _delimiter(delim), _commentTag(commentTag) {
        // Reader can only read in data. No ability to write (trunc or append).
        setAccessMode(std::ios_base::in);
    }

    CSVFileReader_* clone() const override { return new CSVFileReader_(*this); }

    virtual ~CSVFileReader_() { extendCloseDataSource(); }

protected:

    bool extendOpenDataSource() override {

        p_inputStream = new std::ifstream(getFilename(), getAccessMode());
        if (p_inputStream.empty()){
            throw Exception(
                "CSVFileReader::extendOpenDataSource(): "
                "File '" +getFilename()+"' could not be accessed for reading.");
        }
        return true;
    }

    bool extendCloseDataSource() override {
        if (p_inputStream.empty()){
            // Reader created the stream to access the file and its
            // responsibility to release and delete it.
            delete p_inputStream.release();
        }
        _inTable.clear();
        return true;
    }

    void extendPrepareForReading(AbstractDataTable& dt) const override {
        _inTable = &DataTable_<DataType>::downcast(dt);
    }

    /** override the reading capability of the DataAdapter to pull in
    the column labels from a concrete data source */
    void readColumnLabels() const {
        // a line of data read in as a string
        std::string lineString;
        std::string commentString;

        while (std::getline(getDataStream(), lineString)) {
            auto front = lineString.find_first_not_of(" \t\r\n");
            auto offset = lineString.find(_commentTag);
            // first not-whitespace character must be commentTag for
            // line to be commented out.
            if (offset != front) {
                // did not find the comment character
                break;
            }
            commentString += lineString + "\n";
        }

        // Put file comments into the comments of the meta data for now
        XMLDocument& meta = _inTable->updMetaData();
        SimTK::Xml::Comment comments(commentString);
        meta.insertTopLevelNodeBefore(meta.node_begin(), comments);

        Array<std::string> colLabels;
        FileAdapter::tokenize(lineString, *_delimiter.c_str(), colLabels);
        _inTable->updColumnLabels() = colLabels;
    }

    bool extendRead() const override {
        if (_inTable.empty()) {
            throw Exception("CSVFileReader_::read() has no DataTable to populate.\n"
                "Call prepareToRead(DataTable& table) to specify the "
                "table for the read in data.");
        }
        readColumnLabels();
        size_t nrows = 0;
        while (readNextRow()) {
            ++nrows;
        }
        return (nrows > 0);
    }

    /** extend the reading capability of the DataAdapter to read CSV rows */
    bool readNextRow() const  {
        std::string lineString;
        std::getline(getDataStream(), lineString);
        if (lineString.size() == 0){
            throw Exception("FileAdapter::readNextRow() "
                "has no more rows to read");
        }
        Array<std::string> valStrings("", 0, int(_inTable->getNumCols()));
        FileAdapter::tokenize(lineString, *_delimiter.c_str(), valStrings);

        // read in line of strings MUST match number of columns
        //assert(valStrings.size() == dt.getNumCols());
        RowVector_<DataType> row(int(valStrings.size()), DataType(SimTK::NaN));

        for (int i = 0; i < row.size(); ++i){
            std::istringstream(valStrings[i]) >> row[i];
        }
        
        _inTable->appendRow(row);
        return !getDataStream().eof();
    }

    /** Access to the data stream for to enable I/O by concrete FileAdapters.
    Check that it is connected first.  @see connectedToDataStream(). */
    std::istream& getDataStream() const { return *p_inputStream; }

private:
    SimTK::ReferencePtr<std::istream> p_inputStream;

    // writeable reference to the DataTable being filled in 
    // by the reader. No ownership taken. 
    mutable SimTK::ReferencePtr<DataTable_<DataType> > _inTable;

    // Delimiter between elements
    const std::string _delimiter;

    // Comment tag used to block line for comments
    const std::string _commentTag;
   
}; // CSVFileReader_


// Handle data of type SimTK::Real by default
typedef CSVFileReader_<SimTK::Real> CSVFileReader;

} // OpenSim namespace


#endif // OPENSIM_CSV_FILE_ADAPTER_H_