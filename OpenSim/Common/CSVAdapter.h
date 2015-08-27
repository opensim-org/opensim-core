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

#include <fstream>

namespace OpenSim {

//=============================================================================
//=============================================================================
/** CSVAdapter reads a Comma Separated Values file (.csv). 
    First uncommented line is expected to be the column labels, also separated 
    by commas. After that each row has the same number of numerical entries,
    separated by commas. No blank lines or extra spaces are allowed. Here is an
    example:
<pre>
time,x,y,z,theta,phi,psi
0,1.2,-3,2.3,.02,1.03,.9
.1,1.24,-2.9,2.8,.4,.99,.8
1,.9,-2.3,2.2,.7,.08,.77
</pre>
**/


template<typename DataType = SimTK::Real>
class CSVAdapter_ : public FileAdapter {
public:
    using Table = TimeSeriesTable_<DataType>;

    CSVAdapter_* clone() const override { 
        return new CSVAdapter_(*this); 
    }

    void prepareForReading(AbstractDataTable& table) override {
        table_ = &dynamic_cast<Table&>(table);
    }

    void prepareForWriting(const AbstractDataTable& table) override {
        table_ = &dynamic_cast<Table&>(const_cast<AbstractDataTable&>(table));
    }

    void read() override;
    
    void write() override;

    static std::string getIdentifier() {
        return "csv";
    }

private:
    Table* table_;
    
    static const std::string delimiter_read_;
    static const std::string delimiter_write_;
    static const std::string time_column_label_;
};
template<typename DataType>
const std::string CSVAdapter_<DataType>::delimiter_read_{","};
template<typename DataType>
const std::string CSVAdapter_<DataType>::delimiter_write_{","};
template<typename DataType>
const std::string CSVAdapter_<DataType>::time_column_label_{"time"};

template<>
void CSVAdapter_<SimTK::Real>::read() {
    if(filename_.empty())
        throw Exception{"Input filename is not set."};

    std::ifstream in_stream{filename_};

    // First line has the column labels.
    std::string line{};
    std::getline(in_stream, line);
    auto column_labels = tokenize(line, delimiter_read_);
    column_labels.erase(column_labels.begin());

    // Read the rows one at a time.
    std::size_t row_num{0};
    while(std::getline(in_stream, line)) {
        auto row = tokenize(line, delimiter_read_);
        ++row_num;
        
        if(row.size() != column_labels.size() + 1)
            throw Exception{"There are " + std::to_string(column_labels.size())
                    + " column labels but row " + std::to_string(row_num) + 
                    " contains " + std::to_string(row.size()) + " columns."};

        // Columns 2 till the end are data.
        std::vector<SimTK::Real> row_vector{};
        std::transform(row.cbegin() + 1, 
                       row.cend(), 
                       std::back_inserter(row_vector),
                       [] (const std::string& elt) {
                           return std::stod(elt);
                       });

        // Column 0 is time.
        table_->addTimeAndRow(std::stod(row[0]), std::move(row_vector));
    }

    // Set the column labels of the table.
    table_->setColumnLabels(column_labels);
}

template<>
void CSVAdapter_<SimTK::Real>::write() {
    if(filename_.empty())
        throw Exception{"Input filename is not set."};

    std::ofstream out_stream{filename_};

    // First line is column labels.
    out_stream << time_column_label_;
    for(unsigned col = 0; col < table_->getNumColumns(); ++col)
        out_stream << delimiter_write_ << table_->getColumnLabel(col);
    out_stream << std::endl;

    // Data rows.
    for(unsigned row = 0; row < table_->getNumRows(); ++row) {
        out_stream << table_->getTime(row);
        for(unsigned col = 0; col < table_->getNumColumns(); ++col)
            out_stream << delimiter_write_ << table_->getElt(row, col);
        out_stream << std::endl;
    }
}

// Handle data of type SimTK::Real by default
typedef CSVAdapter_<SimTK::Real> CSVAdapter;

} // OpenSim namespace


#endif // OPENSIM_CSV_FILE_ADAPTER_H_
