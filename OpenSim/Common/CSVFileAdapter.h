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

class CSVFileAdapter : public FileAdapter {
public:
    using Table = TimeSeriesTable_<double>;

    CSVFileAdapter()                                 = default;
    CSVFileAdapter(const CSVFileAdapter&)            = default;
    CSVFileAdapter(CSVFileAdapter&&)                 = default;
    CSVFileAdapter& operator=(const CSVFileAdapter&) = default;
    CSVFileAdapter& operator=(CSVFileAdapter&&)      = default;
    ~CSVFileAdapter()                                = default;

    CSVFileAdapter* clone() const override;

    std::unique_ptr<Table> read(const std::string& fileName) const;

    void write(const Table& table, const std::string& fileName) const;

    static const std::string _table;

protected:
    OutputTables extendRead(const std::string& fileName) const override;

    void extendWrite(const InputTables& tables,
                     const std::string& fileName) const override;

private:
    static const std::string delimiter_read_;
    static const std::string delimiter_write_;
    static const std::string time_column_label_;
};

} // namespace OpenSim


#endif // OPENSIM_CSV_FILE_ADAPTER_H_
