/* -------------------------------------------------------------------------- *
 *                          OpenSim:  DelimFileAdapter.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#ifndef OPENSIM_DELIM_FILE_ADAPTER_H_
#define OPENSIM_DELIM_FILE_ADAPTER_H_

#include "FileAdapter.h"


namespace OpenSim {

/** DelimFileAdapter is a FileAdapter that reads and writes text files with
given delimiters. CSVFileAdapter and MOTFileAdapter derive from this class and
set the delimiters appropriately for the files they parse. The read/write
functions return/accept a specific type of DataTable referred to as Table in 
this class.                                                                   
Header in the file is assumed to end with string "endheader" occupying a full
line.                                                                         */
class OSIMCOMMON_API DelimFileAdapter : public FileAdapter {
public:
    DelimFileAdapter()                                   = delete;
    DelimFileAdapter(const DelimFileAdapter&)            = default;
    DelimFileAdapter(DelimFileAdapter&&)                 = default;
    DelimFileAdapter& operator=(const DelimFileAdapter&) = default;
    DelimFileAdapter& operator=(DelimFileAdapter&&)      = default;
    ~DelimFileAdapter()                                  = default;

    /** Create the adapter by setting the delimiters.                         */
    DelimFileAdapter(const std::string& delimitersRead,
                     const std::string& delimterWrite);

    DelimFileAdapter* clone() const override;

    /** Read a given file using the delimiters specified at construction.     */
    std::unique_ptr<TimeSeriesTable> read(const std::string& filename) const;

    /** Write the table to a file using the delimiter specified at 
    construction.                                                             */
    void write(const TimeSeriesTable& table,
               const std::string& filename) const;

    /** Key used for table associative array returned/accepted by write/read. */
    static const std::string _table;

protected:
    /** Implementation of the read functionality.                             */
    OutputTables extendRead(const std::string& filename) const override;

    /** Implementation of the write functionality.                            */
    void extendWrite(const InputTables& tables,
                     const std::string& filename) const override;

private:
    /** Delimiters used for reading.                                          */
    const std::string _delimitersRead;
    /** Delimiter used for writing.                                           */
    const std::string _delimiterWrite;
    /** String representing the end of header in the file.                    */
    static const std::string _endHeaderString;
    /** Column label of the time column.                                      */
    static const std::string _timeColumnLabel;
};

}

#endif // OPENSIM_DELIM_FILE_ADAPTER_H_
