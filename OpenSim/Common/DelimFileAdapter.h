/* -------------------------------------------------------------------------- *
 *                          OpenSim:  DelimFileAdapter.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "SimTKcommon.h"

#include "About.h"
#include "FileAdapter.h"
#include "TimeSeriesTable.h"
#include "OpenSim/Common/IO.h"

#include <string>
#include <fstream>
#include <regex>

namespace OpenSim {

class IncorrectNumTokens : public Exception {
public:
    IncorrectNumTokens(const std::string& file,
                       size_t line,
                       const std::string& func,
                       const std::string& msg) :
        Exception(file, line, func) {

        addMessage(msg);
    }
};

class DataTypeMismatch : public Exception {
public:
    DataTypeMismatch(const std::string& file,
                     size_t line,
                     const std::string& func,
                     const std::string& expected,
                     const std::string& received) :
        Exception(file, line, func) {
        std::string msg = "expected = " + expected;
        msg += " received = " + received;

        addMessage(msg);
    }
};

namespace {
    template<typename T>
    struct is_SimTK_Vec : std::false_type {};

    template<int M, typename ELT, int Stride>
    struct is_SimTK_Vec<SimTK::Vec<M, ELT, Stride>> {
        static constexpr bool value = (M >= 2 && M <= 12);
    };
} // namespace

/** DelimFileAdapter is a FileAdapter that reads and writes text files with
given delimiters. CSVFileAdapter and MOTFileAdapter derive from this class and
set the delimiters appropriately for the files they parse. The read/write
functions return/accept a specific type of DataTable referred to as Table in 
this class.                                                                   
Header in the file is assumed to end with string "endheader" occupying a full
line.                                                                         */
template<typename T>
class DelimFileAdapter : public FileAdapter {
    static_assert(std::is_same<T, double           >::value ||
                  is_SimTK_Vec<T                   >::value ||
                  std::is_same<T, SimTK::UnitVec3  >::value ||
                  std::is_same<T, SimTK::Quaternion>::value ||  
                  std::is_same<T, SimTK::SpatialVec>::value,
                  "Template argument T must be one of the following types : "
                  "double, SimTK::Vec2 to SimTK::Vec9, SimTK::Vec<10> to "
                  "SimTK::Vec<12>, SimTK::UnitVec, SimTK::Quaternion, "
                  "SimTK::SpatialVec");
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

    /** Create the adapter by setting the delimiters.                         */
    DelimFileAdapter(const std::string& delimitersRead,
                     const std::string& delimterWrite,
                     const std::string& compDelimRead,
                     const std::string& compDelimWrite);

    DelimFileAdapter* clone() const override;

    /** Key used for table associative array returned/accepted by write/read. */
    static const std::string tableString();

    /** Name of the data type T (template parameter).                         */
    static inline std::string dataTypeName();

protected:
    /** Implementation of the read functionality.                             */
    OutputTables extendRead(const std::string& filename) const override;

    /** Implementation of the write functionality.                            */
    void extendWrite(const InputTables& tables,
                     const std::string& filename) const override;

    /** Read elements of type T (template parameter) from a sequence of 
    tokens.                                                                   */
    inline SimTK::RowVector_<T> 
    readElems(const std::vector<std::string>& tokens) const;

    /** Write an element of type T (template parameter) to stream with the
    specified precision.                                                      */
    inline void writeElem(std::ostream& stream, 
                          const T& elem,
                          const unsigned& prec) const;

private:
    /** Following overloads implement dataTypeName().                         */
    static inline std::string dataTypeName_impl(double);
    static inline std::string dataTypeName_impl(SimTK::UnitVec3);
    static inline std::string dataTypeName_impl(SimTK::Quaternion);
    static inline std::string dataTypeName_impl(SimTK::SpatialVec);
    template<int M>
    static inline std::string dataTypeName_impl(SimTK::Vec<M>);

    /** Following overloads implement readElems().                            */
    inline SimTK::RowVector_<double>
    readElems_impl(const std::vector<std::string>& tokens,
                   double) const;
    inline SimTK::RowVector_<SimTK::UnitVec3>
    readElems_impl(const std::vector<std::string>& tokens,
                   SimTK::UnitVec3) const;
    inline SimTK::RowVector_<SimTK::Quaternion>
    readElems_impl(const std::vector<std::string>& tokens,
                   SimTK::Quaternion) const;
    inline SimTK::RowVector_<SimTK::SpatialVec>
    readElems_impl(const std::vector<std::string>& tokens,
                   SimTK::SpatialVec) const;
    template<int M>
    inline SimTK::RowVector_<SimTK::Vec<M>>
    readElems_impl(const std::vector<std::string>& tokens,
                   SimTK::Vec<M>) const;

    /** Following overloads implement writeElem().                            */
    inline void writeElem_impl(std::ostream& stream,
                               const double& elem,
                               const unsigned& prec) const;
    inline void writeElem_impl(std::ostream& stream,
                               const SimTK::SpatialVec& elem,
                               const unsigned& prec) const;
    template<int M>
    inline void writeElem_impl(std::ostream& stream,
                               const SimTK::Vec<M>& elem,
                               const unsigned& prec) const;
      
    /** Trim string -- remove specified leading and trailing characters from 
    string. Trims out whitespace by default.                                  */
    static std::string trim(const std::string& str, const char& ch = ' ');
    
    /** Delimiters used for reading.                                          */
    const std::string _delimitersRead;
    /** Delimiter used for writing. Separates elements from each other.       */
    const std::string _delimiterWrite;
    /** Delimiter used for reading. Separates components of an element.       */
    const std::string _compDelimRead;
    /** Delimiter used for writing. Separates components of an element.       */
    const std::string _compDelimWrite;
    /** String representing the end of header in the file.                    */
    static const std::string _endHeaderString;
    /** Column label of the time column.                                      */
    static const std::string _timeColumnLabel;
    /** Key used to read/write data-type name.                                */
    static const std::string _dataTypeString;
    /** Key used to read/write file version number.                           */
    static const std::string _versionString;
    /** Key used to read/write OpenSim version number.                        */
    static const std::string _opensimVersionString;
    /** File version number.                                                  */
    static const std::string _versionNumber;
};


template<typename T>
const std::string 
DelimFileAdapter<T>::tableString() {
    return "table";
}

template<typename T>
const std::string 
DelimFileAdapter<T>::_endHeaderString = "endheader";

template<typename T>
std::string
DelimFileAdapter<T>::dataTypeName() {
    return dataTypeName_impl(T{});
}

template<typename T>
std::string
DelimFileAdapter<T>::dataTypeName_impl(double) {
    return "double";
}

template<typename T>
std::string
DelimFileAdapter<T>::dataTypeName_impl(SimTK::UnitVec3) {
    return "UnitVec3";
}

template<typename T>
std::string
DelimFileAdapter<T>::dataTypeName_impl(SimTK::Quaternion) {
    return "Quaternion";
}

template<typename T>
std::string
DelimFileAdapter<T>::dataTypeName_impl(SimTK::SpatialVec) {
    return "SpatialVec";
}

template<typename T>
template<int M>
std::string
DelimFileAdapter<T>::dataTypeName_impl(SimTK::Vec<M>) {
  return std::string{"Vec"} + std::to_string(M);
}

template<typename T>
const std::string 
DelimFileAdapter<T>::_timeColumnLabel = "time";

template<typename T>
const std::string
DelimFileAdapter<T>::_dataTypeString = "DataType";

template<typename T>
const std::string
DelimFileAdapter<T>::_versionString = "version";

template<typename T>
const std::string
DelimFileAdapter<T>::_versionNumber = "3";

template<typename T>
const std::string
DelimFileAdapter<T>::_opensimVersionString = "OpenSimVersion";

template<typename T>
std::string
DelimFileAdapter<T>::trim(const std::string& str,
                          const char& ch) {
  auto begin = str.find_first_not_of(ch);
  if(begin == std::string::npos)
    return std::string{};

  auto count = str.find_last_not_of(ch) - begin + 1;
  return str.substr(begin, count);
}

template<typename T>
DelimFileAdapter<T>::DelimFileAdapter(const std::string& delimitersRead,
                                      const std::string& delimiterWrite) :
    _delimitersRead{delimitersRead}, 
    _delimiterWrite{delimiterWrite}
{}

template<typename T>
DelimFileAdapter<T>::DelimFileAdapter(const std::string& delimitersRead,
                                      const std::string& delimiterWrite,
                                      const std::string& compDelimRead,
                                      const std::string& compDelimWrite) :
    _delimitersRead{delimitersRead}, 
    _delimiterWrite{delimiterWrite},
    _compDelimRead{compDelimRead},
    _compDelimWrite{compDelimWrite}
{}

template<typename T>
DelimFileAdapter<T>*
DelimFileAdapter<T>::clone() const {
    return new DelimFileAdapter{*this};
}

template<typename T>
typename DelimFileAdapter<T>::OutputTables
DelimFileAdapter<T>::extendRead(const std::string& fileName) const {
    OPENSIM_THROW_IF(fileName.empty(),
                     EmptyFileName);

    std::ifstream in_stream{fileName};
    OPENSIM_THROW_IF(!in_stream.good(),
                     FileDoesNotExist,
                     fileName);
    
    OPENSIM_THROW_IF(in_stream.peek() == std::ifstream::traits_type::eof(),
                     FileIsEmpty,
                     fileName);

    size_t line_num{};
    // All the lines until "endheader" is header.
    std::regex endheader{R"([ \t]*)" + _endHeaderString + R"([ \t]*)"};
    std::regex keyvalue{R"((.*)=(.*))"};
    std::string header{};
    std::string line{};
    std::string numberOrDelim = "[0-9][0-9."+_delimitersRead+" -]+";
    std::regex dataLine{ numberOrDelim };
    ValueArrayDictionary keyValuePairs;
    while(std::getline(in_stream, line)) {
        ++line_num;

        // We might be parsing a file with CRLF (\r\n) line endings on a
        // platform that uses only LF (\n) line endings, in which case the \r
        // is part of `line` and we must remove it manually.
        if (!line.empty() && line.back() == '\r') 
            line.pop_back();

        if(std::regex_match(line, endheader))
            break;
        // Exit this loop for parsing header if we hit data line
        // this will blow up immediately rather than hang
        if (std::regex_match(line, dataLine))
            OPENSIM_THROW(
                Exception,
                "Missing end of header block");

        // Detect Key value pairs of the form "key = value" and add them to
        // metadata.
        std::smatch matchRes{};
        if(std::regex_match(line, matchRes, keyvalue)) {
            auto key = matchRes[1].str();
            auto value = matchRes[2].str();
            IO::TrimWhitespace(value);
            if(!key.empty() && !value.empty()) {
                const auto trimmed_key = trim(key);
                if(trimmed_key == _dataTypeString) {
                    // Discard key-value pair specifying datatype. Datatype is
                    // known at this point.
                    OPENSIM_THROW_IF(value != dataTypeName(),
                                     DataTypeMismatch,
                                     dataTypeName(),
                                     value);
                } else if(trimmed_key == _versionString) {
                    // Discard STO version number. Version number is added
                    // during writing.
                } else if(trimmed_key == _opensimVersionString) {
                    // Discard OpenSim version number. Version number is added
                    // during writing.
                } else {
                    keyValuePairs.setValueForKey(key, value);
                }
                continue;
            }
        }

        if(header.empty())
            header = line;
        else
            header += "\n" + line;
    }
    keyValuePairs.setValueForKey("header", header);

    // Callable to get the next line in form of vector of tokens.
    auto nextLine = [&] {
        return getNextLine(in_stream, _delimitersRead);
    };

    // Read the line containing column labels and fill up the column labels
    // container.
    std::vector<std::string> column_labels{};
    while (column_labels.size() == 0) { // keep going down rows to find labels
        column_labels = nextLine();
        // for labels we never expect empty elements, so remove them
        IO::eraseEmptyElements(column_labels);
        ++line_num;
    }

    OPENSIM_THROW_IF(column_labels.size() == 0, Exception,
                     "No column labels detected in file '" + fileName + "'.");
    
    // Column 0 is the time column. Check and get rid of it. The data in this
    // column is maintained separately from rest of the data.
    OPENSIM_THROW_IF(column_labels[0] != _timeColumnLabel,
                     UnexpectedColumnLabel,
                     fileName,
                     _timeColumnLabel,
                     column_labels[0]);
    column_labels.erase(column_labels.begin());

    // Read the rows one at a time and fill up the time column container and
    // the data container. Start with a reasonable initial capacity for
    // tradeoff between a small file and larger files. 100 worked well for
    // a 50 MB file with ~80000 lines.
    std::vector<double> timeVec;
    int initCapacity = 100;
    int ncol = static_cast<int>(column_labels.size());
    timeVec.reserve(initCapacity);
    SimTK::Matrix_<T> matrix(initCapacity, ncol);
    
    // Initialize current row and capacity
    int curCapacity = initCapacity;
    int curRow = 0;

    // Start looping through each line
    auto row = nextLine();
    while (!row.empty()) {
        ++line_num;
        
        // Double capacity if we reach the end of the containers.
        // This is necessary until Simbody issue #401 is addressed.
        if (curRow+1 > curCapacity) {
            curCapacity *= 2;
            timeVec.reserve(curCapacity);
            matrix.resizeKeep(curCapacity, ncol);
        }

        // Time is column 0.
        timeVec.push_back(std::stod(row.front()));
        row.erase(row.begin());

        auto row_vector = readElems(row);

        OPENSIM_THROW_IF(row_vector.size() != (int)column_labels.size(),
            RowLengthMismatch,
            fileName,
            line_num,
            column_labels.size(),
            static_cast<size_t>(row_vector.size()));
        
        matrix.updRow(curRow) = std::move(row_vector);

        row = nextLine();
        ++curRow;
    }

    // Resize the matrix down to the correct number of rows.
    // This is necessary until Simbody issue #401 is addressed.
    matrix.resizeKeep(curRow, ncol);

    // Create the table and update other metadata from above
    auto table = 
        std::make_shared<TimeSeriesTable_<T>>(timeVec, matrix, column_labels);
    table->updTableMetaData() = keyValuePairs;

    OutputTables output_tables{};
    output_tables.emplace(tableString(), table);

    return output_tables;
}

template<typename T>
SimTK::RowVector_<T>
DelimFileAdapter<T>::readElems(const std::vector<std::string>& tokens) const {
    return readElems_impl(tokens, T{});
}

template<typename T>
SimTK::RowVector_<double>
DelimFileAdapter<T>::readElems_impl(const std::vector<std::string>& tokens,
                                    double) const {
    SimTK::RowVector_<double> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i)
        elems[static_cast<int>(i)] = std::stod(tokens[i]);

    return elems;
}

template<typename T>
SimTK::RowVector_<SimTK::UnitVec3>
DelimFileAdapter<T>::readElems_impl(const std::vector<std::string>& tokens,
                                    SimTK::UnitVec3) const {
    SimTK::RowVector_<SimTK::UnitVec3> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 3, 
                         IncorrectNumTokens,
                         "Expected 3x (multiple of 3) number of tokens.");
        elems[i] = SimTK::UnitVec3{std::stod(comps[0]),
                                   std::stod(comps[1]),
                                   std::stod(comps[2])};
    }

    return elems;
}

template<typename T>
SimTK::RowVector_<SimTK::Quaternion>
DelimFileAdapter<T>::readElems_impl(const std::vector<std::string>& tokens,
                                    SimTK::Quaternion) const {
    SimTK::RowVector_<SimTK::Quaternion> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 4, 
                         IncorrectNumTokens,
                         "Expected 4x (multiple of 4) number of tokens.");
        elems[i] = SimTK::Quaternion{std::stod(comps[0]),
                                     std::stod(comps[1]),
                                     std::stod(comps[2]),
                                     std::stod(comps[3])};
    }

    return elems;
}

template<typename T>
SimTK::RowVector_<SimTK::SpatialVec>
DelimFileAdapter<T>::readElems_impl(const std::vector<std::string>& tokens,
                                    SimTK::SpatialVec) const {
    SimTK::RowVector_<SimTK::SpatialVec> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 6, 
                         IncorrectNumTokens,
                         "Expected 6x (multiple of 6) number of tokens.");
        elems[i] = SimTK::SpatialVec{{std::stod(comps[0]),
                                      std::stod(comps[1]),
                                      std::stod(comps[2])},
                                     {std::stod(comps[3]),
                                      std::stod(comps[4]),
                                      std::stod(comps[5])}};
    }

    return elems;
}

template<typename T>
template<int M>
SimTK::RowVector_<SimTK::Vec<M>>
DelimFileAdapter<T>::readElems_impl(const std::vector<std::string>& tokens,
                                    SimTK::Vec<M>) const {
    SimTK::RowVector_<SimTK::Vec<M>> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != M, 
                         IncorrectNumTokens,
                         "Expected " + std::to_string(M) +
                         "x (multiple of " + std::to_string(M) +
                         ") number of tokens.");
        for(int j = 0; j < M; ++j) {
            elems[i][j] = std::stod(comps[j]);
        }
    }

    return elems;
}
  
template<typename T>
void
DelimFileAdapter<T>::extendWrite(const InputTables& absTables, 
                                 const std::string& fileName) const {
    OPENSIM_THROW_IF(absTables.empty(),
                     NoTableFound);

    const TimeSeriesTable_<T>* table{};
    try {
        auto abs_table = absTables.at(tableString());
        table = dynamic_cast<const TimeSeriesTable_<T>*>(abs_table);
    } catch(std::out_of_range&) {
        OPENSIM_THROW(KeyMissing,
                      tableString());
    } catch(std::bad_cast&) {
        OPENSIM_THROW(IncorrectTableType);
    }

    OPENSIM_THROW_IF(fileName.empty(),
                     EmptyFileName);

    std::ofstream out_stream{fileName};

    // First line of the stream is the header.
    if (table->getTableMetaData().hasKey("header")) {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey("header").
                      template getValue<std::string>() << "\n";
    }
    // Write rest of the key-value pairs and end the header.
    for(const auto& key : table->getTableMetaDataKeys()) {
        try {
            if(key != "header")
                out_stream << key << "=" 
                           << table->
                              template getTableMetaData<std::string>(key) 
                           << "\n";
        } catch(const InvalidTemplateArgument&) {}
    }
    // Write name of the data-type -- vec3, vec6, etc.
    out_stream << _dataTypeString << "=" << dataTypeName() << "\n";
    // Write version number.
    out_stream << _versionString << "=" << _versionNumber << "\n";
    out_stream << _opensimVersionString << "=" << GetVersion() << "\n";
    out_stream << _endHeaderString << "\n";

    // Line containing column labels.
    out_stream << _timeColumnLabel;
    for(unsigned col = 0; col < table->getNumColumns(); ++col)
        out_stream << _delimiterWrite
                   << table->
                      getDependentsMetaData().
                      getValueArrayForKey("labels")[col].
                      template getValue<std::string>();
    out_stream << "\n";

    // Data rows.
    for(unsigned row = 0; row < table->getNumRows(); ++row) {
        constexpr auto prec = std::numeric_limits<double>::digits10 + 1;
        out_stream << std::setprecision(prec)
                   << table->getIndependentColumn()[row];
        const auto& row_r = table->getRowAtIndex(row);
        for(unsigned col = 0; col < table->getNumColumns(); ++col) {
            const auto& elt = row_r[col];
            out_stream << _delimiterWrite;
            writeElem(out_stream, elt, prec);
        }
        out_stream << "\n";
    }
}

template<typename T>
void
DelimFileAdapter<T>::writeElem(std::ostream& stream,
                               const T& elem,
                               const unsigned& prec) const {
    writeElem_impl(stream, elem, prec);
}

template<typename T>
void
DelimFileAdapter<T>::writeElem_impl(std::ostream& stream,
                                    const double& elem,
                                    const unsigned& prec) const {
    stream << std::setprecision(prec) << elem;
}

template<typename T>
void
DelimFileAdapter<T>::writeElem_impl(std::ostream& stream,
                                    const SimTK::SpatialVec& elem,
                                    const unsigned& prec) const {
    stream                    << std::setprecision(prec) << elem[0][0]
           << _compDelimWrite << std::setprecision(prec) << elem[0][1]
           << _compDelimWrite << std::setprecision(prec) << elem[0][2]
           << _compDelimWrite << std::setprecision(prec) << elem[1][0]
           << _compDelimWrite << std::setprecision(prec) << elem[1][1]
           << _compDelimWrite << std::setprecision(prec) << elem[1][2];
}

template<typename T>
template<int M>
void
DelimFileAdapter<T>::writeElem_impl(std::ostream& stream,
                                    const SimTK::Vec<M>& elem,
                                    const unsigned& prec) const {
    stream << std::setprecision(prec) << elem[0];
    for(auto i = 1u; i < M; ++i)
        stream << _compDelimWrite << std::setprecision(prec) << elem[i];
}

} // namespace OpenSim

#endif // OPENSIM_DELIM_FILE_ADAPTER_H_
