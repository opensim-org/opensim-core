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

#include "SimTKcommon.h"

#include "FileAdapter.h"
#include "TimeSeriesTable.h"

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

/** DelimFileAdapter is a FileAdapter that reads and writes text files with
given delimiters. CSVFileAdapter and MOTFileAdapter derive from this class and
set the delimiters appropriately for the files they parse. The read/write
functions return/accept a specific type of DataTable referred to as Table in 
this class.                                                                   
Header in the file is assumed to end with string "endheader" occupying a full
line.                                                                         */
template<typename T>
class OSIMCOMMON_API DelimFileAdapter : public FileAdapter {
    static_assert(std::is_same<T, double           >::value ||
                  std::is_same<T, SimTK::Vec2      >::value ||
                  std::is_same<T, SimTK::Vec3      >::value ||
                  std::is_same<T, SimTK::Vec4      >::value ||
                  std::is_same<T, SimTK::Vec5      >::value ||
                  std::is_same<T, SimTK::Vec6      >::value ||
                  std::is_same<T, SimTK::UnitVec3  >::value ||
                  std::is_same<T, SimTK::Quaternion>::value ||  
                  std::is_same<T, SimTK::SpatialVec>::value,
                  "Template argument T must be one of the following types : "
                  "double, SimTK::Vec2, SimTK::Vec3, SimTK::Vec4, SimTK::Vec5, "
                  "SimTK::Vec6, SimTK::UnitVec, SimTK::Quaternion, "
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
    static const std::string _table;

    /** Name of the data type T (template parameter).                         */
    static const std::string dataTypeName();

protected:
    /** Implementation of the read functionality.                             */
    OutputTables extendRead(const std::string& filename) const override;

    /** Implementation of the write functionality.                            */
    void extendWrite(const InputTables& tables,
                     const std::string& filename) const override;

    /** Read elements of type T (template parameter) from a sequence of 
    tokens.                                                                   */
    SimTK::RowVector_<T> 
    readElems(const std::vector<std::string>& tokens) const;

    /** Write an element of type T (template parameter) to stream with the
    specified precision.                                                      */
    void writeElem(std::ostream& stream, 
                   const T& elem,
                   const unsigned& prec) const;

private:
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
    /** File version number.                                                  */
    static const std::string _versionNumber;
};


template<typename T>
const std::string 
DelimFileAdapter<T>::_table{"table"};

template<typename T>
const std::string 
DelimFileAdapter<T>::_endHeaderString{"endheader"};

template<>
inline const std::string 
DelimFileAdapter<double>::dataTypeName() {
    return "double";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::Vec2>::dataTypeName() {
    return "Vec2";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::Vec3>::dataTypeName() {
    return "Vec3";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::Vec4>::dataTypeName() {
    return "Vec4";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::Vec5>::dataTypeName() {
    return "Vec5";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::Vec6>::dataTypeName() {
    return "Vec6";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::UnitVec3>::dataTypeName() {
    return "UnitVec3";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::Quaternion>::dataTypeName() {
    return "Quaternion";
}

template<>
inline const std::string 
DelimFileAdapter<SimTK::SpatialVec>::dataTypeName() {
    return "SpatialVec";
}

template<typename T>
const std::string 
DelimFileAdapter<T>::_timeColumnLabel{"time"};

template<typename T>
const std::string
DelimFileAdapter<T>::_dataTypeString{"DataType"};

template<typename T>
const std::string
DelimFileAdapter<T>::_versionString{"version"};

template<typename T>
const std::string
DelimFileAdapter<T>::_versionNumber{"2"};

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

    auto table = std::make_shared<TimeSeriesTable_<T>>();

    size_t line_num{};
    // All the lines until "endheader" is header.
    std::regex endheader{R"([ \t]*)" + _endHeaderString + R"([ \t]*)"};
    std::regex keyvalue{R"((.*)=(.*))"};
    std::string header{};
    std::string line{};
    while(std::getline(in_stream, line)) {
        ++line_num;
        if(std::regex_match(line, endheader))
            break;

        // Detect Key value pairs of the form "key = value" and add them to
        // metadata.
        std::smatch matchRes{};
        if(std::regex_match(line, matchRes, keyvalue)) {
            auto key = matchRes[1].str();
            auto value = matchRes[2].str();
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
                // Discard version number. Version number is added during
                // writing. 
              } else {
                table->updTableMetaData().setValueForKey(key, value);
              }
              continue;
            }
        }

        if(header.empty())
            header = line;
        else
            header += "\n" + line;
    }
    table->updTableMetaData().setValueForKey("header", header);

    // Callable to get the next line in form of vector of tokens.
    auto nextLine = [&] {
        return getNextLine(in_stream, _delimitersRead);
    };

    // Read the line containing column labels and fill up the column labels
    // container.
    auto column_labels = nextLine();
    ++line_num;
    // Column 0 is the time column. Check and get rid of it. The data in this
    // column is maintained separately from rest of the data.
    OPENSIM_THROW_IF(column_labels[0] != _timeColumnLabel,
                     UnexpectedColumnLabel,
                     fileName,
                     _timeColumnLabel,
                     column_labels[0]);
    column_labels.erase(column_labels.begin());
    // Set the column labels as metadata.
    ValueArray<std::string> value_array{};
    for(const auto& cl : column_labels)
        value_array.upd().push_back(SimTK::Value<std::string>{cl});
    typename TimeSeriesTable_<T>::DependentsMetaData dep_metadata{};
    dep_metadata.setValueArrayForKey("labels", value_array);
    table->setDependentsMetaData(dep_metadata);

    // Read the rows one at a time and fill up the time column container and
    // the data container.
    auto row = nextLine();
    while(!row.empty()) {
        ++line_num;

        // Time is column 0.
        double time = std::stod(row.front());
        row.erase(row.begin());

        auto row_vector = readElems(row);

        OPENSIM_THROW_IF(row_vector.size() != column_labels.size(),
                         RowLengthMismatch,
                         fileName,
                         line_num,
                         column_labels.size(),
                         static_cast<size_t>(row_vector.size()));

        // Column 1 is time.
        table->appendRow(time, std::move(row_vector));

        row = nextLine();
    }

    OutputTables output_tables{};
    output_tables.emplace(_table, table);

    return output_tables;
}

template<>
inline SimTK::RowVector_<double>
DelimFileAdapter<double>::readElems(
                                 const std::vector<std::string>& tokens) const {
    SimTK::RowVector_<double> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i)
        elems[static_cast<int>(i)] = std::stod(tokens[i]);

    return elems;
}

template<>
inline SimTK::RowVector_<SimTK::Vec2>
DelimFileAdapter<SimTK::Vec2>::readElems(
                                 const std::vector<std::string>& tokens) const {
    SimTK::RowVector_<SimTK::Vec2> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 2,
                         IncorrectNumTokens,
                         "Expected 2x (multiple of 2) number of tokens.");
        elems[i] = SimTK::Vec2{std::stod(comps[0]),
                               std::stod(comps[1])};
    }

    return elems;
}

template<>
inline SimTK::RowVector_<SimTK::Vec3>
DelimFileAdapter<SimTK::Vec3>::readElems(
                                 const std::vector<std::string>& tokens) const {
    SimTK::RowVector_<SimTK::Vec3> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 3, 
                         IncorrectNumTokens,
                         "Expected 3x (multiple of 3) number of tokens.");
        elems[i] = SimTK::Vec3{std::stod(comps[0]),
                               std::stod(comps[1]),
                               std::stod(comps[2])};
        
    }

    return elems;
}

template<>
inline SimTK::RowVector_<SimTK::Vec4>
DelimFileAdapter<SimTK::Vec4>::readElems(
                                 const std::vector<std::string>& tokens) const {
    SimTK::RowVector_<SimTK::Vec4> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 4, 
                         IncorrectNumTokens,
                         "Expected 4x (multiple of 4) number of tokens.");
        elems[i] = SimTK::Vec4{std::stod(comps[0]),
                               std::stod(comps[1]),
                               std::stod(comps[2]),
                               std::stod(comps[3])};
        
    }

    return elems;
}

template<>
inline SimTK::RowVector_<SimTK::Vec5>
DelimFileAdapter<SimTK::Vec5>::readElems(
                                 const std::vector<std::string>& tokens) const {
    SimTK::RowVector_<SimTK::Vec5> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 5, 
                         IncorrectNumTokens,
                         "Expected 5x (multiple of 5) number of tokens.");
        elems[i] = SimTK::Vec5{std::stod(comps[0]),
                               std::stod(comps[1]),
                               std::stod(comps[2]),
                               std::stod(comps[3]),
                               std::stod(comps[4])};
        
    }

    return elems;
}

template<>
inline SimTK::RowVector_<SimTK::Vec6>
DelimFileAdapter<SimTK::Vec6>::readElems(
                                 const std::vector<std::string>& tokens) const {
    SimTK::RowVector_<SimTK::Vec6> elems{static_cast<int>(tokens.size())};
    for(auto i = 0u; i < tokens.size(); ++i) {
        auto comps = tokenize(tokens[i], _compDelimRead);
        OPENSIM_THROW_IF(comps.size() != 6, 
                         IncorrectNumTokens,
                         "Expected 6x (multiple of 6) number of tokens.");
        elems[i] = SimTK::Vec6{std::stod(comps[0]),
                               std::stod(comps[1]),
                               std::stod(comps[2]),
                               std::stod(comps[3]),
                               std::stod(comps[4]),
                               std::stod(comps[5])};
        
    }

    return elems;
}

template<>
inline SimTK::RowVector_<SimTK::UnitVec3>
DelimFileAdapter<SimTK::UnitVec3>::readElems(
                                 const std::vector<std::string>& tokens) const {
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

template<>
inline SimTK::RowVector_<SimTK::Quaternion>
DelimFileAdapter<SimTK::Quaternion>::readElems(
                                 const std::vector<std::string>& tokens) const {
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

template<>
inline SimTK::RowVector_<SimTK::SpatialVec>
DelimFileAdapter<SimTK::SpatialVec>::readElems(
                                 const std::vector<std::string>& tokens) const {
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

// /** Read SimTK::Transform_<double>. The transform is a 3x4 matrix and the data
// in the input vector is organized column-wise -- [col0, col1, col2, col3]. First 
// three columns form the rotation matrix. The last column is translation.       */
// template<>
// inline SimTK::RowVector_<SimTK::Transform_<double>>
// DelimFileAdapter<SimTK::Transform_<double>>::readElems(
//                                  const std::vector<std::string>& tokens) const {
//     OPENSIM_THROW_IF(tokens.size() % 12 != 0,
//                      IncorrectNumTokens,
//                      "Expected 12x (multiple of 12) number of tokens.");

//     SimTK::RowVector_<SimTK::Transform_<double>> 
//         elems{static_cast<int>(tokens.size() / 12)};
//     for(auto i = 0u; i < tokens.size(); i += 12) {
//         SimTK::Tranform_<double> elem{};
//         auto& rotMat = elems[i / 12].updR();
//         auto& trans = elems[i / 12].updT();
        
//         for(auto c = 0u; c < 3; ++c)
//             for(auto r = 0u; r < 3; ++r)
//                 rotMat(r, c) = std::stod(tokens.at(i + (c * 3) + r));
        
//         trans[0] = std::stod(tokens.at(i + 9));
//         trans[1] = std::stod(tokens.at(i + 10));
//         trans[2] = std::stod(tokens.at(i + 11));
//     }

//     return elems;
// }

template<typename T>
void
DelimFileAdapter<T>::extendWrite(const InputTables& absTables, 
                                 const std::string& fileName) const {
    OPENSIM_THROW_IF(absTables.empty(),
                     NoTableFound);

    const TimeSeriesTable_<T>* table{};
    try {
        auto abs_table = absTables.at(_table);
        table = dynamic_cast<const TimeSeriesTable_<T>*>(abs_table);
    } catch(std::out_of_range&) {
        OPENSIM_THROW(KeyMissing,
                      _table);
    } catch(std::bad_cast&) {
        OPENSIM_THROW(IncorrectTableType);
    }

    OPENSIM_THROW_IF(fileName.empty(),
                     EmptyFileName);

    std::ofstream out_stream{fileName};

    // First line of the stream is the header.
    try {
        out_stream << table->
                      getTableMetaData().
                      getValueForKey("header").
                      template getValue<std::string>() << "\n";
    } catch(KeyNotFound&) {
        // No operation. Continue with other keys in table metadata.
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

template<>
inline void 
DelimFileAdapter<double>::writeElem(std::ostream& stream,
                                    const double& elem,
                                    const unsigned& prec) const {
    stream << std::setprecision(prec) << elem;
}

template<>
inline void 
DelimFileAdapter<SimTK::Vec2>::writeElem(std::ostream& stream,
                                         const SimTK::Vec2& elem,
                                         const unsigned& prec) const {
    stream                    << std::setprecision(prec) << elem[0]
           << _compDelimWrite << std::setprecision(prec) << elem[1];
}

template<>
inline void 
DelimFileAdapter<SimTK::Vec3>::writeElem(std::ostream& stream,
                                         const SimTK::Vec3& elem,
                                         const unsigned& prec) const {
    stream                    << std::setprecision(prec) << elem[0]
           << _compDelimWrite << std::setprecision(prec) << elem[1]
           << _compDelimWrite << std::setprecision(prec) << elem[2];
}

template<>
inline void 
DelimFileAdapter<SimTK::Vec4>::writeElem(std::ostream& stream,
                                         const SimTK::Vec4& elem,
                                         const unsigned& prec) const {
    stream << std::setprecision(prec) << elem[0];
    for(auto i = 1u; i < 4; ++i)
        stream << _compDelimWrite << std::setprecision(prec) << elem[i];
}

template<>
inline void 
DelimFileAdapter<SimTK::Vec5>::writeElem(std::ostream& stream,
                                         const SimTK::Vec5& elem,
                                         const unsigned& prec) const {
    stream << std::setprecision(prec) << elem[0];
    for(auto i = 1u; i < 5; ++i)
        stream << _compDelimWrite << std::setprecision(prec) << elem[i];
}

template<>
inline void 
DelimFileAdapter<SimTK::Vec6>::writeElem(std::ostream& stream,
                                         const SimTK::Vec6& elem,
                                         const unsigned& prec) const {
    stream << std::setprecision(prec) << elem[0];
    for(auto i = 1u; i < 6; ++i)
        stream << _compDelimWrite << std::setprecision(prec) << elem[i];
}

template<>
inline void 
DelimFileAdapter<SimTK::UnitVec3>::writeElem(std::ostream& stream,
                                             const SimTK::UnitVec3& elem,
                                             const unsigned& prec) const {
    stream                    << std::setprecision(prec) << elem[0]
           << _compDelimWrite << std::setprecision(prec) << elem[1]
           << _compDelimWrite << std::setprecision(prec) << elem[2];
}

template<>
inline void 
DelimFileAdapter<SimTK::Quaternion>::writeElem(std::ostream& stream,
                                              const SimTK::Quaternion& elem,
                                               const unsigned& prec) const {
    stream << std::setprecision(prec) << elem[0];
    for(auto i = 1u; i < 4; ++i)
        stream << _compDelimWrite << std::setprecision(prec) << elem[i];
}

template<>
inline void 
DelimFileAdapter<SimTK::SpatialVec>::writeElem(std::ostream& stream,
                                               const SimTK::SpatialVec& elem,
                                               const unsigned& prec) const {
    stream                    << std::setprecision(prec) << elem[0][0]
           << _compDelimWrite << std::setprecision(prec) << elem[0][1]
           << _compDelimWrite << std::setprecision(prec) << elem[0][2]
           << _compDelimWrite << std::setprecision(prec) << elem[1][0]
           << _compDelimWrite << std::setprecision(prec) << elem[1][1]
           << _compDelimWrite << std::setprecision(prec) << elem[1][2];
}

// /** Write SimTK::Transform_<double> to stream. The tranform is a 3x4 matrix and 
// the data is written column-wise -- [col0, col1, col2, col3]. First 3 columns 
// form the rotation. The last column is translation.                            */
// template<>
// inline void
// DelimFileAdapter<SimTK::Transform_<double>>::writeElem(std::ostream& stream,
//                                           const SimTK::Transform_<double>& elem,
//                                           const unsigned& prec) const {
//     const auto& mat34 = elem.asMat34();
//     stream << std::setprecision(prec) << mat34(0, 0)
//            << _compDelimWrite << std::setprecision(prec) << mat34(1, 0)
//            << _compDelimWrite << std::setprecision(prec) << mat34(2, 0);
//     for(auto c = 1u; c < 4; ++c)
//         for(auto r = 0u; r < 3; ++r)
//             stream << _compDelimWrite << std::setprecision(prec) << mat34(r, c);
// }

}

#endif // OPENSIM_DELIM_FILE_ADAPTER_H_
