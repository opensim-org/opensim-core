/* -------------------------------------------------------------------------- *
 *                          OpenSim:  STOFileAdapter.h                        *
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

#ifndef OPENSIM_STO_FILE_ADAPTER_H_
#define OPENSIM_STO_FILE_ADAPTER_H_

#include "DelimFileAdapter.h"


namespace OpenSim {

class STODataTypeNotSupported : public Exception {
public:
    STODataTypeNotSupported(const std::string& file,
                            size_t line,
                            const std::string& func,
                            const std::string& datatype) :
        Exception(file, line, func) {
        std::string msg = "Datatype '" + datatype + "' is not supported.";

        addMessage(msg);
    }
};

class STODataTypeNotFound : public Exception {
public:
    STODataTypeNotFound(const std::string& file,
                        size_t line,
                        const std::string& func) :
        Exception(file, line, func) {
        std::string msg = "DataType not specified in the header.";

        addMessage(msg);
    }
};

/** STOFileAdapter is a DelimFileAdapter that presets the delimiters 
appropriately for STO files.                                                  */
template<typename T>
class OSIMCOMMON_API STOFileAdapter_ : public DelimFileAdapter<T> {
public:
    STOFileAdapter_();
    STOFileAdapter_(const STOFileAdapter_&)            = default;
    STOFileAdapter_(STOFileAdapter_&&)                 = default;
    STOFileAdapter_& operator=(const STOFileAdapter_&) = default;
    STOFileAdapter_& operator=(STOFileAdapter_&&)      = default;
    ~STOFileAdapter_()                                = default;

    STOFileAdapter_* clone() const override;

    /** Read a STO file.                                                      */
    static
    TimeSeriesTable_<T> read(const std::string& fileName);

    /** Write a STO file.                                                     */
    static
    void write(const TimeSeriesTable_<T>& table, const std::string& fileName);
};

template<typename T>
STOFileAdapter_<T>::STOFileAdapter_() :
    DelimFileAdapter<T>(" \t", // delimites for read between elements
                        "\t",  // delimiter for write between elements
                        ",",   // delim for reading components(within element)
                        ","    // delim for writing components(within element)
                        ) {}

template<typename T>
STOFileAdapter_<T>*
STOFileAdapter_<T>::clone() const {
    return new STOFileAdapter_{*this};
}

template<typename T>
TimeSeriesTable_<T>
STOFileAdapter_<T>::read(const std::string& fileName) {
    auto abs_table = STOFileAdapter_{}.
                     extendRead(fileName).
                     at(DelimFileAdapter<T>::_table);
    return static_cast<TimeSeriesTable_<T>&>(*abs_table);
}

template<typename T>
void 
STOFileAdapter_<T>::write(const TimeSeriesTable_<T>& table, 
                         const std::string& fileName) {
    DataAdapter::InputTables tables{};
    tables.emplace(DelimFileAdapter<T>::_table, &table);
    STOFileAdapter_{}.extendWrite(tables, fileName);
}

std::shared_ptr<DataAdapter> 
createSTOFileAdapterForReading(const std::string& fileName);

std::shared_ptr<DataAdapter>
createSTOFileAdapterForWriting(const AbstractDataTable& table);

}

#endif // OPENSIM_STO_FILE_ADAPTER_H_
