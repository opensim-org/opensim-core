/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataAdapter.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
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

#ifndef OPENSIM_DATA_ADAPTER_H_
#define OPENSIM_DATA_ADAPTER_H_

// Non-standard headers.
#include "osimCommonDLL.h"
#include "Exception.h"
#include "AbstractDataTable.h"

// Standard headers.
#include <string>
#include <unordered_map>
#include <memory>


namespace OpenSim {

class DataAdapterAlreadyRegistered : public Exception {
public:
    DataAdapterAlreadyRegistered(const std::string& file,
                                 size_t line,
                                 const std::string& func,
                                 const std::string& key) :
        Exception(file, line, func) {
        std::string msg = "An adapter for key '" + key + "' already exists.";

        addMessage(msg);
    }
};

class NoRegisteredDataAdapter : public Exception {
public:
    NoRegisteredDataAdapter(const std::string& file,
                            size_t line,
                            const std::string& func,
                            const std::string& key) :
        Exception(file, line, func) {
        std::string msg = "No registered adapter for key '" + key + "'.";

        addMessage(msg);
    }
}; 

class TableNotFoundException : public Exception {
public:
    TableNotFoundException(const std::string& name) :
        Exception() {
        std::string msg = "No table with the name '" + name + "' was found. Check for spelling errors.";

        addMessage(msg);
    }
};
/** DataAdapter is an abstract class defining an interface for reading/writing
in/out the contents of a DataTable. It enables access to/from various data
sources/sinks such as: streams, files, databases and devices. The DataTable
is independent of the form and format of the data in/out of the source/sink.
Concrete classes handle the details (e.g. format, sequential access, etc...) 
associated with a particular data source/sink.
The base DataAdapter contains a static registry to serve as a factory for 
concrete DataAdpaters, given a string identifier of the type of adapter.
The adapter knows the source format and data flow (read, write, both).
String identifiers can be associated with file formats according to known
file extensions.                                                              */
class OSIMCOMMON_API DataAdapter {
public:
    /** Type of the registry containing registered adapters.                  */
    typedef std::map<std::string, std::shared_ptr<DataAdapter>> 
            RegisteredDataAdapters;
    /** Collection of tables returned by reading methods implemented in derived
    classes.                                                                  */
    typedef std::map<std::string, std::shared_ptr<AbstractDataTable>>
            OutputTables;
    /** Collection of tables accepted by writing methods implemented in derived
    classes.                                                                  */
    typedef std::map<std::string, const AbstractDataTable*>
            InputTables;

    virtual DataAdapter* clone() const = 0;

    DataAdapter()                              = default;
    DataAdapter(const DataAdapter&)            = default;
    DataAdapter(DataAdapter&&)                 = default;
    DataAdapter& operator=(const DataAdapter&) = default;
    DataAdapter& operator=(DataAdapter&&)      = default;
    virtual ~DataAdapter()                     = default;

    /** Register a concrete DataAdapter by its unique string identifier.
    Registration permits access to the required concrete adapter by
    identifier lookup. As such, identifiers must be unique, but adapters may
    be registered with multiple identifiers. For example, a data file may 
    have multiple valid extensions (e.g. ".jpg: and ".jpeg") in which case
    both extensions would be valid identifiers for the same adapter. If an
    identifier is already in use an Exception is thrown.
    All OpenSim data adapters are automatically registered at start of the 
    program.                                                                  */
    static
    bool registerDataAdapter(const std::string& identifier,
                             const DataAdapter& adapter);

    /** Public interface to read data from a dataSourceSpecification, typically a file or folder */
    DataAdapter::OutputTables read(const std::string& dataSourceSpecification) const {
        return extendRead(dataSourceSpecification);
    }

    /** Generic interface to retrieve a specific table by name from read result */
    const std::shared_ptr<AbstractDataTable> getDataTable(const OutputTables& tables, const std::string tableName) {
        if (tables.find(tableName) == tables.end()) {
            // Throw exception table not found, likely typo in tableName
            throw TableNotFoundException(tableName);
        }
        return tables.at(tableName);
    }
protected:
    /** Creator of concrete DataAdapter(s) for the specified source type by its
    unique identifier (string). For example, for file based sources, a 
    component can acquire the necessary adapter instance to read the data
    by the file extension if the extension is used as its identifier.         */
    static
    std::shared_ptr<DataAdapter> createAdapter(const std::string& identifier);

    /** Implements reading functionality.                                    */
    virtual OutputTables extendRead(const std::string& sourceName) const = 0;

    /** Implements writing functionality.                                     */
    virtual void extendWrite(const InputTables& tables, 
                             const std::string& sinkName) const = 0;

private:
    /** Collection of registered adapters.                                    */
    static RegisteredDataAdapters _registeredDataAdapters;
};

} // namespace OpenSim

#endif // OPENSIM_DATA_ADAPTER_H_
