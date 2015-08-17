/* -------------------------------------------------------------------------- *
 *                            OpenSim:  DataAdapter.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
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


#ifndef OPENSIM_COMMON_DATAADAPTER_H
#define OPENSIM_COMMON_DATAADAPTER_H

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"
#include "TimeSeriesTable.h"

// Standard headers.
#include <string>
#include <unordered_map>
#include <memory>


namespace OpenSim {

/**
 * DataAdapter is an abstract class defining an interface for reading/writing
 * in/out the contents of a DataTable. It enables access to/from various data
 * sources/sinks such as: streams, files, databases and devices. The DataTable
 * is independent of the form and format of the data in/out of the source/sink.
 * Concrete classes handle the details (e.g. format, sequential access, etc...) 
 * associated with a particular data source/sink.
 *
 * The base DataAdapter contains a static registry to serve as a factory for 
 * concrete DataAdpaters, given a string identifier of the type of adapter.
 * The adapter knows the source format and data flow (read, write, both).
 * String identifiers can be associated with file formats according to known
 * file extensions.                                                           */
class DataAdapter {
public:
    using RegisteredDataAdapters = 
        std::unordered_map<std::string, std::unique_ptr<DataAdapter>>;

    virtual DataAdapter* clone() const = 0;

    DataAdapter() = default;
    DataAdapter(const DataAdapter&) = default;

    virtual ~DataAdapter() {}

    /** Register a concrete DataAdapter by its unique string identifier.
        Registration permits access to the required concrete adapter by
        identifier lookup. As such, identifiers must be unique, but adapters may
        be registered with multiple identifiers. For example, a data file may 
        have multiple valid extensions (e.g. ".jpg: and ".jpeg") in which case
        both extensions would be valid identifiers for the same adapter. If an
        identifier is already in use an Exception is thrown. 
    @param[in] identifier string used to uniquely identify the required
               concrete adapter to process a source/sink. For file adapters,
               register unique file extension(s) in order for OpenSim to read/
               write different file formats.
    @param[in] adapter the concrete DataAdapter required to process the type
    (format) of the data for the source/sink.                                 */
    static
    void registerDataAdapter(const std::string& identifier,
                             const DataAdapter& adapter);

    /** Creator of concrete DataAdapter(s) for the specified source type by its
        unique identifier (string). For example, for file based sources, a 
        component can acquire the necessary adapter instance to read the data
        by the file extension if the extension is used as its identifier.
    @param[in] identifier string used to uniquely identify the concrete adapter
               that processes data from/to sources/sinks of a particular type.
               For file adapters, .                                           */
    static
    std::unique_ptr<DataAdapter> createAdapter(const std::string& identifier);

    /** Give the adapter an opportunity to prepare itself for reading data
        into a DataTable. Adapter keeps a reference to the DataTable for
        all subsequent read access.
        @param[out] table  The DataTable to be filled by the DataAdapter.     */
    virtual void prepareForReading(AbstractDataTable& datatable) = 0;

    /** Read in rows of the DataTable from this adapter's data source.
    If a file this can be all the rows of data. If the source is a stream
    or a device, this may be a row in time or the last few buffered rows.
    @return  done  returns true if there is no more data to be read in.       */
    virtual void read() = 0;

private:
    static RegisteredDataAdapters registered_data_adapters;
};

} // namepsace OpenSim

#endif // OPENSIM_COMMON_DATAADAPTER_H
