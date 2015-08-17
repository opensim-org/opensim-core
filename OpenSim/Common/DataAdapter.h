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

class DataAdapter {
public:
    using RegisteredDataAdapters = 
        std::unordered_map<std::string, std::unique_ptr<DataAdapter>>;

    virtual DataAdapter* clone() const = 0;

    DataAdapter() = default;
    DataAdapter(const DataAdapter&) = default;

    virtual ~DataAdapter() {}

    static
    void registerDataAdapter(const std::string& identifier,
                             const DataAdapter& adapter);

    static
    std::unique_ptr<DataAdapter> createAdapter(const std::string& identifier);

    virtual void prepareForReading(AbstractDataTable& datatable) = 0;

    virtual void read() = 0;

private:
    static RegisteredDataAdapters registered_data_adapters;
};

} // namepsace OpenSim

#endif // OPENSIM_COMMON_DATAADAPTER_H
