/* -------------------------------------------------------------------------- *
 *                          OpenSim:  FileAdapter.h                           *
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

#ifndef OPENSIM_FILE_ADAPTER_H_
#define OPENSIM_FILE_ADAPTER_H_

/** @file
* This file defines an abstract FileAdapter class, which implements the 
  OpenSim::DataAdpater interface for reading or writing data files.
*/
#include "DataAdapter.h"

#include <vector>

namespace OpenSim {

/** FileAdapter class for constructing DataAdapters that specifically access
data sources that are files. It provides utilities for resolving paths and
format parsing.
Concrete classes handle the individual formats and specific DataTypes.        */
class FileAdapter : public DataAdapter {
public:
    FileAdapter()                              = default;
    FileAdapter(const FileAdapter&)            = default;
    FileAdapter(FileAdapter&&)                 = default;
    FileAdapter& operator=(const FileAdapter&) = default;
    FileAdapter& operator=(FileAdapter&&)      = default;
    virtual ~FileAdapter()                     = default;
 
    static OutputTables readFile(const std::string& fileName);

    static void writeFile(const InputTables& tables, 
                          const std::string& fileName);
    
    static
    std::string findExtension(const std::string& filename);

    std::vector<std::string> tokenize(const std::string& str, 
                                      const std::string& delims) const;
};

} // OpenSim namespace

#endif // OPENSIM_FILE_ADAPTER_H_
