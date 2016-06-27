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

/** STOFileAdapter is a DelimFileAdapter that presets the delimiters 
appropriately for STO files.                                                  */
class OSIMCOMMON_API STOFileAdapter : public DelimFileAdapter {
public:
    STOFileAdapter();
    STOFileAdapter(const STOFileAdapter&)            = default;
    STOFileAdapter(STOFileAdapter&&)                 = default;
    STOFileAdapter& operator=(const STOFileAdapter&) = default;
    STOFileAdapter& operator=(STOFileAdapter&&)      = default;
    ~STOFileAdapter()                                = default;

    STOFileAdapter* clone() const override;

    /** Read a STO file.                                                      */
    static
    TimeSeriesTable read(const std::string& fileName);

    /** Write a STO file.                                                     */
    static
    void write(const TimeSeriesTable& table, const std::string& fileName);
};

}

#endif // OPENSIM_STO_FILE_ADAPTER_H_
