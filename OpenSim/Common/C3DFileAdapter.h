/* -------------------------------------------------------------------------- *
 *                          OpenSim:  C3DFileAdapter.h                        *
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

#ifndef OPENSIM_C3D_FILE_ADAPTER_H_
#define OPENSIM_C3D_FILE_ADAPTER_H_

#ifdef WITH_BTK

#include "FileAdapter.h"
#include "Event.h"

template<typename> class shrik;

namespace OpenSim {

class OSIMCOMMON_API C3DFileAdapter : public FileAdapter {
public:
    typedef std::vector<Event>                         EventTable; 
    typedef std::map<std::string, std::shared_ptr<TimeSeriesTableVec3>> Tables;

    C3DFileAdapter()                                 = default;
    C3DFileAdapter(const C3DFileAdapter&)            = default;
    C3DFileAdapter(C3DFileAdapter&&)                 = default;
    C3DFileAdapter& operator=(const C3DFileAdapter&) = default;
    C3DFileAdapter& operator=(C3DFileAdapter&&)      = default;
    ~C3DFileAdapter()                                = default;

    C3DFileAdapter* clone() const override;
    
    static
    Tables read(const std::string& fileName);

    static
    void write(const Tables& markerTable, const std::string& fileName);

    static const std::string _markers;
    static const std::string _forces;

protected:
    OutputTables extendRead(const std::string& fileName) const override;

    void extendWrite(const InputTables& tables,
                     const std::string& fileName) const override;

private:
    static const std::unordered_map<std::string, std::size_t> _unit_index;

};

} // namespace OpenSim

#endif // WITH_BTK

#endif // OPENSIM_C3D_FILE_ADAPTER_H_
