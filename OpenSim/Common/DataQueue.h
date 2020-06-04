#ifndef OPENSIM_DATA_QUEUE_H_
#define OPENSIM_DATA_QUEUE_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  DataQueue.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2020 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

#include <OpenSim/Common/osimCommonDLL.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This base (abstract) class defines the interface for a DataQueue. A data structure
 * to maintain a queue of data to be passed between computations that are potentially
 * different in processing speeds, decoupling the producers (e.g. File or live stream)
 * from consumers. Synchronization mechanism will be implmented to allow handling of 
 * multiple threads or significant differences in speeds.
 *
 * @author Ayman Habib
 */
template<class T> class DataQueue_ {
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    virtual ~DataQueue_() {}
    
    DataQueue_() = default;

    //--------------------------------------------------------------------------
    // DataQueue Interface
    //--------------------------------------------------------------------------

private:
    std::queue<std::tuple<double, SimTK::Array_<T>>> m_data_queue;
    //=============================================================================
};  // END of class templatized DataQueue_<T>
//=============================================================================
}

#endif // OPENSIM_DATA_QUEUE_H_
