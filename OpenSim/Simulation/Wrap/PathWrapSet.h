#ifndef OPENSIM_PATH_WRAP_SET_H
#define OPENSIM_PATH_WRAP_SET_H
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathWrapSet.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include "PathWrap.h"

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

/** @cond **/ // hide from Doxygen

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of muscle wrap instances.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API PathWrapSet : public Set<PathWrap> {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrapSet, Set<PathWrap>);

private:
    void setNull();
public:
    PathWrapSet();
    PathWrapSet(const PathWrapSet& aPathWrapSet);
    ~PathWrapSet(void);
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    PathWrapSet& operator=(const PathWrapSet &aPathWrapSet);
#endif
//=============================================================================
};  // END of class PathWrapSet
//=============================================================================
//=============================================================================

/** @endcond **/

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_SET_H
