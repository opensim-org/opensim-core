#ifndef OPENSIM_PROBE_SET_H_
#define OPENSIM_PROBE_SET_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  ProbeSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
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
#include <OpenSim/Simulation/Model/Probe.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {


//=============================================================================
//=============================================================================
/**
 * A class for holding a set of probes.
 *
 * @authors Tim Dorn
 * @version 1.0
 */

class OSIMSIMULATION_API ProbeSet : public ModelComponentSet<Probe> {
OpenSim_DECLARE_CONCRETE_OBJECT(ProbeSet, ModelComponentSet<Probe>);

public:
    ProbeSet();
    ProbeSet(const ProbeSet& aAbsProbeSet);


//=============================================================================
};  // END of class ProbeSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PROBE_SET_H_
