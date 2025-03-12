#ifndef _opensim_h_
#define _opensim_h_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  OpenSim.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "Common/osimCommon.h"
#include "Simulation/osimSimulation.h"

#include "Actuators/osimActuators.h"
#include "Analyses/osimAnalyses.h"
#include "Tools/osimTools.h"

class osimInstantiator
{
public:
    osimInstantiator() {
        RegisterTypes_osimCommon();
        RegisterTypes_osimSimulation();
        RegisterTypes_osimActuators();
        RegisterTypes_osimAnalyses();
        RegisterTypes_osimTools();
    }
};

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimInstantiator instantiator;
#endif

#endif // _opensim_h_
