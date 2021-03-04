/* -------------------------------------------------------------------------- *
 * OpenSim JAM: RegisterTypes_osimJAM.cpp                                     *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimJAM.h"
#include "OpenSim/Simulation/Model/Blankevoort1991Ligament.h"
#include "OpenSim/Simulation/Model/Smith2018ContactMesh.h"
#include "OpenSim/Simulation/Model/Smith2018ArticularContactForce.h"
#include "JointMechanicsTool.h"
#include "ForsimTool.h"
//#include "COMAKTool.h"
//#include "COMAKInverseKinematicsTool.h"

using namespace OpenSim;
static osimJAMInstantiator instantiator;

OSIMJAM_API void RegisterTypes_osimJAM() {
    try {
    Object::registerType(Blankevoort1991Ligament());
    Object::registerType(Smith2018ContactMesh());
    Object::registerType(Smith2018ArticularContactForce());
    Object::registerType(JointMechanicsTool());
    Object::registerType(ForsimTool());
    /*Object::registerType(COMAKTool());
    Object::registerType(COMAKSecondaryCoordinate());
    Object::registerType(COMAKSecondaryCoordinateSet());
    Object::registerType(COMAKCostFunctionParameter());
    Object::registerType(COMAKCostFunctionParameterSet());
    Object::registerType(COMAKInverseKinematicsTool());*/

    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimJAM Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimJAMInstantiator::osimJAMInstantiator() { registerDllClasses(); }

void osimJAMInstantiator::registerDllClasses() { RegisterTypes_osimJAM(); }
