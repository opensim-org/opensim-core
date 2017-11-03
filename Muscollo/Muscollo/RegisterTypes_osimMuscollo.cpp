/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: RegisterTypes_osimMuscollo.cpp                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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
#include "RegisterTypes_osimMuscollo.h"
#include <OpenSim/Common/Object.h>

#include "MucoCost.h"
#include "MucoProblem.h"
#include "MucoSolver.h"
#include "MucoTool.h"
#include "MucoTropterSolver.h"
#include "InverseMuscleSolver/GlobalStaticOptimization.h"
#include "InverseMuscleSolver/INDYGO.h"
#include "MucoStateTrackingCost.h"

#include <exception>
#include <iostream>

using namespace OpenSim;

static osimMuscolloInstantiator instantiator;

OSIMMUSCOLLO_API void RegisterTypes_osimMuscollo() {
    try {
        Object::registerType(MucoFinalTimeCost());
        Object::registerType(MucoStateTrackingCost());
        Object::registerType(MucoPhase());
        Object::registerType(MucoVariableInfo());
        Object::registerType(MucoProblem());
        Object::registerType(MucoTool());
        Object::registerType(MucoTropterSolver());


        Object::registerType(GlobalStaticOptimization());
        Object::registerType(INDYGO());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMuscollo Object registration:\n"
                << e.what() << std::endl;
    }
}

osimMuscolloInstantiator::osimMuscolloInstantiator() {
    registerDllClasses();
}

void osimMuscolloInstantiator::registerDllClasses() {
    RegisterTypes_osimMuscollo();
}