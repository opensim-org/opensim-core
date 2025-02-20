/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMocoCustomEffortGoal.cpp                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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
#include "MocoCustomEffortGoal.h"
#include "RegisterTypes_osimMocoCustomEffortGoal.h"

using namespace OpenSim;

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimMocoCustomEffortGoalInstantiator instantiator;
#endif

OSIMMOCOCUSTOMEFFORTGOAL_API void RegisterTypes_osimMocoCustomEffortGoal() {
    try {
        Object::registerType(MocoCustomEffortGoal());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMocoCustomEffortGoal "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoCustomEffortGoalInstantiator::osimMocoCustomEffortGoalInstantiator() {
    registerDllClasses();
}

void osimMocoCustomEffortGoalInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoCustomEffortGoal();
}
