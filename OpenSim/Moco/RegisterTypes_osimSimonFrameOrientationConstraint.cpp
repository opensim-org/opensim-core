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
#include "SimonFrameOrientationConstraint.h"
#include "RegisterTypes_osimSimonFrameOrientationConstraint.h"

using namespace OpenSim;

static osimSimonFrameOrientationConstraintInstantiator instantiator;
static osimSimonFrameOrientationConstraintPairInstantiator instantiator2;

OSIMMOCOCUSTOMEFFORTGOAL_API void RegisterTypes_osimMocoCustomEffortGoal() {
    try {
        Object::registerType(SimonFrameOrientationConstraint());
    } catch (const std::exception& e) {
        std::cerr << "ERROR during SimonFrameOrientationConstraint "
                     "Object registration:\n"
                  << e.what() << std::endl;
    }

    try {
        Object::registerType(SimonFrameOrientationConstraintPair());
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR during SimonFrameOrientationConstraintPair "
            "Object registration:\n"
            << e.what() << std::endl;
    }
}




osimSimonFrameOrientationConstraintInstantiator::osimSimonFrameOrientationConstraintInstantiator() {
    registerDllClasses();
}

void osimSimonFrameOrientationConstraintInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoCustomEffortGoal();
}

osimSimonFrameOrientationConstraintPairInstantiator::osimSimonFrameOrientationConstraintPairInstantiator() {
    registerDllClasses();
}

void osimSimonFrameOrientationConstraintPairInstantiator::registerDllClasses() {
    RegisterTypes_osimMocoCustomEffortGoal();
}