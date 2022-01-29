#ifndef OPENSIM_REGISTERTYPES_OSIMSIMONFRAMEORIENTATIONCONSTRAINT_H 
#define OPENSIM_REGISTERTYPES_OSIMSIMONFRAMEORIENTATIONCONSTRAINT_H 
/* -------------------------------------------------------------------------- *
 * OpenSim: RegisterTypes_osimMocoCustomEffortGoal.h                          *
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

#include "osimSimonFrameOrientationConstraintDLL.h"

extern "C" {

OSIMMOCOCUSTOMEFFORTGOAL_API void RegisterTypes_osimSimonFrameOrientationConstraint();
OSIMMOCOCUSTOMEFFORTGOAL_API void RegisterTypes_osimSimonFrameOrientationConstraintPair();
}

class osimSimonFrameOrientationConstraintInstantiator {
public:
    osimSimonFrameOrientationConstraintInstantiator();
private:
    void registerDllClasses();
};

class osimSimonFrameOrientationConstraintPairInstantiator {
public:
    osimSimonFrameOrientationConstraintPairInstantiator();
private:
    void registerDllClasses();
};
#endif // OPENSIM_REGISTERTYPES_OSIMSIMONFRAMEORIENTATIONCONSTRAINT_H 
