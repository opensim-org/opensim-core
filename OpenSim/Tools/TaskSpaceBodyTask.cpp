/* -------------------------------------------------------------------------- *
 *                         OpenSim: BodyTask.cpp                              *
 * -------------------------------------------------------------------------- *
 * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own research:                 *
 *                                                                            *
 * Pickle and Sundararajan. "Predictive simulation of human movement in       *
 * OpenSim using floating-base task space control".                           *
 *                                                                            *
 * Copyright (c) 2023 CFD Research Corporation and the Authors                *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Nathan Pickle, Garrett Tuer               *
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

#include "TaskSpaceBodyTask.h"

#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

BodyTask::BodyTask(){
    constructProperties();
    setNull();
}

void BodyTask::constructProperties() {
    constructProperty_wrt_body("");
    constructProperty_express_body("ground");
    constructProperty_direction_vectors();
    // Set default direction vectors
    set_direction_vectors(0, SimTK::Vec3(1, 0, 0));
    set_direction_vectors(1, SimTK::Vec3(0, 1, 0));
    set_direction_vectors(2, SimTK::Vec3(0, 0, 1));

    set_kp(Array<double>(100, 3));
    set_kv(Array<double>(20, 3));
    set_ka(Array<double>(1, 3));
    set_weight(Array<double>(1, 3));
}

void BodyTask::copyData(const BodyTask& aTask) {
    
    copyProperty_wrt_body(aTask);
    copyProperty_express_body(aTask);
    copyProperty_direction_vectors(aTask);
}

BodyTask& BodyTask::operator=(const BodyTask& aTask) {
    // BASE CLASS
    Object::operator=(aTask);

    // DATA
    copyData(aTask);

    return (*this);
}

void BodyTask::setNull() {
    setName(DEFAULT_NAME);

    //_model = NULL;
    _nTrk = 0;
    _pErrLast = SimTK::Vector(Vec3(0.0));
    _pErr = SimTK::Vector(Vec3(0.0));
    _vErrLast = SimTK::Vector(Vec3(0.0));
    _vErr = SimTK::Vector(Vec3(0.0));
    _aDes = SimTK::Vector(Vec3(0.0));
    _p = SimTK::Vector(Vec3(0.0));
    _v = SimTK::Vector(Vec3(0.0));
    _inertialPTrk = SimTK::Vector(Vec3(0.0));
    _inertialVTrk = SimTK::Vector(Vec3(0.0));
    _a = SimTK::Vector(Vec3(0.0));
    _j = SimTK::Vector(Vec3(0.0));
}

void BodyTask::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
}

void BodyTask::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
}