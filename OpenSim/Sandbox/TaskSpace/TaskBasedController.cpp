/* ------------------------------------------------------------------------- *
*             OpenSim:  TaskBasedController.cpp                              *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Dimitar Stanev                                                  *
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

#include "TaskBasedController.h"

using namespace OpenSim;
using namespace SimTK;

TaskBasedController::TaskBasedController(TaskSpace& taskSpace,
    std::string endEffectorBody, Vec3 endEffectorBodyOffset)
    : m_taskSpace(taskSpace),
    m_endEffectorBody(endEffectorBody),
    m_endEffectorBodyOffset(endEffectorBodyOffset)
{
    m_isInitialized = false;
    m_desiredAcc = Vec3(0, 0, 0);
}

TaskBasedController::~TaskBasedController()
{

}

void TaskBasedController::setAcceleration(Vec3 acc)
{
    m_desiredAcc = acc;
}

void TaskBasedController::computeForce(const State& s,
    Vector_<SpatialVec>& bodyForces, Vector& generalizedForces) const
{
    if (!m_isInitialized)
    {
        m_taskSpace->addStationTask(
            _model->getBodySet().get(m_endEffectorBody).getMobilizedBodyIndex(),
            m_endEffectorBodyOffset);
        m_isInitialized = true;
    }

    _model->realizeVelocity(s);

    generalizedForces +=
        m_taskSpace->calcInverseDynamics(s, Vector(m_desiredAcc)) +
        m_taskSpace->calcGravityCompensation(s);;
}

