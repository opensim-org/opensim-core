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

#ifndef TASK_BASED_CONTROLLER_H
#define TASK_BASED_CONTROLLER_H

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Force.h>

#include "TaskSpace.h"

namespace OpenSim
{

/**
* Computes the generalized forces based on the a task-based
* specified acceleration, in an iterative manner. Gravity
* compensation is also considered in the computation of the
* forces.
*
* @author Dimitar Stanev
*/
class TaskBasedController : public Force
{
    OpenSim_DECLARE_CONCRETE_OBJECT(TaskBasedController, Force);
public:

    TaskBasedController(TaskSpace& taskSpace,
        std::string endEffectorBody,
        SimTK::Vec3 endEffectorBodyOffset);

    ~TaskBasedController();

    void setAcceleration(SimTK::Vec3 desiredAcceleration);

protected:

    void computeForce(const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) const override;

private:

    SimTK::ReferencePtr<TaskSpace> m_taskSpace;

    std::string m_endEffectorBody;
    SimTK::Vec3 m_endEffectorBodyOffset;

    SimTK::Vec3 m_desiredAcc;

    mutable bool m_isInitialized;

}; // end of class

}  // end of namespace OpenSim

#endif
