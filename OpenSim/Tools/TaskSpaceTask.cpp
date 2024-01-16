/* -------------------------------------------------------------------------- *
 * OpenSim TaskSpace: TaskSpaceTask.cpp                                       *
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
 * Author(s): Aravind Sundararajan, Nathan Pickle, Garrett Tuer and Frank C.  *
 *            Anderson                                                        *
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

#include "TaskSpaceTask.h"

#include "OpenSim/Common/Constant.h"
#include "OpenSim/Simulation/Model/Model.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

TaskSpaceTask::TaskSpaceTask(){
    constructProperties();
    setNull();
}

void TaskSpaceTask::constructProperties() {
    constructProperty_priority(0);
    constructProperty_position_functions();
    constructProperty_velocity_functions();
    constructProperty_acceleration_functions();
    constructProperty_kp();
    constructProperty_kv();
    constructProperty_ka();
    constructProperty_weight();
}

void TaskSpaceTask::copyData(const TaskSpaceTask& aTask) {
    copyProperty_priority(aTask);
    copyProperty_position_functions(aTask);
    copyProperty_velocity_functions(aTask);
    copyProperty_acceleration_functions(aTask);
    copyProperty_kp(aTask);
    copyProperty_kv(aTask);
    copyProperty_ka(aTask);
    copyProperty_weight(aTask);
}

TaskSpaceTask& TaskSpaceTask::operator=(const TaskSpaceTask& aTask) {
    // BASE CLASS
    Object::operator=(aTask);

    // DATA
    copyData(aTask);

    return (*this);
}

void TaskSpaceTask::setNull() {
    setName(DEFAULT_NAME);

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

void TaskSpaceTask::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
}

void TaskSpaceTask::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
}

//-----------------------------------------------------------------------------
// VALIDATION
//-----------------------------------------------------------------------------
void TaskSpaceTask::checkFunctions() const 
{
    if (getProperty_position_functions().size() == 0) {
        log_warn("TaskSpaceTask position tracking function not set.");
        return;
    }
}

//-----------------------------------------------------------------------------
// TASK KINEMATICS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the task position.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task position.
 * @throws Exception for an invalid task.
 */
double TaskSpaceTask::getTaskPosition(int aWhich, double aT) const {
    if ((aWhich < 0) || (aWhich >= _nTrk)) {
            OPENSIM_THROW(OpenSim::Exception,
                "TaskSpaceTask:Invalid Task");
    }
    double position =
            get_position_functions(aWhich).calcValue(SimTK::Vector(1, aT));
    return (position);
}
//_____________________________________________________________________________
/**
 * Get the task velocity.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task velocity.
 * @throws Exception for an invalid task.
 */
double TaskSpaceTask::getTaskVelocity(int aWhich, double aT) const {
    if ((aWhich < 0) || (aWhich >= _nTrk)) {
            OPENSIM_THROW(OpenSim::Exception,
                "TaskSpaceTask:Invalid Task");
    }

    double velocity;
    if (getProperty_velocity_functions().size() > aWhich) {
        velocity =
                get_velocity_functions(aWhich).calcValue(SimTK::Vector(1, aT));
    } else {
        std::vector<int> derivComponents(1);
        derivComponents[0] = 0;
        velocity = get_position_functions(aWhich).calcDerivative(
                derivComponents, SimTK::Vector(1, aT));
    }

    return (velocity);
} //_____________________________________________________________________________
/**
 * Get the task acceleration.
 *
 * @param aWhich Specifies which task goal (0, 1, or 2).
 * @param aT Time (in real time units).
 * @return Task acceleration.
 * @throws Exception for an invalid task.
 */
double TaskSpaceTask::getTaskAcceleration(int aWhich, double aT) const {
    if ((aWhich < 0) || (aWhich >= _nTrk)) {
            OPENSIM_THROW(OpenSim::Exception,
                "TaskSpaceTask:Invalid Task");
    }

    double acceleration;
    if (getProperty_acceleration_functions().size() > aWhich) {
        acceleration = get_acceleration_functions(aWhich).calcValue(
                SimTK::Vector(1, aT));
    } else {
        std::vector<int> derivComponents(2);
        derivComponents[0] = 0;
        derivComponents[1] = 0;
        acceleration = get_position_functions(aWhich).calcDerivative(
                derivComponents, SimTK::Vector(1, aT));
    }
    return (acceleration);
}

//-----------------------------------------------------------------------------
// LAST POSITION ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the last achieved position error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aE0 Last position error for track goal 0.
 * @param aE1 Last position error for track goal 1.
 * @param aE2 Last position error for track goal 2.
 */
void TaskSpaceTask::setPositionErrorLast(double aE0, double aE1, double aE2) {
    _pErrLast[0] = aE0;
    _pErrLast[1] = aE1;
    _pErrLast[2] = aE2;
}
//_____________________________________________________________________________
/**
 * Get the last achieved position error for the track goal at a given index.
 * This information is useful for checking that error dynamics are being
 * followed.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Last position error.
 */
double TaskSpaceTask::getPositionErrorLast(int aWhich) const {
    if (aWhich < 0) return (0.0);
    if (aWhich >= _nTrk) return (0.0);
    return (_pErrLast[aWhich]);
}

//-----------------------------------------------------------------------------
// LAST VELOCITY ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the last achieved velocity error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aE0 Last velocity error for track goal 0.
 * @param aE1 Last velocity error for track goal 1.
 * @param aE2 Last velocity error for track goal 2.
 */
void TaskSpaceTask::setVelocityErrorLast(double aE0, double aE1, double aE2) {
    _vErrLast[0] = aE0;
    _vErrLast[1] = aE1;
    _vErrLast[2] = aE2;
}
//_____________________________________________________________________________
/**
 * Get the last achieved velocity error.  This information is useful for
 * checking that error dynamics are being followed.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Last velocity error.
 */
double TaskSpaceTask::getVelocityErrorLast(int aWhich) const {
    if (aWhich < 0) return (0.0);
    if (aWhich >= _nTrk) return (0.0);
    return (_vErrLast[aWhich]);
}

//-----------------------------------------------------------------------------
// TRACK ERRORS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the position track error of a specified track goal.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Error
 */
double TaskSpaceTask::getPositionError(int aWhich) const {
    if (aWhich < 0) return (0.0);
    if (aWhich >= _nTrk) return (0.0);
    return (_pErr[aWhich]);
}
//_____________________________________________________________________________
/**
 * Get the velocity track error of a specified track goal.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Error
 */
double TaskSpaceTask::getVelocityError(int aWhich) const {
    if (aWhich < 0) return (0.0);
    if (aWhich >= _nTrk) return (0.0);
    return (_vErr[aWhich]);
}

//-----------------------------------------------------------------------------
// DESIRED ACCELERATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the desired acceleration of a specified track goal.
 * The method computeDesiredAccelerations() must be called first for the
 * values returned by this method to be valid.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Desired acceleration.  SimTK::NaN is returned on an error.
 */
double TaskSpaceTask::getDesiredAcceleration(int aWhich) const {
    if (aWhich < 0) return (SimTK::NaN);
    if (aWhich >= _nTrk) return (SimTK::NaN);
    return (_aDes[aWhich]);
}

SimTK::Vector TaskSpaceTask::getDesiredAccelerations() const { return (_aDes); }
//-----------------------------------------------------------------------------
// ACCELERATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration of a specified track goal.  The acceleration returned
 * is the dot product of the appropriate track-goal direction and the
 * acceleration of the point or orientation in question.  In the case of
 * generalized coordinates, the acceleration of the generalized coordinate
 * is returned (i.e., a direction is not appropriate).
 *
 * For the value returned by this method to be valid, the method
 * computeAccelerations() must be called first.
 *
 * @param aWhich Specifies which track goal (0, 1, or 2).
 * @return Acceleration.  SimTK::NaN is returned on an error.
 */
double TaskSpaceTask::getAcceleration(int aWhich) const {
    if (aWhich < 0) return (SimTK::NaN);
    if (aWhich >= _nTrk) return (SimTK::NaN);
    return (_a[aWhich]);
}

//-----------------------------------------------------------------------------
// JACOBIAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get a copy of the Jacobian matrix.
 *
 * For the value returned by this method to be valid, the method
 * computeJacobian() must be called first.
 *
 * @return Jacobian.  SimTK::NaN is returned on an error.
 */
SimTK::Matrix TaskSpaceTask::getJacobian() const 
{ 
    return _j; 
}

//_____________________________________________________________________________
/**
 * Get a copy of the Jacobian bias vector.
 *
 * For the value returned by this method to be valid, the method
 * computeBias() must be called first.
 *
 * @return Bias.  SimTK::NaN is returned on an error.
 */
SimTK::Vector TaskSpaceTask::getBias() const 
{ 
    return _b; 
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
void TaskSpaceTask::update(const SimTK::State& s) {
    computeBias(s);
    computeDesiredAccelerations(s, s.getTime());
    computeJacobian(s);

    log_debug("{} desired accelerations: {}", getName(), _aDes);
    log_debug("{} position error: {}", getName(), _pErr);
    log_debug("{} velocity error: {}", getName(), _vErr);
}