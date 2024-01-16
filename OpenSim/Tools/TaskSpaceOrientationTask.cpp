/* -------------------------------------------------------------------------- *
 *                 OpenSim: TaskSpaceOrientationTask.cpp                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "OpenSim/Simulation/Model/Model.h"
#include "TaskSpaceOrientationTask.h"

using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Vector;

//=============================================================================
// STATICS
//=============================================================================

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
OrientationTask::~OrientationTask() {}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
OrientationTask::OrientationTask() {
    setupProperties();
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
OrientationTask::OrientationTask(const OrientationTask& aTask)
        : BodyTask(aTask) {
    setNull();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * @brief Set the Null object
 *
 */
void OrientationTask::setNull() {
    _nTrk = 3;
    _p = SimTK::Vector(Vec3(0.0));
    _v = SimTK::Vector(Vec3(0.0));
    _inertialPTrk = SimTK::Vector(Vec3(0.0));
    _inertialVTrk = SimTK::Vector(Vec3(0.0));
    _pErrLast = Vector(Vec3(0.0));
    _pErr = Vector(Vec3(0.0));
    _vErrLast = Vector(Vec3(0.0));
    _vErr = Vector(Vec3(0.0));
    _aDes = Vector(Vec3(0.0));
    _a = Vector(Vec3(0.0));
}
/**
 * @brief Set the properties of the task
 *
 */
void OrientationTask::setupProperties() {
    // Set default direction vectors
    set_direction_vectors(0, SimTK::Vec3(1, 0, 0));
    set_direction_vectors(1, SimTK::Vec3(0, 1, 0));
    set_direction_vectors(2, SimTK::Vec3(0, 0, 1));
    setName("OrientationTask");
    // Set default gains and weights
    set_kp(Array<double>(100, 3));
    set_kv(Array<double>(20, 3));
    set_ka(Array<double>(1, 3));
    set_weight(Array<double>(1, 3));
}

//_____________________________________________________________________________
/**
 * @brief update work variables
 *
 * @param s current workingSimTK::State
 */
void OrientationTask::updateWorkVariables(const SimTK::State& s) {
    _p = 0;
    _v = 0;
    if (hasModel()) {
        const BodySet& bs = getModel().getBodySet();

        if (s.getSystemStage() < SimTK::Stage::Velocity) {
            getModel().getMultibodySystem().realize(s, SimTK::Stage::Velocity);
        }

        _wrtBody = &bs.get(get_wrt_body());

        double dirCos[3][3];
        _model->getSimbodyEngine()
            .getDirectionCosines(s, _model->getBodySet().get(get_wrt_body()), dirCos);
        _model->getSimbodyEngine()
            .convertDirectionCosinesToAngles(dirCos, &_p[0], &_p[1], &_p[2]);
        if (_p[0] != _p[0]) {
            OPENSIM_THROW(OpenSim::Exception,"OrientationTask.updateWorkVariables: references invalid orientation components");
        }

        _v = Vector(_wrtBody->getVelocityInGround(s)[0]);
        if (_v[0] != _v[0]) {
            OPENSIM_THROW(OpenSim::Exception,"OrientationTask.updateWorkVariables: ERROR- orientation task reference invalid orientation velocity components");
        }
    }
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the angular position and velocity errors.
 * This method assumes the states have been set for the model.
 *
 * @param aT Current time in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void OrientationTask::computeErrors(const SimTK::State& s, double aT) {
    updateWorkVariables(s);

    const BodySet& bs = getModel().getBodySet();

    _inertialPTrk = 0;
    _inertialVTrk = 0;

    if (get_express_body() == "ground") {
        for (int i = 0; i < 3; ++i) {
            _inertialPTrk[i] =
                    get_position_functions(i).calcValue(SimTK::Vector(1, aT));
            if (getProperty_velocity_functions().size() == 0) {
                std::vector<int> derivComponents(1);
                derivComponents[0] = 0;
                _inertialVTrk[i] = get_position_functions(i).calcDerivative(
                        derivComponents, SimTK::Vector(1, aT));
            } else {
                _inertialVTrk[i] = get_velocity_functions(i).calcValue(
                        SimTK::Vector(1, aT));
            }
        }
    } else {
        _expressBody = &bs.get(get_express_body());

        SimTK::Vec3 oVec, wVec;
        // Retrieve the desired Euler angles (relative to _expressBody)
        for (int i = 0; i < 3; ++i) {
            oVec(i) = get_position_functions(i).calcValue(SimTK::Vector(1, aT));
        }
        // Construct a transform using the desired relative angles
        SimTK::Rotation rotation;
        rotation.setRotationToBodyFixedXYZ(oVec);
        SimTK::Transform Tdes(rotation);
        // Express desired rotations in ground
        _inertialPTrk = Vector((_expressBody->getTransformInGround(s) * Tdes)
                                       .R()
                                       .convertRotationToBodyFixedXYZ());

        for (int i = 0; i < 3; ++i) {
            if (getProperty_velocity_functions().size() == 0) {
                wVec[i] = get_position_functions(i).calcDerivative(
                        {0}, SimTK::Vector(1, aT));
            } else {
                wVec[i] = get_velocity_functions(i).calcValue(
                        SimTK::Vector(1, aT));
            }
        }
        SimTK::Vec3 wVecInertial = _expressBody->expressVectorInGround(s, wVec);
        _inertialVTrk = Vector(_expressBody->getAngularVelocityInGround(s) +
                               wVecInertial); //(SimTK::Pi/180)*
    }

    for (int k = 0; k < 3; ++k) {
        _pErr[k] = 0.0;
        _vErr[k] = 0.0;
        for (int j = 0; j < 3; ++j) {
            _pErr[k] += (_inertialPTrk[j] * get_direction_vectors(k)[j] -
                                _p[j] * get_direction_vectors(k)[j]);
            _vErr[k] += (_inertialVTrk[j] * get_direction_vectors(k)[j] -
                                _v[j] * get_direction_vectors(k)[j]);
        }
    }
}
//_____________________________________________________________________________

/**
 * @brief Compute the desired angular accelerations.
 *
 * @param s current working SimTK::State
 * @param aT delta time
 */
void OrientationTask::computeDesiredAccelerations(
        const SimTK::State& s, double aT) {
    computeDesiredAccelerations(s, aT, aT);
}
//_____________________________________________________________________________
/**
 * Compute the desired angular accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aTI Initial time of the controlled interval in real time units.
 * @param aTF Final time of the controlled interval in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void OrientationTask::computeDesiredAccelerations(
        const SimTK::State& s, double aTI, double aTF) {
    _aDes = SimTK::NaN;

    // CHECK
    checkFunctions();

    // COMPUTE ERRORS
    computeErrors(s, aTI);

    // DESIRED ACCELERATION
    double p;
    double v;
    double a;

    for (int i = 0; i < 3; ++i) {
        p = get_kp(i) * _pErr[i];
        v = get_kv(i) * _vErr[i];
        if (getProperty_acceleration_functions().size() == 0) {
            std::vector<int> derivComponents(2, 0);
            a = get_ka(i) * get_position_functions(i).calcDerivative(
                                    derivComponents, SimTK::Vector(1, aTF));
        } else {
            a = get_ka(i) *
                get_acceleration_functions(i).calcValue(SimTK::Vector(1, aTF));
        }
        _aDes[i] = get_weight(i) * (a + v + p);
    }
}

//_____________________________________________________________________________
/**
 * Compute the angular acceleration of the specified frame.
 * For the computed accelerations to be correct,
 * Model::computeAccelerations() must have already been called.
 *
 * @see Model::computeAccelerations()
 * @see suTrackObject::getAcceleration()
 */
void OrientationTask::computeAccelerations(const SimTK::State& s) {
    // CHECK
    if (!hasModel()) return;

    // ACCELERATION
    _a = 0;
    const BodySet& bs = getModel().getBodySet();

    _wrtBody = &bs.get(get_wrt_body());
    _a = SimTK::Vector(_wrtBody->getAngularAccelerationInGround(s));
    if (_a[0] != _a[0])
        OPENSIM_THROW(OpenSim::Exception,"OrientationTask.computeAccelerations: ERROR- orientation task reference invalid orientation acceleration components");
}

//_____________________________________________________________________________
/**
 * @brief Compute the Jacobian for the frame given the current state
 *
 * @param s current working SimTK::State
 */
void OrientationTask::computeJacobian(const SimTK::State& s) {
    SimTK::Matrix j_temp;
    getModel().getMatterSubsystem().calcFrameJacobian(s,
            getModel()
                    .getBodySet()
                    .get(get_wrt_body())
                    .getMobilizedBodyIndex(),
            SimTK::Vec3(0), j_temp);
    _j = j_temp(0, 0, 3, j_temp.ncol());
}

//_____________________________________________________________________________
/**
 * @brief Compute the bias for the task
 *
 * @param s current working SimTK::State
 */
void OrientationTask::computeBias(const SimTK::State& s) {
    SimTK::SpatialVec jdu =
            getModel().getMatterSubsystem().calcBiasForFrameJacobian(s,
                    getModel()
                            .getBodySet()
                            .get(get_wrt_body())
                            .getMobilizedBodyIndex(),
                    SimTK::Vec3(0));
    _b = SimTK::Vector(-1.0 * jdu[0]);
}