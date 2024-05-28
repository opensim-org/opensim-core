/* -------------------------------------------------------------------------- *
 *                 OpenSim: TaskSpaceInertiaTask.cpp                          *
 * -------------------------------------------------------------------------- *
 * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own work:                 *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "OpenSim/Simulation/Model/Model.h"
#include "TaskSpaceInertiaTask.h"

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
InertiaTask::~InertiaTask() {}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InertiaTask::InertiaTask() {
    setupProperties();
    setNull();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 */
InertiaTask::InertiaTask(const InertiaTask& aTask) : BodyTask(aTask) {
    setNull();
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void InertiaTask::setNull() {
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

void InertiaTask::setupProperties() {
    // Set default direction vectors
    set_direction_vectors(0, SimTK::Vec3(1, 0, 0));
    set_direction_vectors(1, SimTK::Vec3(0, 1, 0));
    set_direction_vectors(2, SimTK::Vec3(0, 0, 1));
    setName("InertiaTask");
    // Set default gains
    set_kp(Array<double>(100, 3));
    set_kv(Array<double>(20, 3));
    set_ka(Array<double>(1, 3));
}
// void InertiaTask::setTaskFunctions(int i, Function* aF0) {
//     if (aF0 != NULL) set_position_functions(i, *aF0);
// }

//_____________________________________________________________________________
/**
 * Update work variables
 */
void InertiaTask::updateWorkVariables(const SimTK::State& s) {
    _p = 0;
    _v = 0;
    if (hasModel()) {
        const BodySet& bs = getModel().getBodySet();

        if (s.getSystemStage() < SimTK::Stage::Velocity) {
            getModel().getMultibodySystem().realize(s, SimTK::Stage::Velocity);
        }

        SimTK::Vec3 pVec, vVec;
        double Mass = 0.0;
        _p = Vector(getModel()
                            .getMatterSubsystem()
                            .calcSystemCentralMomentum(s)
                            .get(0));
        _v = Vector(getModel()
                            .getMatterSubsystem()
                            .calcSystemCentralMomentum(s)
                            .get(0));
        // COMPUTE COM OF WHOLE BODY
        if (_v[0] != _v[0]){
            OPENSIM_THROW(OpenSim::Exception,
                "StationTask.computeAccelerations -  references invalid velocity components");
        }

        if (_p[0] != _p[0]){
            OPENSIM_THROW(OpenSim::Exception,
                "StationTask.computeAccelerations -  references invalid position components");
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
void InertiaTask::computeErrors(const SimTK::State& s, double aT) {
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
        _inertialVTrk =
                Vector((SimTK::Pi / 180) *
                                _expressBody->getAngularVelocityInGround(s) +
                        wVecInertial);
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

    // DEBUGGING
     log_debug("t: {}", s.getTime());
     log_debug("InertiaTask {}", this->get_wrt_body());
     log_debug("_inertialPTrk = {}", _inertialPTrk);
     log_debug("_inertialVTrk = {}", _inertialVTrk);
     log_debug("_p = {}", _p);
     log_debug("_v = {}", _v);
     log_debug("_pErr = {}", _pErr);
     log_debug("_vErr = {}", _vErr);
}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 */
void InertiaTask::computeDesiredAccelerations(
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
void InertiaTask::computeDesiredAccelerations(
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
            // derivComponents[0]=0;
            // derivComponents[1]=0;
            a = get_ka(i) * get_position_functions(i).calcDerivative(
                                    derivComponents, SimTK::Vector(1, aTF));
        } else {
            a = get_ka(i) *
                get_acceleration_functions(i).calcValue(SimTK::Vector(1, aTF));
        }
        _aDes[i] = get_weight(i) * (a + v + p);

        // PRINT
        // printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf
        // pErr=%lf\n",getWRTBodyName().c_str(),aTF,_aDes[i],
        //         a,_vErr[i],_pErr[i]);
        // if (i==2) std::cout << std::endl;
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
void InertiaTask::computeAccelerations(const SimTK::State& s) {
    // CHECK
    if (!hasModel()) return;

    // ACCELERATION
    _a = 0;
    const BodySet& bs = getModel().getBodySet();

    _a = SimTK::Vector(
            getModel().getMatterSubsystem().calcSystemCentralMomentum(s).get(
                    0));
    if (_a[0] != _a[0])
        throw Exception(
                "InertiaTask.computeAccelerations: ERROR- orientation task '" +
                        getName() +
                        "' references invalid acceleration components",
                __FILE__, __LINE__);
}

//_____________________________________________________________________________
/**
 * @brief Compute the Jacobian.
 *
 * @param s SimTK::State
 */
void InertiaTask::computeJacobian(const SimTK::State& s) {
    double M = getModel().getTotalMass(s);
    SimTK::Matrix J(3, s.getNU(), 0.0);
    for (int i = 0; i < getModel().getBodySet().getSize(); ++i) {
        auto m = getModel().getBodySet()[i].get_mass();
        if (m == 0) {
            // std::cout << "Warning: body "
            //           << getModel().getBodySet()[i].getName()
            //           << " has zero mass" << std::endl;
            continue;
        }
        auto com = getModel().getBodySet()[i].get_mass_center();
        SimTK::Matrix temp;
        getModel().getMatterSubsystem().calcStationJacobian(s,
                getModel().getBodySet()[i].getMobilizedBodyIndex(), com, temp);
        J += m * temp;
    }
    _j = J / M;
}

//_____________________________________________________________________________
/**
 * @brief Compute the Jacobian bias
 *
 * @param s SimTK::State
 */
void InertiaTask::computeBias(const SimTK::State& s) {
    double M = getModel().getTotalMass(s);
    SimTK::Vec3 JdotQdot(0);
    for (int i = 0; i < getModel().getBodySet().getSize(); ++i) {
        auto m = getModel().getBodySet()[i].get_mass();
        if (m == 0) {
            // std::cout << "Warning: body "
            //           << getModel().getBodySet()[i].getName()
            //           << " has zero mass" << std::endl;
            continue;
        }
        auto com = getModel().getBodySet()[i].get_mass_center();
        SimTK::Vec3 temp =
                getModel().getMatterSubsystem().calcBiasForStationJacobian(s,
                        getModel().getBodySet()[i].getMobilizedBodyIndex(),
                        com);
        JdotQdot += m * temp;
    }
    _b = Vector(-1.0 * JdotQdot / M);
}

// void InertiaTask::update(const SimTK::State& s) {
//     computeBias(s);
//     computeDesiredAccelerations(s, s.getTime());
//     computeJacobian(s);
// }