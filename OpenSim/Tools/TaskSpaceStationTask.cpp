/* -------------------------------------------------------------------------- *
 *                 OpenSim: TaskSpaceStationTask.cpp                          *
 * -------------------------------------------------------------------------- *
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
#include "TaskSpaceStationTask.h"

using namespace std;
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
StationTask::~StationTask() {}
//_____________________________________________________________________________
/**
 * Construct a task for a specified point.
 *
 */
StationTask::StationTask() : BodyTask()
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Point task to be copied.
 */
StationTask::StationTask(const StationTask& aTask)
        : BodyTask(aTask) {
    setNull();
    copyData(aTask);
    set_point(aTask.get_point());
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void StationTask::setNull() 
{
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
 * @brief Construct properties for the task.
 *
 */
void StationTask::constructProperties() 
{
    setName("StationTask");
    constructProperty_point(SimTK::Vec3(0.0));

    constructProperty_left_foot();
    constructProperty_right_foot();
}

//_____________________________________________________________________________
/**
 * Copy only the member data of specified object.
 */
void StationTask::copyData(const StationTask& aTask) 
{
    //_model = aTask.getModel();
    copyProperty_point(aTask);
    copyProperty_left_foot(aTask);
    copyProperty_right_foot(aTask);
}

//_____________________________________________________________________________
/**
 * Update work variables
 */
void StationTask::updateWorkVariables(const SimTK::State& s) 
{
    int j = 0;
    _p = 0;
    _v = 0;
    if (hasModel()) {
        const BodySet& bs = getModel().getBodySet();
        if (s.getSystemStage() < SimTK::Stage::Velocity){
            getModel().getMultibodySystem().realize(s, SimTK::Stage::Velocity);
        }
        
        if (get_wrt_body() == "center_of_mass") {
            // COMPUTE COM OF WHOLE BODY
            _p = Vector(getModel().calcMassCenterPosition(s));
            _v = Vector(getModel().calcMassCenterVelocity(s));
            if (_v[0] != _v[0]){
                std::cout << "name " << getName() << std::endl;
                std::cout << "time " << s.getTime() << std::endl;
                std::cout << "v0 0 " << _v[0] << std::endl;
                std::cout << "v0 1 " << _v[0] << std::endl;
                std::cout << "v0 2 " << _v[0] << std::endl;
                std::cout << "p0 0 " << _p[0] << std::endl;
                std::cout << "p0 1 " << _p[0] << std::endl;
                std::cout << "p0 2 " << _p[0] << std::endl;
                OPENSIM_THROW(OpenSim::Exception,
                "StationTask.computeAccelerations -  references invalid velocitycomponents");
            }

            if (_p[0] != _p[0]){
                OPENSIM_THROW(OpenSim::Exception,
                "StationTask.computeAccelerations -  references invalid position components");
            }

        } else {
            _wrtBody = &bs.get(get_wrt_body());
            SimTK::Vec3 _point = get_point();
            _p = Vector(_wrtBody->findStationLocationInGround(s, _point));
            if (_p[0] != _p[0]){
                OPENSIM_THROW(OpenSim::Exception,
                "StationTask.updateWorkVariables -  references invalid position components");
            }

            _v = Vector(_wrtBody->findStationVelocityInGround(s, _point));
            if (_v[0] != _v[0]){
                OPENSIM_THROW(OpenSim::Exception,
                "StationTask.updateWorkVariables -  references invalid velocity components");
            }
        }
    }
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aTask Object to be copied.
 * @return  Reference to the altered object.
 */
StationTask& StationTask::operator=(const StationTask& aTask) 
{
    // BASE CLASS
    TaskSpaceTask::operator=(aTask);

    // DATA
    copyData(aTask);
    return (*this);
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the position and velocity errors.
 * This method assumes the states have been set for the model.
 *
 * @param aT Current time in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void StationTask::computeErrors(const SimTK::State& s, double aT) 
{
    updateWorkVariables(s);

    // COMPUTE ERRORS
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

    } else if (get_express_body() == "base_of_support") {
        SimTK::Vec3 pVec, vVec, origin;
        for (int i = 0; i < 3; ++i) {
            pVec(i) = get_position_functions(i).calcValue(SimTK::Vector(1, aT));
        }
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
        auto& left_foot_name = get_left_foot();
        auto& right_foot_name = get_right_foot();
        auto& foot_r = bs.get(right_foot_name);
        auto& foot_l = bs.get(left_foot_name);
        SimTK::Vec3 foot_r_p = foot_r.findStationLocationInGround(
                s, foot_r.getMassCenter());
        SimTK::Vec3 foot_l_p = foot_l.findStationLocationInGround(
                s, foot_l.getMassCenter());
        auto& _point = get_point();
        SimTK::Vec3 bosp = 0.5 * (foot_r_p + foot_l_p) + _point;
        SimTK::Vec3 bosv =
                getModel().getGround().findStationVelocityInGround(s, bosp);
        for (int i = 0; i < 3; ++i) {
            _inertialPTrk[i] += bosp[i];
            _inertialVTrk[i] += bosv[i];
        }

    } else {
        _expressBody = &bs.get(get_express_body());

        SimTK::Vec3 pVec, vVec, origin;

        for (int i = 0; i < 3; ++i) {
            pVec(i) = get_position_functions(i).calcValue(SimTK::Vector(1, aT));
        }
        _inertialPTrk =
                Vector(_expressBody->findStationLocationInGround(s, pVec));
        if (getProperty_velocity_functions().size() == 0) {
            // FOR REVIEW: Is this correct? Shouldn't it use the derivative of
            // the position function?
            _inertialVTrk =
                    Vector(_expressBody->findStationVelocityInGround(s, pVec));
        } else {
            for (int i = 0; i < 3; ++i) {
                vVec(i) = get_velocity_functions(i).calcValue(
                        SimTK::Vector(1, aT));
            }
            // FOR REVIEW: Should origin be get_point() instead?
            auto& _point = get_point();
            _inertialVTrk = Vector(
                    _expressBody->findStationVelocityInGround(s, _point));
            _inertialVTrk +=
                    Vector(vVec); // _vTrk is velocity in _expressBody, so it is
                                  // simply added to velocity of _expressBody
                                  // origin in inertial frame
        }
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
 * Compute the desired accelerations.
 */
void StationTask::computeDesiredAccelerations(
        const SimTK::State& s, double aT) {
    computeDesiredAccelerations(s, aT, aT);
}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aTI Initial time of the controlled interval in real time units.
 * @param aTF Final time of the controlled interval in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void StationTask::computeDesiredAccelerations(
        const SimTK::State& s, double aTI, double aTF) {
    _aDes = SimTK::NaN;

    checkFunctions();

    computeErrors(s, aTI);

    // DESIRED ACCELERATION
    double p;
    double v;
    double a;
    for (int i = 0; i < 3; ++i) {
        p = get_kp(i) * _pErr[i];
        v = get_kv(i) * _vErr[i];
        if (getProperty_acceleration_functions().size() == 0) {
            std::vector<int> derivComponents(2);
            derivComponents[0] = 0;
            derivComponents[1] = 0;
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
 * Compute the acceleration of the appropriate point.
 * For the computed accelerations to be correct,
 * Model::computeAccelerations() must have already been called.
 *
 * For joints (i.e., generalized coordinates), the acceleration is
 * not computed.  It has already been computed and is simply retrieved
 * from the model.
 *
 * @see Model::computeAccelerations()
 * @see suTrackObject::getAcceleration()
 */
void StationTask::computeAccelerations(const SimTK::State& s) {
    // CHECK
    if (!hasModel()) return;

    // ACCELERATION
    _a = 0;
    const BodySet& bs = getModel().getBodySet();
    if (get_wrt_body() == "center_of_mass") {
        SimTK::Vec3 pVec, vVec, aVec, com;
        double Mass = 0.0;
        for (int i = 0; i < bs.getSize(); ++i) {
            Body& body = bs.get(i);
            com = body.get_mass_center();
            aVec = body.findStationAccelerationInGround(s, com);
            if (aVec[0] != aVec[0]){
                OPENSIM_THROW(OpenSim::Exception,
                "StationTask.computeAccelerations -  references invalid body acceleration components");
            }

            //  ADD TO WHOLE BODY MASS
            Mass += body.get_mass();
            _a += SimTK::Vector(body.get_mass() * aVec);
        }
        _a /= Mass;

    } else {
        _wrtBody = &bs.get(get_wrt_body());
        auto& _point = get_point();
        _a = SimTK::Vector(
                _wrtBody->findStationAccelerationInGround(s, _point));
    }
    if (_a[0] != _a[0]){
            OPENSIM_THROW(OpenSim::Exception,
            "StationTask.computeAccelerations -  references invalid station acceleration components");
    }
}

//_____________________________________________________________________________
/**
 * Compute the Jacobian for the tracked point given the current state
 * of the model.
 */
void StationTask::computeJacobian(const SimTK::State& s) {
    if (get_wrt_body() == "center_of_mass") {
        double M = getModel().getTotalMass(s);
        SimTK::Matrix J(3, s.getNU(), 0.0);
        for (int i = 0; i < getModel().getBodySet().getSize(); ++i) {
            auto m = getModel().getBodySet()[i].get_mass();
            if (m == 0) {
                cout << "Warning: body "
                     << getModel().getBodySet()[i].getName()
                     << " has zero mass" << endl;
                continue;
            }
            auto com = getModel().getBodySet()[i].get_mass_center();
            SimTK::Matrix temp;
            getModel().getMatterSubsystem().calcStationJacobian(s,
                    getModel().getBodySet()[i].getMobilizedBodyIndex(), com,
                    temp);
            J += m * temp;
        }
        _j = J / M;
    } else {
        auto& _point = get_point();
        getModel().getMatterSubsystem().calcStationJacobian(s,
                getModel()
                        .getBodySet()
                        .get(get_wrt_body())
                        .getMobilizedBodyIndex(),
                _point, _j);
    }
}

//_____________________________________________________________________________
/**
 * Compute the Jacobian bias term for the tracked point given the current state
 * of the model.
 */
void StationTask::computeBias(const SimTK::State& s) {
    if (get_wrt_body() == "center_of_mass") {
        double M = getModel().getTotalMass(s);
        SimTK::Vec3 JdotQdot(0);
        for (int i = 0; i < getModel().getBodySet().getSize(); ++i) {
            auto m = getModel().getBodySet()[i].get_mass();
            if (m == 0) {
                cout << "Warning: body "
                     << getModel().getBodySet()[i].getName()
                     << " has zero mass" << endl;
                continue;
            }
            auto com = getModel().getBodySet()[i].get_mass_center();
            SimTK::Vec3 temp =
                    getModel().getMatterSubsystem().calcBiasForStationJacobian(
                            s,
                            getModel().getBodySet()[i].getMobilizedBodyIndex(),
                            com);
            JdotQdot += m * temp;
        }
        _b = Vector(-1.0 * JdotQdot / M);
    } else {
        auto& _point = get_point();
        SimTK::Vec3 JdotQdot =
                getModel().getMatterSubsystem().calcBiasForStationJacobian(s,
                        getModel()
                                .getBodySet()
                                .get(get_wrt_body())
                                .getMobilizedBodyIndex(),
                        _point);
        _b = Vector(-1.0 * JdotQdot);
    }
}

//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 *
 * @param aDeep If true, update this object and all its child objects
 * (that is, member variables that are Object's); if false, update only
 * the member variables that are not Object's.
 */
void StationTask::updateFromXMLNode(
        SimTK::Xml::Element& aNode, int versionNumber) {
    Super::updateFromXMLNode(aNode, versionNumber);
}
