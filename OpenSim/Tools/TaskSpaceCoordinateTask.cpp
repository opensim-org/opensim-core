/* -------------------------------------------------------------------------- *
 *                 OpenSim: TaskSpaceCoordinateTask.cpp                       *
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
#include "TaskSpaceCoordinateTask.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;

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
CoordinateTask::~CoordinateTask() {}

//_____________________________________________________________________________
/**
 * Construct a task for a specified generalized coordinate.
 *
 * @param aQID ID of the generalized coordinate to be tracked.
 * @todo Instead of an integer id, the name of the coordinate
 * should be used.
 */
CoordinateTask::CoordinateTask(
        const std::string& aCoordinateName, bool limit_avoidance)
        : TaskSpaceTask() {
    setupProperties();
    setNull();
    set_coordinate(aCoordinateName);
    setName(aCoordinateName + "Task");
    set_limit_avoidance(limit_avoidance);
    
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Joint task to be copied.
 */
CoordinateTask::CoordinateTask(const CoordinateTask& aTask)
        : TaskSpaceTask(aTask) {
    setNull();
    copyData(aTask);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void CoordinateTask::setNull() {
    _nTrk = 1;
    _q = NULL;
    _p = SimTK::Vector(SimTK::Vector(1, 0.0));
    _v = SimTK::Vector(SimTK::Vector(1, 0.0));
    _inertialPTrk = SimTK::Vector(SimTK::Vector(1, 0.0));
    _inertialVTrk = SimTK::Vector(SimTK::Vector(1, 0.0));
    _pErrLast = SimTK::Vector(SimTK::Vector(1, 0.0));
    _pErr = SimTK::Vector(SimTK::Vector(1, 0.0));
    _vErrLast = SimTK::Vector(SimTK::Vector(1, 0.0));
    _vErr = SimTK::Vector(SimTK::Vector(1, 0.0));
    _aDes = SimTK::Vector(SimTK::Vector(1, 0.0));
    _a = SimTK::Vector(SimTK::Vector(1, 0.0));
}
//_____________________________________________________________________________
/**
 * @brief Set up serialized member variables.
 *
 * @param aCoordinateName Name of the coordinate to be tracked.
 */
void CoordinateTask::setupProperties() {
    //Construct properties unique to CoordinateTask
    constructProperty_limit_avoidance(false);
    constructProperty_coordinate("");
    
    //Set inherited properties
    setName("CoordTask");
    // Set default gains and weight
    set_kp(Array<double>(100, 1));
    set_kv(Array<double>(20, 1));
    set_ka(Array<double>(1, 1));
    set_weight(Array<double>(1, 1));
}

//_____________________________________________________________________________
/**
 * @brief Copy data members from one CoordinateTask to another.
 *
 * @param aTask CoordinateTask to be copied.
 */
void CoordinateTask::copyData(const CoordinateTask& aTask) 
{
    copyProperty_coordinate(aTask);
    copyProperty_limit_avoidance(aTask);

    updateWorkVariables();
}

//_____________________________________________________________________________
/**
 * @brief Update the work variables based on the current model.
 * This method is called by setModel() and setCoordinateName().
 *
 *
 */
void CoordinateTask::updateWorkVariables() {
    _q = NULL;
    if (hasModel()) {
        try {
            int nv = getModel().getWorkingState().getNU();
            _q = &getModel().getCoordinateSet().get(get_coordinate());
            //Set the Jacobian entry corresponding to this coordinate to 1.0
            int _mbti = getModel()
                                .getMatterSubsystem()
                                .getMobilizedBody(_q->getBodyIndex())
                                .getFirstUIndex(getModel().getWorkingState()) +
                        _q->getMobilizerQIndex();
            _j = SimTK::Matrix(1, nv, 0.0);
            _j(0, _mbti) = 1.0;
            _b = SimTK::Vector(1, 0.0);
        } catch (...) {
            OPENSIM_THROW(OpenSim::Exception,
                    "CoordinateTask.updateWorkVariables: ERROR- joint task references invalid coordinate");
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
CoordinateTask& CoordinateTask::operator=(const CoordinateTask& aTask) {
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
void CoordinateTask::computeErrors(const SimTK::State& s, double aT) {
    if (get_limit_avoidance() == true) {
        _pErr[0] =
                (0.5 * (_q->getRangeMax() - _q->getRangeMin()) -
                                       _q->getValue(s));
    } else {
        _pErr[0] = (get_position_functions(0).calcValue(SimTK::Vector(1, aT)) -
                           _q->getValue(s));
    }

    if (getProperty_velocity_functions().size() == 0) {
        std::vector<int> derivComponents(1);
        derivComponents[0] = 0;
        if (get_limit_avoidance() == true) {
            _vErr[0] = (0.0 - _q->getSpeedValue(s));
        } else {
            _vErr[0] = (get_position_functions(0).calcDerivative(
                                derivComponents, SimTK::Vector(1, aT)) -
                               _q->getSpeedValue(s));
        }
    } else {
        if (get_limit_avoidance() == true) {
            _vErr[0] = (0.0 - _q->getSpeedValue(s));
        } else {
            _vErr[0] = (get_velocity_functions(0).calcValue(
                                               SimTK::Vector(1, aT)) -
                                              _q->getSpeedValue(s));
        }
    }
}

//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aT Time at which the desired accelerations are to be computed in
 * real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void CoordinateTask::computeDesiredAccelerations(
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
void CoordinateTask::computeDesiredAccelerations(
        const SimTK::State& s, double aTI, double aTF) {
    double a;
    // CHECK
    if (!hasModel()) {
        log_warn("CoordinateTask model not set.");
        return;
    }
    if (getProperty_position_functions().size() == 0) {
        log_warn("CoordinateTask position tracking function not set.");
        return;
    }
    if (_q == NULL) {
        updateWorkVariables();
        return;
    }
    _a = SimTK::Vector(1, 0.0);
    _aDes = SimTK::Vector(1, 0.0);
    checkFunctions();

    // COMPUTE ERRORS
    computeErrors(s, aTI);

    // DESIRED ACCELERATION
    double p = get_kp(0) * _pErr[0];
    double v = get_kv(0) * _vErr[0];
    if (getProperty_acceleration_functions().size() == 0) {
        std::vector<int> derivComponents(2);
        derivComponents[0] = 0;
        derivComponents[1] = 0;
        a = get_ka(0) * get_position_functions(0).calcDerivative(
                                derivComponents, SimTK::Vector(1, aTF));
    } else {
        a = get_ka(0) *
            get_acceleration_functions(0).calcValue(SimTK::Vector(1, aTF));
    }
    _aDes[0] = get_weight(0) * (a + v + p);
}
//_____________________________________________________________________________
/**
 * Compute the acceleration of the appropriate generalized coordinate.
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
void CoordinateTask::computeAccelerations(const SimTK::State& s) {
    _a = SimTK::NaN;
    // CHECK
    if (!hasModel()) return;

    _a = SimTK::Vector(1, 1.0);
    // ACCELERATION
    _a[0] = _q->getAccelerationValue(s);
}

//_____________________________________________________________________________
/**
 * @brief Compute the Jacobian, since the task is already in coordinate space,
 * the jacobian is nu x 1 identity with 1 in the column of the specified
 * coordinate. When considering joint limit avoidance, the expression for J is
 * based on the potential field expression.
 * @param s current state
 * @return SimTK::Matrix Jacobian
 * @see CoordinateTask::computeJointLimitMatrix(const SimTK::State& s)
 *
 */
void CoordinateTask::computeJacobian(const SimTK::State& s) {
    SimTK::Matrix J(1, s.getNU(), 0.0);
    auto& matter = getModel().getMatterSubsystem();
    int _mbti = matter.getMobilizedBody(_q->getBodyIndex()).getFirstUIndex(s) +
                _q->getMobilizerQIndex();
    if (get_limit_avoidance() == true) { // use the potential field expression
                                         // if we care about limit avoidance.
        double max = _q->getRangeMax();
        double min = _q->getRangeMin();
        double buffer = (max - min) * .10;
        double minbuf = min + buffer;
        double maxbuf = max - buffer;
        double val = _q->getValue(s);
        if (val <= minbuf) {
            J(0, _mbti) = 1.0 * (1.0 - computeJointLimitMatrix(s));
        } else if (val >= maxbuf) {
            J(0, _mbti) = 1.0 * (1.0 - computeJointLimitMatrix(s));
        }
    } else {
        J(0, _mbti) = 1.0;
    }
    _j = J;
}

//_____________________________________________________________________________
/**
 * Compute the joint limit matrix, which is used in the potential field
 * expression for the Jacobian.
 *
 *  @see CoordinateTask::computeJointLimitMatrix(const SimTK::State& s)
 */
double CoordinateTask::computeJointLimitMatrix(const SimTK::State& s) {
    double JL = 0.0;
    double max, min, buffer, minbuf, maxbuf, val, deltaH = 0.0;
    max = _q->getRangeMax();
    min = _q->getRangeMin();
    buffer = (max - min) * .10;
    minbuf = min + buffer;
    maxbuf = max - buffer;
    val = _q->getValue(s);
    if (val <= min) {
        JL = 1.0;
    } else if ((val > min) && (val < minbuf)) {
        JL = 0.5 + 0.5 * sin((Pi / buffer) * (val - minbuf) + Pi / 2);
    } else if ((val <= maxbuf) && (val >= minbuf)) {
        JL = 0.0;
    } else if ((val > maxbuf) && (val < max)) {
        JL = 0.5 + 0.5 * sin((Pi / buffer) * (val - maxbuf) - Pi / 2);
    } else if (val >= max) {
        JL = 1.0;
    }
    return JL;
}

//_____________________________________________________________________________
/**
 * @brief Compute the Jacobian bias term for the tracked point given the current
 * state of the model.
 *
 * @param s current state
 * @return SimTK::Vector bias term
 * @see CoordinateTask::computeJointLimitBias(const SimTK::State& s)
 */
void CoordinateTask::computeBias(const SimTK::State& s) {
    if (_q == NULL) { updateWorkVariables(); }
    
    SimTK::SpatialVec jdu =
            getModel().getMatterSubsystem().calcBiasForFrameJacobian(s,
                    _q->getJoint().getParentFrame().getMobilizedBodyIndex(),
                    SimTK::Vec3(0));
    // Jdot*u is 0, because Jdot is zero. [0 ... 1 .. 0] * u = 0
    _b = SimTK::Vector(1,0.0 * jdu[0][_q->getMobilizerQIndex()]);
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
void CoordinateTask::updateFromXMLNode(
        SimTK::Xml::Element& aNode, int versionNumber) {
    Super::updateFromXMLNode(aNode, versionNumber);
}
