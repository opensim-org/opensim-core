#ifndef OPENSIM_TASK_SPACE_COORDINATE_TASK_H_
#define OPENSIM_TASK_SPACE_COORDINATE_TASK_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim: TaskSpaceCoordinateTask.h                       *
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

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include "OpenSim/Simulation/osimSimulation.h"
#include "OpenSim/Common/PropertyDbl.h"
#include "TaskSpaceTask.h"

namespace OpenSim {

class Coordinate;

//=============================================================================
//=============================================================================
/**
 * A class for specifying the tracking task for a joint.
 */
class OSIMTOOLS_API CoordinateTask : public TaskSpaceTask {
    OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateTask, TaskSpaceTask);
//==========================================================================
// DATA
//==========================================================================
public:

    OpenSim_DECLARE_PROPERTY(coordinate, std::string,
            "Name of coordinate with respect to which a tracking objective is "
            "specified. ");

    OpenSim_DECLARE_PROPERTY(limit_avoidance, bool,
            "Boolean to indicate whether this coordinate should be used for "
            "limit avoidance (true) or coordinate space control (false).");

protected:
    // Work Variables
    const OpenSim::Coordinate* _q;
    mutable int _mbti;

    //==========================================================================
    // METHODS
    //==========================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    /**
     * Construct a task for a specified generalized coordinate.
     *
     * @param aQID ID of the generalized coordinate to be tracked.
     * @todo Instead of an integer id, the name of the coordinate
     * should be used.
     */
    CoordinateTask(const std::string& aCoordinateName = "",
            bool limit_avoidance = false);

    /**
     * Copy constructor.
     *
     * @param aTask Joint task to be copied.
     */
    CoordinateTask(const CoordinateTask& aTask);

    /**
     * Destructor.
     */
    virtual ~CoordinateTask();

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    /**
     * @brief Set the limit avoidance flag
     *
     * @param b flag to set
     */
    void setLimitAvoidance(const bool b) { set_limit_avoidance(b); }

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    /**
     * Compute the position and velocity errors.
     * This method assumes the states have been set for the model.
     *
     * @param aT Current time in real time units.
     * @see Model::set()
     * @see Model::setStates()
     */
    void computeErrors(const SimTK::State& s, double aT) override;

    /**
     * Compute the desired accelerations.
     * This method assumes that the states have been set for the model.
     *
     * @param aT Time at which the desired accelerations are to be computed in
     * real time units.
     * @see Model::set()
     * @see Model::setStates()
     */
    void computeDesiredAccelerations(const SimTK::State& s, double aT) override;

    /**
     * Compute the desired accelerations.
     * This method assumes that the states have been set for the model.
     *
     * @param aTI Initial time of the controlled interval in real time units.
     * @param aTF Final time of the controlled interval in real time units.
     * @see Model::set()
     * @see Model::setStates()
     */
    void computeDesiredAccelerations(
            const SimTK::State& s, double aTI, double aTF) override;
    
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
    void computeAccelerations(const SimTK::State& s) override;

    /**
     * @brief Compute the Jacobian, since the task is already in coordinate
     * space, the jacobian is nu x 1 identity with 1 in the column of the
     * specified coordinate. When considering joint limit avoidance, the
     * expression for J is based on the potential field expression.
     * @param s current state
     * @return SimTK::Matrix Jacobian
     * @see CoordinateTask::computeJointLimitMatrix(const SimTK::State& s)
     *
     */
    void computeJacobian(const SimTK::State& s) override;

    /**
     * @brief Compute the Jacobian bias term for the tracked point given the
     * current state of the model.
     *
     * @param s current state
     * @return SimTK::Vector bias term
     * @see CoordinateTask::computeJointLimitBias(const SimTK::State& s)
     */
    void computeBias(const SimTK::State& s) override;

    /**
     * @brief Update internal storage of task dynamics.
     *
     * @param s current state
     */
    // void update(const SimTK::State& s) override;

    /**
     * Compute the joint limit matrix, which is used in the potential field
     * expression for the Jacobian.
     *
     *  @see CoordinateTask::computeJointLimitMatrix(const SimTK::State& s)
     */
    double computeJointLimitMatrix(const SimTK::State& s);
            
    //--------------------------------------------------------------------------
    // XML
    //--------------------------------------------------------------------------
    /**
     * Update this object based on its XML node.
     *
     * @param aDeep If true, update this object and all its child objects
     * (that is, member variables that are Object's); if false, update only
     * the member variables that are not Object's.
     */
    void updateFromXMLNode(
            SimTK::Xml::Element& aNode, int versionNumber = -1) override;
#ifndef SWIG
    /**
     * Assignment operator.
     *
     * @param aTask Object to be copied.
     * @return  Reference to the altered object.
     */
    CoordinateTask& operator=(const CoordinateTask& aTask);
#endif

private:
    /**
     * Set NULL values for all member variables.
     */
    void setNull();
    
    /**
     * @brief Set up serialized member variables.
     *
     * @param aCoordinateName Name of the coordinate to be tracked.
     */
    void setupProperties();

    /**
     * @brief Copy data members from one CoordinateTask to another.
     *
     * @param aTask CoordinateTask to be copied.
     */
    void copyData(const CoordinateTask& aTask);

    /**
     * @brief Update the work variables based on the current model.
     * This method is called by setModel() and setCoordinateName().
     */
    void updateWorkVariables();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------

//==========================================================================
}; // END of class CoordinateTask
//==========================================================================
}; // namespace OpenSim

#endif // OPENSIM_TASK_SPACE_COORDINATE_TASK_H_
