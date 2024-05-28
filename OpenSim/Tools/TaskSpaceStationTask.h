#ifndef OPENSIM_TASK_SPACE_STATION_TASK_H_
#define OPENSIM_TASK_SPACE_STATION_TASK_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim: TaskSpaceStationTask.h                           *
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

//============================================================================
// INCLUDE
//============================================================================
#include "osimToolsDLL.h"
#include "OpenSim/Simulation/osimSimulation.h"
#include "OpenSim/Common/PropertyDbl.h"
#include "TaskSpaceBodyTask.h"

namespace OpenSim {

class Body;

//=============================================================================
//=============================================================================
/**
 * A class for specifying and computing parameters for tracking a point.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API StationTask : public BodyTask {
    OpenSim_DECLARE_CONCRETE_OBJECT(StationTask, BodyTask);

public:
    OpenSim_DECLARE_PROPERTY(point, SimTK::Vec3,
        "Point in body frame with respect to which an objective is tracked.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(left_foot, std::string,
        "Name of body representing left foot. Used to calculate base of support.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(right_foot, std::string,
        "Name of body representing right foot. Used to calculate base of support.");
    
    //=============================================================================
    // DATA
    //=============================================================================
protected:
    // REFERENCES
    const Body *_wrtBody, *_expressBody;
    
    //=============================================================================
    // METHODS
    //=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /**
     * Construct a task for a specified point.
     *
     */
    StationTask();

    /**
     * Copy constructor.
     *
     * @param aTask Point task to be copied.
     */
    StationTask(const StationTask& aTask);

    /**
     * Destructor.
     */
    virtual ~StationTask();

private:
    /**
     * Set NULL values for all member variables.
     */
    void setNull();

    /**
     * @brief Construct properties for the task.
     *
     */
    void constructProperties();

    /**
     * Copy only the member data of specified object.
     */
    void copyData(const StationTask& aTask);

    /**
     * Update work variables
     */
    void updateWorkVariables(const SimTK::State& s);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    /**
     * Assignment operator.
     *
     * @param aTask Object to be copied.
     * @return  Reference to the altered object.
     */
    StationTask& operator=(const StationTask& aTask);
#endif

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
    void computeAccelerations(const SimTK::State& s) override;

    /**
     * Compute the Jacobian for the tracked point given the current state
     * of the model.
     */
    void computeJacobian(const SimTK::State& s) override;

    /**
     * Compute the Jacobian bias term for the tracked point given the current state
     * of the model.
     */
    void computeBias(const SimTK::State& s) override;
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

//=============================================================================
}; // END of class StationTask
//=============================================================================
//=============================================================================
}; // namespace OpenSim

#endif // OPENSIM_TASK_SPACE_STATION_TASK_H_
