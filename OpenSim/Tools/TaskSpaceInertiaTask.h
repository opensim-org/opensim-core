#ifndef OPENSIM_TASK_SPACE_INERTIA_TASK_H_
#define OPENSIM_TASK_SPACE_INERTIA_TASK_H_
/* -------------------------------------------------------------------------- *
 *               OpenSim: TaskSpaceInertiaTask.h                              *
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
 * A class for tracking the orientation of a body.
 */
class OSIMTOOLS_API InertiaTask : public BodyTask {
    OpenSim_DECLARE_CONCRETE_OBJECT(InertiaTask, BodyTask);

    //=============================================================================
    // DATA
    //=============================================================================
protected:
    // Work Variables
    const Body *_wrtBody, *_expressBody;

    //=============================================================================
    // METHODS
    //=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    
    /**
     * Default constructor.
     */
    InertiaTask();

    /**
     * Copy constructor.
     */
    InertiaTask(const InertiaTask& aTask);

    /**
     * Destructor.
     */
    virtual ~InertiaTask();

private:
    /**
     * Set NULL values for all member variables.
     */

    void setNull();
    void setupProperties();

    /**
     * Update work variables
     */
    void updateWorkVariables(const SimTK::State& s);

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
public:
    /**
     * Compute the angular position and velocity errors.
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
     * Compute the desired angular accelerations.
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
     * Compute the angular acceleration of the specified frame.
     * For the computed accelerations to be correct,
     * Model::computeAccelerations() must have already been called.
     *
     * @see Model::computeAccelerations()
     * @see suTrackObject::getAcceleration()
     */
    void computeAccelerations(const SimTK::State& s) override;

    /**
     * @brief Compute the Jacobian.
     *
     * @param s SimTK::State
     */
    void computeJacobian(const SimTK::State& s) override;

    /**
     * @brief Compute the Jacobian bias
     *
     * @param s SimTK::State
     */
    void computeBias(const SimTK::State& s) override;
    // void update(const SimTK::State& s) override;
    // void setTaskFunctions(int i, Function* aF0);

//=============================================================================
}; // END of class InertiaTask
//=============================================================================
//=============================================================================
}; // namespace OpenSim

#endif // OPENSIM_TASK_SPACE_INERTIA_TASK_H_
