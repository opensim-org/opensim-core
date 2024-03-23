#ifndef OPENSIM_TASK_SPACE_TORQUE_CONTROLLER_H_
#define OPENSIM_TASK_SPACE_TORQUE_CONTROLLER_H_
/* -------------------------------------------------------------------------- *
 *                 OpenSim: TaskSpaceTorqueController.h                       *
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
 *            and Dimitar Stanev                                              *
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

#include "OpenSim/Simulation/Control/Controller.h"

#include "TaskSpaceTaskSet.h"
#include "TaskSpaceConstraintModel.h"

namespace OpenSim {
/**
 * \brief Computes and applies the generalized forces that track the task
 * goals provided a function for evaluating the control strategy.
 *
 * As this is a generalized force controller that evaluates the control
 * strategy and applies the forces to the model. As this is a Controller
 * and not a Force, CoordinateActuators are appended to the model and
 * actuated according to the control strategy.
 */

class ConstraintProjection;

class OSIMTOOLS_API TaskSpaceTorqueController : public Controller {
    OpenSim_DECLARE_CONCRETE_OBJECT(TaskSpaceTorqueController, Controller);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_UNNAMED_PROPERTY(TaskSpaceTaskSet, 
            "Set of tracking goals in task space.");

    OpenSim_DECLARE_UNNAMED_PROPERTY(ConstraintModel,
            "Name of the constraint model to use.");

    OpenSim_DECLARE_LIST_PROPERTY(excluded_coords, std::string,
            "List of coordinates that are not actuated.");

    OpenSim_DECLARE_PROPERTY(control_strategy, std::string,
            "Type of feedback control strategy to implement.");

    OpenSim_DECLARE_PROPERTY(use_constraint_nullspace, bool,
            "If true, constraints are given priority -1.");

//=============================================================================
// METHODS
//=============================================================================
public:
    //CONSTRUCTION
    TaskSpaceTorqueController();
    TaskSpaceTorqueController(const TaskSpaceTorqueController& aController);

    /**
     * Prints the applied forces to a .sto file format.
     *
     * @param aPrefix is added to the file (useful for batch simulations).
     * @param aDir is the directory where the results will be stored.
     */
    void printResults(const std::string &aPrefix, const std::string &aDir);

    /** Controller. */
    void computeControls(
            const SimTK::State& s, SimTK::Vector& controls) const override;

    SimTK::Vector _tau;

    void setDT(double aDT);
    double getDT() const;
    void setTargetTime(double aTime);
    double getTargetTime() const;
    void setTargetDT(double aDT);
    double getTargetDT() const;
    void setCheckTargetTime(bool aTrueFalse);
    bool getCheckTargetTime() const;

protected:

    /** Compute the generalized forces tau needed to control the model. */
    SimTK::Vector calcTau(const SimTK::State& s) const;
    
    void extendFinalizeFromProperties() override;

    //TODO: not needed?
    // for adding any components to the underlying system
    void extendAddToSystem( SimTK::MultibodySystem& system) const override;

    void extendConnectToModel(Model& model) override;

    /** Selection matrix \f$ \tau^{'} = B \tau \f$ for under-actuation. */
    mutable SimTK::Matrix B;

    /* Current priority level being evaluated. */
    int max_priority = 0;

    /* Number of generalized coordinates. */
    int n = 0;

    /* Selected constraint projection method. */
    //ConstraintProjection* m_constraint_projection;

    /* Priority-sorted augmented Jacobian matrix. */
    std::vector<SimTK::Matrix> Jt_p;

    // Priority-sorted desired accelerations
    std::vector<SimTK::Vector> xtddot_p; 

    // Priority-sorted task bias
    std::vector<SimTK::Vector> bt_p; 

    /** Stores the applied generalized forces. */
    mutable Storage appliedForces;

    /** Next integration step size that is to be taken by the integrator. */
    double _dt;
    /** Last integration step size that was taken before a new integration
    step size was set in order to step exactly to the target time. */
    double _lastDT;
    /** Flag indicating when the last integration step size should be restored.
    Normally it is restored only following a change to the integration
    step size that was made to step exactly to the end of a target interval. */
    bool _restoreDT;
    /** The target time is the time in the future (in normalized units) for
    which the controls have been calculated.  If an integrator is taking
    steps prior to the target time, the controls should not have to be
    computed again. */
    double _tf;

    /** The step size used to generate a new target time, once the
    old target time has been reached. */
    double _targetDT;

    /** Whether or not to check the target time. */
    bool _checkTargetTime;

private:
    
    void constructProperties();
    void setNull();
    void copyData(const TaskSpaceTorqueController& aController);

    /**
     * @brief Create the selection matrix \f$ \tau^{'} = S \tau \f$ for
     * under-actuation. Generates the matrix from the acuated_dofs property.
     * This method is called automatically when the model is set using
     * setModel(), but should be called again if the actuated_dofs are altered.
     * Must be called during initialization.
     *
     * @param model
     *
     * \throws Exception if user-specified actuated DoFs do not match number of
     * DoFs in model.
     */
    SimTK::Matrix createSelectionMatrix() const;

//=============================================================================
}; // END of class TaskSpaceTorqueController
//=============================================================================
//=============================================================================
} // namespace OpenSim

#endif
