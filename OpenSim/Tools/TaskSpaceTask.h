#ifndef OPENSIM_TASK_SPACE_TASK_H_
#define OPENSIM_TASK_SPACE_TASK_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: TaskSpaceTask.h                              *
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

#include "osimToolsDLL.h"

#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Common/Function.h"

namespace OpenSim {

/**
 * \brief TaskSpace analog for CMC_Task or TrackingTask
 */
class OSIMTOOLS_API TaskSpaceTask : public ModelComponent {
    OpenSim_DECLARE_ABSTRACT_OBJECT(TaskSpaceTask, ModelComponent);

public:
    //=============================================================================
    // PROPERTIES
    //=============================================================================
    // PROPERTIES
    OpenSim_DECLARE_PROPERTY(
            priority, int, "Priority level of the task. 0 is highest.");

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(position_functions, OpenSim::Function,
            3, "Position task functions.  Different types of tasks can "
            "require different numbers of task functions.  For example, to track"
            "a joint angle, only one task function is needed.  However, to track"
            "a position, up to three task functions may be needed. If tracking "
            "experimental data, do not specify these functions - splines will be "
            "generated from the data.");

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(velocity_functions, OpenSim::Function,
            3, "Velocity task functions.  If velocity task functions are not "
            "specified, derivatives of the position task function are used. If "
            "tracking experimental data, do not specify these functions - splines "
            "will be generated from the data.");

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(acceleration_functions,
            OpenSim::Function, 3,
            "Velocity task functions.  If acceleration task functions are not "
            "specified, derivatives of the position task function are used. If "
            "tracking experimental data, do not specify these functions - splines "
            "will be generated from the data.");

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(weight, double, 3, "Weight with which a "
         "task is tracked relative to other tasks. To track a task more "
         "tightly, make the weight larger.");
    
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(kp, double, 3,
            "Position error feedback gain (stiffness). "
            "To achieve critical damping of errors, choose kv = 2*sqrt(kp).");

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(kv, double, 3,
            "Velocity error feedback gain (damping). "
            "To achieve critical damping of errors, choose kv = 2*sqrt(kp).");

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(ka, double, 3,
            "Feedforward acceleration gain.  "
            "This is normally set to 1.0, so no gain.");

    //=============================================================================
    // PUBLIC METHODS
    //=============================================================================

    TaskSpaceTask();
    virtual ~TaskSpaceTask() {}
#ifndef SWIG
    TaskSpaceTask& operator=(const TaskSpaceTask& aTaskObject);
#endif
    virtual void update(const SimTK::State& s);
    virtual void computeErrors(const SimTK::State& s, double aT) = 0;
    virtual void computeDesiredAccelerations(
            const SimTK::State& s, double aT) = 0;
    virtual void computeDesiredAccelerations(
            const SimTK::State& s, double aTI, double aTF) = 0;
    virtual void computeAccelerations(const SimTK::State& s) = 0;
    virtual void computeJacobian(const SimTK::State& s) = 0;
    virtual void computeBias(const SimTK::State& s) = 0;

    /**
     * Get the task position.
     *
     * @param aWhich Specifies which task goal (0, 1, or 2).
     * @param aT Time (in real time units).
     * @return Task position.
     * @throws Exception for an invalid task.
     */
    double getTaskPosition(int aWhich, double aT) const;

    /**
     * Get the task velocity.
     *
     * @param aWhich Specifies which task goal (0, 1, or 2).
     * @param aT Time (in real time units).
     * @return Task velocity.
     * @throws Exception for an invalid task.
     */
    double getTaskVelocity(int aWhich, double aT) const;

    /**
     * Get the task acceleration.
     *
     * @param aWhich Specifies which task goal (0, 1, or 2).
     * @param aT Time (in real time units).
     * @return Task acceleration.
     * @throws Exception for an invalid task.
     */
    double getTaskAcceleration(int aWhich, double aT) const;
    
    /**
     * Set the last achieved position error.  This information is useful for
     * checking that error dynamics are being followed.
     *
     * @param aE0 Last position error for track goal 0.
     * @param aE1 Last position error for track goal 1.
     * @param aE2 Last position error for track goal 2.
     */
    void setPositionErrorLast(double aE0, double aE1 = 0.0, double aE2 = 0.0);

    /**
     * Get the last achieved position error for the track goal at a given index.
     * This information is useful for checking that error dynamics are being
     * followed.
     *
     * @param aWhich Specifies which track goal (0, 1, or 2).
     * @return Last position error.
     */
    double getPositionErrorLast(int aWhich) const;
    void setVelocityErrorLast(double aE0, double aE1 = 0.0, double aE2 = 0.0);
    double getVelocityErrorLast(int aWhich) const;
    // ERRORS
    double getPositionError(int aWhich) const;
    double getVelocityError(int aWhich) const;
    // DESIRED ACCELERATIONS
    double getDesiredAcceleration(int aWhich) const;
    SimTK::Vector getDesiredAccelerations() const;
    // ACCELERATIONS
    double getAcceleration(int aWhich) const;
    // JACOBIAN
    SimTK::Matrix getJacobian() const;
    SimTK::Vector getBias() const;

protected:
    // Model component interface.
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;

    // const Model* _model;
    int _nTrk;
    void checkFunctions() const;

    /** Last position error. */
    SimTK::Vector _pErrLast;
    /** Position error. */
    SimTK::Vector _pErr;
    /** Last velocity error. */
    SimTK::Vector _vErrLast;
    /** Velocity error. */
    SimTK::Vector _vErr;
    /** Desired accelerations. */
    SimTK::Vector _aDes;
    /** Positions. */
    SimTK::Vector _p, _inertialPTrk;
    /** Velocities. */
    SimTK::Vector _v, _inertialVTrk;
    /** Accelerations. */
    SimTK::Vector _a;
    /** Jacobian. */
    SimTK::Matrix _j;
    /** Jacobian bias. */
    SimTK::Vector _b;

private:
    void constructProperties();
    void setNull();
    void copyData(const TaskSpaceTask& aTaskObject);

//=============================================================================
}; // end of class TaskSpaceTask
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_TASK_SPACE_TASK_H_