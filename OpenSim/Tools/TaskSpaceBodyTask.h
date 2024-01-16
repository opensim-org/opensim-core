#ifndef OPENSIM_TASK_SPACE_BODY_TASK_H_
#define OPENSIM_TASK_SPACE_BODY_TASK_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim: BodyTask.h                              *
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

#include "osimToolsDLL.h"
#include "TaskSpaceTask.h"

namespace OpenSim {

/**
 * \brief Tracking task associated with a body (e.g., position or orientation).
 */
class OSIMTOOLS_API BodyTask : public TaskSpaceTask {
    OpenSim_DECLARE_ABSTRACT_OBJECT(BodyTask, TaskSpaceTask);

public:
    //=============================================================================
    // PROPERTIES
    //=============================================================================
    OpenSim_DECLARE_PROPERTY(wrt_body, std::string,
            "Name of body frame with respect to which a tracking objective is "
            "specified. "
            "The special name 'center_of_mass' refers to the system center of "
            "mass. "
            "This property is not used for tracking joint angles.");

    OpenSim_DECLARE_PROPERTY(express_body, std::string,
            "Name of body frame in which the tracking "
            "objectives are expressed.  This property is not used for tracking "
            "joint angles.");

    OpenSim_DECLARE_LIST_PROPERTY(direction_vectors, SimTK::Vec3,
            "Direction vector[3] for each component of a task. "
            "Joint tasks do not use this property.");

    //=============================================================================
    // PUBLIC METHODS
    //=============================================================================

    BodyTask();
    virtual ~BodyTask() {}
#ifndef SWIG
    BodyTask& operator=(const BodyTask& aTaskObject);
#endif
    
    // virtual void computeErrors(const SimTK::State& s, double aT) = 0;
    // virtual void computeDesiredAccelerations(
    //         const SimTK::State& s, double aT) = 0;
    // virtual void computeDesiredAccelerations(
    //         const SimTK::State& s, double aTI, double aTF) = 0;
    // virtual void computeAccelerations(const SimTK::State& s) = 0;
    // virtual void computeJacobian(const SimTK::State& s);
    // virtual void computeBias(const SimTK::State& s);
    // virtual void update(const SimTK::State& s);

protected:
    // Model component interface.
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;

private:
    void constructProperties();
    void setNull();
    void copyData(const BodyTask& aTaskObject);

//=============================================================================
}; // end of class TaskSpaceBodyTask
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_TASK_SPACE_BODY_TASK_H_