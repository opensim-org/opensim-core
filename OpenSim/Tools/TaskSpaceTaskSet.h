#ifndef OPENSIM_TASK_SPACE_TASK_SET_H_
#define OPENSIM_TASK_SPACE_TASK_SET_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim: TaskSpaceTaskSet.h                            *
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

#include "OpenSim/Simulation/Model/ModelComponentSet.h"
#include "TaskSpaceTask.h"

namespace OpenSim {
//=============================================================================
//=============================================================================
/**
 * A class for holding a set of tasks.
 */
class OSIMTOOLS_API TaskSpaceTaskSet :  public ModelComponentSet<TaskSpaceTask> {
OpenSim_DECLARE_CONCRETE_OBJECT(TaskSpaceTaskSet, ModelComponentSet<TaskSpaceTask>);

public:
    /** Use Super's constructors. @see ModelComponentSet */
    using Super::Super;

    // default copy, assignment operator, and destructor

//=============================================================================
};  // END of class TaskSpaceTaskSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_TASK_SPACE_TASK_SET_H_
