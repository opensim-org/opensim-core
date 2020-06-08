#ifndef OPENSIM_IKTASK_H_
#define OPENSIM_IKTASK_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  IKTask.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @author Eran Guendelman, Ayman Habib
 */

class OSIMTOOLS_API IKTask : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(IKTask, Object);

protected:
    OpenSim_DECLARE_PROPERTY(apply, bool,
        "Whether or not this task will be used during inverse kinematics solve, default is true.");

    OpenSim_DECLARE_PROPERTY(weight, double,
            "Weight given to the task when solving inverse kinematics problems, default is 0.");
public:
    IKTask() { 
        constructProperties();
    }

    bool getApply() const { return get_apply(); }
    void setApply(bool aApply) { upd_apply() = aApply; }

    double getWeight() { return get_weight(); }
    void setWeight(double weight) { upd_weight() = weight; }

private:
    void constructProperties() { 
        constructProperty_apply(true);
        constructProperty_weight(0.0);
    }
    //=============================================================================
};  // END of class IKTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_IKTASK_H_
