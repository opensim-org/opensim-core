#ifndef _OPENSIM_COMAK_SETTINGS_H_
#define _OPENSIM_COMAK_SETTINGS_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  COMAKSettings.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// INCLUDE
#include "osimJAMDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/FunctionSet.h>
//=============================================================================
//=============================================================================
namespace OpenSim {


class OSIMJAM_API COMAKSecondaryCoordinate : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKSecondaryCoordinate, Object)

public:
    OpenSim_DECLARE_PROPERTY(coordinate, std::string, 
        "Path to Coordinate in model.")

    /*OpenSim_DECLARE_PROPERTY(comak_damping, double,
        "Coefficient to penalize frame-to-frame changes in predicted "
        "secondary coordinate values. "
        "The default value is 1.0.")*/

    OpenSim_DECLARE_PROPERTY(max_change, double, "Limit on the maximum "
        "frame-to-frame changes in secondary coordinate values. "
        "The default value is 0.05.")

    COMAKSecondaryCoordinate();
    void constructProperties();
}; //END of class COMAKSecondaryCoordinate

class OSIMJAM_API COMAKCostFunctionParameter : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(COMAKCostFunctionParameter, Object)

public:
    OpenSim_DECLARE_PROPERTY(actuator, std::string, 
        "Path to actuator in model.")

    OpenSim_DECLARE_PROPERTY(weight, Function, 
        "Time varying weighting coefficient that multiplies the entire "
        "actuator activation term in the COMAK optimization cost function."
        "W in the cost function: W*(a-d)^n. "
        "The Default value is Constant(1.0).")

    OpenSim_DECLARE_PROPERTY(desired_activation, Function, 
        "Time varying term that is subtracted from the actuator activation "
        "within the term rasied to the activation_exponent in the COMAK "
        "optimization cost function."
        "d in the cost function: W*(a-d)^n. "
        "The Default value is Constant(0.0).")
    
    OpenSim_DECLARE_PROPERTY(activation_lower_bound, Function, 
        "Time varying lower bound on the actuator activation in the "
        "COMAK optimization constraints."
        "The Default value is Constant(0.0).")

    OpenSim_DECLARE_PROPERTY(activation_upper_bound, Function, 
        "Time varying lower bound on the actuator activation in the "
        "COMAK optimization constraints."
        "The Default value is Constant(1.0).")

    COMAKCostFunctionParameter();
    void constructProperties();
}; //END of class COMAKCostFunctionParameter

}; //namespace
//=============================================================================
//=============================================================================

#endif // _OPENSIM_COMAK_SETTINGS_H_


