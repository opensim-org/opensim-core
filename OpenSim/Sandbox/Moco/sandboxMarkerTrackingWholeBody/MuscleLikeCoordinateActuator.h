#ifndef OPENSIM_MUSCLELIKECOORDINATEACTUATOR_H
#define OPENSIM_MUSCLELIKECOORDINATEACTUATOR_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MuscleLikeCoordinateActuator.h                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Carmichael Ong                                                  *
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

//#include "osimMocoDLL.h"
#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Common/osimCommon.h>

namespace OpenSim { 

/// This class derives from CoordinateActuator but has additional properties 
/// that mimic muscle-like properties according to the coordinate. The 
/// generalized force that is applied along the coordinate is calculated by a 
/// gain, an excitation (control signal), a function describing force vs 
/// coordinate, and a function describing a force vs the derivative of the 
/// coordinate:
///
///              F = (gain)*(excitation)*F_max(q)*F_vel(qdot)
///
/// The actuator expects two different function for the F_max curves, one for 
/// when excitation is positive and one for when excitation is negative. Each 
/// of the curves must be given as an Opensim::Function.
///
/// The F_vel curve is described with a single parameter, qdot_max, as 
/// described in Blake Ashby's thesis (2004). For positive excitations, 
/// the function returns:
///     0,                                             for qdot > qdot_max
///     1.5,                          for qdot < (-0.35678917232)*qdot_max
///     1 - (1.01750751592)*atan(1.5*qdot/qdot_max),             otherwise
/// For negative excitations, the inequalities flip and the sign for qdot_max 
/// flips.
///
/// @author Carmichael Ong
///
class MuscleLikeCoordinateActuator : public CoordinateActuator {
OpenSim_DECLARE_CONCRETE_OBJECT(MuscleLikeCoordinateActuator, 
        CoordinateActuator);
public:
    OpenSim_DECLARE_PROPERTY(qdot_max, double,
        "The velocity at which the actuator produces zero force.");
    OpenSim_DECLARE_PROPERTY(pos_force_vs_coordinate_function, Function,
        "Function that defines the optimal force curve vs. coordinate with "
        "positive activation.");
    OpenSim_DECLARE_PROPERTY(neg_force_vs_coordinate_function, Function,
        "Function that defines the optimal force curve vs. coordinate with "
        "negative activation.");

    /// Default constructor leaves coordinate name unspecified, or you can
    /// provide it. 
    MuscleLikeCoordinateActuator(const std::string& coordinateName="");

    /// Convience constructor.
    MuscleLikeCoordinateActuator(const std::string& coordinateName, 
            double optimalForce, 
            Function* posForceVsCoordinateFunction, 
            Function* negForceVsCoordinateFunction, 
            double qdot_max);

    /// Get and set methods.
    void setPosForceVsCoordinateFunction(
            Function* posForceVsCoordinateFunction);
    const Function* getPosForceVsCoordinateFunction();

    void setNegForceVsCoordinateFunction(
            Function* negForceVsCoordinateFunction);
    const Function* getNegForceVsCoordinateFunction();

    void setMaxVelocity(double qdot_max);
    double getMaxVelocity();

private:
    /// Compute all quantities necessary for applying the actuator force to the
    /// model.
    double computeActuation(const SimTK::State& s) const override;

    /// Helper functions to calculate F_max and F_vel for computeActuation.
    double getForceVsCoordinateFunctionValue(const SimTK::State& s) const;
    double getVelocityMultiplier(const SimTK::State& s) const;

    void setNull();
    void constructProperties();

}; // class MuscleLikeCoordinateActuator

} //namespace Opensim

#endif // OPENSIM_MUSCLELIKECOORDINATEACTUATOR_H


