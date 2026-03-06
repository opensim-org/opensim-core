#ifndef OPENSIM_COORDINATE_LINEAR_STOP_FORCE_H
#define OPENSIM_COORDINATE_LINEAR_STOP_FORCE_H
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  CoordinateLinearStopForce.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2026 Stanford University and the Authors                     *
 * Author(s): Nicholas Bianco                                                 *
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

#include "Force.h"

#include "simbody/internal/Force_MobilityLinearStop.h"

namespace OpenSim {

/**
 * Generate a compliant stop force that acts to keep a coordinate within
 * specified bounds. This force element generates no force when the coordinate
 * is in bounds, but when either the lower or upper bound is exceeded
 * it generates a generalized force opposing further violation of the bound. The
 * generated force is composed of a stiffness force (or torque, or generalized
 * force) that is linear in the violation of the bound, and damping that is
 * linear in the rate qdot. Internally, this class encapsulates the Simbody
 * force element SimTK::MobilityLinearStop.
 *
 * @note CoordinateLinearStopForce currently works only for coordinates q whose
 * time derivatives qdot are just the corresponding generalized speed u. That is
 * the case for translational mobilities, pin, universal, and gimbal mobilizers
 * but not free or ball mobilizers.
 *
 * \section Theory
 *
 * Given a coordinate q and limits q_lower and q_upper, the generalized
 * force generated here is:
 * <pre>
 *        ⎧           0,               q_low ≤ q ≤ q_up
 *   f =  ⎨ min(0, -k·x·(1 + d·qdot)), q > q_up,  x = q - q_up  (x > 0, f ≤ 0)
 *        ⎩ max(0, -k·x·(1 - d·qdot)), q < q_low, x = q - q_low (x < 0, f ≥ 0)
 * </pre>
 * where k is a stiffness parameter and d is a damping coefficient.
 *
 * Damping occurs both during compression and expansion, but the max() and
 * min() functions in the above equations prevent "sticking" behavior, i.e.,
 * damping will never generate a force that opposes motion back toward
 * the limit. Unlike CoordinateLimitForce and ExponentialCoordinateLimitForce,
 * this force does not have a smooth transition from zero to the applied force
 * as the coordinate exceeds its limit. Finally, it  uses a Hunt and
 * Crossley-like damping model where the damping force is zero when you first
 * touch the stop.
 */
class OSIMSIMULATION_API CoordinateLinearStopForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateLinearStopForce, Force);
public:
    OpenSim_DECLARE_PROPERTY(coordinate, std::string,
        "The name or full path of the coordinate to which this stop force is "
        "applied.");
    OpenSim_DECLARE_PROPERTY(stiffness, double,
        "The stiffness coefficient of the stop force (N-m/rad for rotational, "
        "N/m for translational).");
    OpenSim_DECLARE_PROPERTY(damping, double,
        "The damping coefficient of the stop force (N-m/(rad/s) for "
        "rotational, N/(m/s) for translational).");
    OpenSim_DECLARE_PROPERTY(upper_limit, double,
        "Upper coordinate limit (rad for rotational, m for translational).");
    OpenSim_DECLARE_PROPERTY(lower_limit, double,
        "Lower coordinate limit (rad for rotational, m for translational).");

    /**
     * Default constructor.
     */
    CoordinateLinearStopForce();

    /**
     * Convenience constructor.
     *
     * @param[in] coordinateNameOrPath The name or full path of the coordinate
     * to which the limit forces are applied.
     * @param[in] stiffness The stiffness coefficient of the stop force
     * (N-m/rad for rotational, N/m for translational).
     * @param[in] damping The damping coefficient of the stop force
     * (N-m/(rad/s) for rotational, N/(m/s) for translational).
     * @param[in] lowerLimit The lower limit of the coordinate range of motion
     * (rad for rotational, m for translational).
     * @param[in] upperLimit The upper limit of the coordinate range of motion
     * (rad for rotational, m for translational).
     */
    CoordinateLinearStopForce(const std::string& coordinateName,
        double stiffness, double damping, double lowerLimit, double upperLimit);

    // FORCE INTERFACE
    double computePotentialEnergy(const SimTK::State& state) const override;

    // REPORTING
    Array<std::string> getRecordLabels() const override;
    Array<double> getRecordValues(const SimTK::State& state) const override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    void constructProperties();

    // HELPERS
    const Coordinate& getCoordinate() const;
    const SimTK::Force::MobilityLinearStop& getMobilityLinearStop() const;

}; // class CoordinateLinearStopForce

} // namespace OpenSim

#endif // OPENSIM_COORDINATE_LINEAR_STOP_FORCE_H