#ifndef OPENSIM_FORCE_CONSUMER_H_
#define OPENSIM_FORCE_CONSUMER_H_

/* -------------------------------------------------------------------------- *
*                         OpenSim: ForceConsumer.h                           *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2024 Stanford University and the Authors                *
* Author(s): Adam Kewley                                                     *
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

#include <SimTKcommon/SmallMatrix.h>  // `SimTK::Vec3`

namespace OpenSim { class Coordinate; }
namespace OpenSim { class PhysicalFrame; }
namespace SimTK { class State; }

namespace OpenSim
{

/**
* A `ForceConsumer` is an abstract class that can consume forces via its `consume*`
* methods. It is typically used in conjunction with `ForceProducer`s, which take
* a mutable reference to a `ForceConsumer` and emit their forces into it.
*
* The `ForceConsumer` API does not dictate how concrete implementations should
* handle the forces. This is to support (e.g.) implementations that actually
* apply the forces (as in `ForceProducer::computeForce`), or implementations that
* print force debugging information, or UIs that want to display the forces as 3D
* decorations.
*/
class ForceConsumer {
protected:
    ForceConsumer() = default;
    ForceConsumer(const ForceConsumer&) = default;
    ForceConsumer(ForceConsumer&&) noexcept = default;
    ForceConsumer& operator=(const ForceConsumer&) = default;
    ForceConsumer& operator=(ForceConsumer&&) noexcept = default;

public:
    virtual ~ForceConsumer() noexcept = default;

    /**
     * Consumes a generalized force.
     *
     * @param state    the `SimTK::State` to which the force applies
     * @param coord    the generalized coordinate to which the force applies
     * @param force    the (scalar) force change in the generalized coordinate
     */
    void consumeGeneralizedForce(
        const SimTK::State& state,
        const Coordinate& coord,
        double force
    );

    /**
     * Consumes a body force.
     *
     * Note: prefer using `consumePointForce` if the calling code has point-force
     *       information. This is because it enables downstream code to introspect,
     *       visualize, or debug point forces, which might be more useful than
     *       visualizing fully-resolved body forces.
     *
     * @param state     the `SimTK::State` to which the force applies
     * @param body      the body to which the force applies
     * @param force     the force, expressed in the body frame
     */
    void consumeBodyForce(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::Vec3& force
    );

    /**
     * Consumes a body torque.
     *
     * @param state     the `SimTK::State` to which the torque applies
     * @param body      the body to which the torque applies
     * @param torque    the torque, specified in the inertial frame, to apply
     */
    void consumeBodyTorque(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::Vec3& torque
    );

    /**
     * Consumes a point force. That is, a force applied at a point within a frame.
     *
     * @param state    the `SimTK::State` to which the force applies
     * @param frame    the frame in which `point` is defined
     * @param point    a point in `frame` where `force` applies
     * @param force    a force that should be applied to `point`
     */
    void consumePointForce(
        const SimTK::State& state,
        const PhysicalFrame& frame,
        const SimTK::Vec3& point,
        const SimTK::Vec3& force
    );

private:
    /**
     * Subclasses of `ForceConsumer` may implement this method. There are no expectations
     * on how an implementation should handle a call to this function.
     */
    virtual void implConsumeGeneralizedForce(
        const SimTK::State& state,
        const Coordinate& coord,
        double force)
    {}

    /**
    * Subclasses of `ForceConsumer` may implement this method. There are no expectations
    * on how an implementation should handle a call to this function.
    */
    virtual void implConsumeBodyForce(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::Vec3& force)
    {}

    /**
     * Subclasses of `ForceConsumer` may implement this method. There are no expectations
     * on how an implementation should handle a call to this function.
     */
    virtual void implConsumeBodyTorque(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::Vec3& torque)
    {}

    /**
     * Subclasses of `ForceConsumer` may implement this method. There are no expectations
     * on how an implementation should handle a call to this function.
     */
    virtual void implConsumePointForce(
        const SimTK::State& state,
        const PhysicalFrame& frame,
        const SimTK::Vec3& point,
        const SimTK::Vec3& force)
    {}
};

}  // namespace OpenSim

#endif  // OPENSIM_FORCE_CONSUMER_H_
