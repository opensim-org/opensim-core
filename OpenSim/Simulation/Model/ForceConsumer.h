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

#include <SimTKcommon/SmallMatrix.h>              // for `SimTK::Vec3`
#include <SimTKcommon/internal/MassProperties.h>  // for `SimTK::SpatialVec`

namespace OpenSim { class Coordinate; }
namespace OpenSim { class PhysicalFrame; }
namespace SimTK { class State; }

namespace OpenSim
{

/**
* A `ForceConsumer` is an abstract class that can consume forces via its `consume*`
* methods. It is typically used in conjunction with `ForceProducer`s, which produce
* the forces that this API consumes.
*
* The `ForceConsumer` API does not dictate how concrete implementations should
* handle the forces. This is to support several use-cases (examples):
*
* - Implementations that actually apply the forces (see: `ForceProducer::computeForce`, `ForceApplier`),
* - Implementations that print force debugging information
* - Implementations that want to render force vectors in 3D (e.g. UIs)
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
     * This is the `ForceConsumer`'s dual to `OpenSim::Force::applyGeneralizedForce`
     *
     * @param state    the state that was used to evaluate the force
     * @param coord    the generalized coordinate to which the force applies
     * @param force    the (scalar) force change in the generalized coordinate
     */
    void consumeGeneralizedForce(
        const SimTK::State& state,
        const Coordinate& coord,
        double force)
    {
        implConsumeGeneralizedForce(state, coord, force);
    }

    /**
     * Consumes a body torque (index 0) and force (index 1) as a `SimTK::SpatialVec`
     *
     * If callers to this function are handling point-force-like data, then they should
     * prefer calling `consumePointForce` and `consumeTorque` seperately instead of
     * calling this. This so that concrete `ForceConsumer` implementations are able
     * to introspect the point forces, rather than only receiving fully-resolved body
     * forces.
     *
     * @param state         the state that was used to evaluate the force
     * @param body          the body (frame) to which the force applies (at its center)
     * @param spatialVec    a `SimTK::SpatialVector` that contains the torque at index 0 and force at index 1
     */
    void consumeBodySpatialVec(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::SpatialVec& spatialVec)
    {
        implConsumeBodySpatialVec(state, body, spatialVec);
    }

    /**
     * Consumes a body torque.
     *
     * This is the `ForceConsumer`'s dual to `OpenSim::Force::applyTorque`.
     *
     * If a caller to this function has both a body torque and a body (not point) force
     * available, then it should prefer calling `consumeBodySpatialVec`.
     *
     * @param state     the state that was used to evaluate the torque
     * @param body      the body to which the torque applies
     * @param torque    the torque vector, specified in the inertial frame
     */
    void consumeTorque(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::Vec3& torque)
    {
        consumeBodySpatialVec(state, body, SimTK::SpatialVec{torque, SimTK::Vec3{0.0, 0.0, 0.0}});
    }

    /**
     * Consumes a point force. That is, a force applied at a point (a "station") within
     * a frame.
     *
     * This is the `ForceConsumer`'s dual to `OpenSim::Force::applyForceToPoint`
     *
     * @param state    the state that was used to evaluate the force
     * @param frame    the frame in which `point` is defined
     * @param point    a point in `frame` where `force` applies
     * @param force    the force vector, specified in the inertial (ground) frame
     */
    void consumePointForce(
        const SimTK::State& state,
        const PhysicalFrame& frame,
        const SimTK::Vec3& point,
        const SimTK::Vec3& force)
    {
        implConsumePointForce(state, frame, point, force);
    }

private:
    /**
     * Subclasses of `ForceConsumer` may implement this method. There are no expectations
     * on how an implementation should handle a call to this function.
     *
     * @param state    the state that was used to evaluate the force
     * @param coord    the generalized coordinate to which the force applies
     * @param force    the (scalar) force change in the generalized coordinate
     */
    virtual void implConsumeGeneralizedForce(
        const SimTK::State& state,
        const Coordinate& coord,
        double force)
    {}

    /**
    * Subclasses of `ForceConsumer` may implement this method. There are no expectations
    * on how an implementation should handle a call to this function.
    *
    * @param state         the state that was used to evaluate the force
    * @param body          the body (frame) to which the force applies (at its center)
    * @param spatialVec    a `SimTK::SpatialVector` that contains the torque at index 0 and force at index 1
    */
    virtual void implConsumeBodySpatialVec(
        const SimTK::State& state,
        const PhysicalFrame& body,
        const SimTK::SpatialVec& spatialVec)
    {}

    /**
     * Subclasses of `ForceConsumer` may implement this method. There are no expectations
     * on how an implementation should handle a call to this function.
     *
     * @param state    the state that was used to evaluate the force
     * @param frame    the frame in which `point` is defined
     * @param point    a point in `frame` where `force` applies
     * @param force    the force vector, specified in the inertial (ground) frame
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
