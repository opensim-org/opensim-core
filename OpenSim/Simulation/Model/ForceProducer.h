#ifndef OPENSIM_FORCE_PRODUCER_H_
#define OPENSIM_FORCE_PRODUCER_H_

/* -------------------------------------------------------------------------- *
 *                         OpenSim: ForceProducer.h                           *
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim { class ForceConsumer; }
namespace SimTK { class State; }

namespace OpenSim {

/**
 * A `ForceProducer` is an abstract `OpenSim::Force` that can emit (produce)
 * its forces one-by-one into a virtual `OpenSim::ForceConsumer`.
 *
 * The benefit of this is that it enables arbitrary external code to directly
 * introspect each force before it gets resolved to the underlying body-/generalized-force
 * vector that `OpenSim::Force::computeForce` manipulates. This can be useful for
 * visualizing/dumping user data (e.g. because user-written `OpenSim::ExternalForce`s
 * produce point-forces) or debugging (because it's easier to debug forces if they
 * come one-at-a-time rather than trying to figure out which parts of downstream code
 * touched which parts of a `SimTK::Vector_<SimTK::SpatialVec>` during
 * `OpenSim::Force::computeForce`).
 */
class OSIMSIMULATION_API ForceProducer : public Force {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ForceProducer, Force);

protected:
    using Force::Force;  // forward the `Force` constructor

public:

    /**
     * Uses `implProduceForces` to produce (emit) forces evaluated from `state` into the
     * provided `ForceConsumer`.
     *
     * @note this function only produces the forces and does not apply them to anything. It's
     *       up to the `ForceConsumer` implementation to handle the forces. Therefore,
     *       `Force::appliesForces` is ignored by this method.
     *
     * @param state       the state used to evaluate forces
     * @param consumer    a `ForceConsumer` that shall receive each of the produced forces
     */
    void produceForces(const SimTK::State& state, ForceConsumer& forceConsumer) const
    {
        implProduceForces(state, forceConsumer);
    }

    /**
     * Inhereted from `OpenSim::Force`.
     *
     * `ForceProducer` overrides `OpenSim::Force::computeForce` with a default
     * implementation that, provided `OpenSim::Force::appliesForces` is `true`,
     * internally uses `produceForces` to mutate the provided `bodyForces` in a
     * manner that's compatible with the `OpenSim::Force` API.
     */
    void computeForce(
        const SimTK::State& state,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces
    ) const override;

private:

    /**
     * Subclasses of `ForceProducer` must implement this method.
     *
     * Implementations should evaluate forces from `state` and pass them into
     * `forceConsumer` by calling the most appropriate `consume*` function. The
     * `ForceConsumer`'s API documentation outlines each available consumption
     * function (+ preferred usage).
     */
    virtual void implProduceForces(
        const SimTK::State& state,
        ForceConsumer& forceConsumer
    ) const = 0;
};

}

#endif  // OPENSIM_FORCE_PRODUCER_H_
