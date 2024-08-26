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

#include "OpenSim/Common/Object.h"
#include "OpenSim/Simulation/Model/Force.h"
#include "OpenSim/Simulation/osimSimulationDLL.h"

#include <functional>

namespace OpenSim { class ForceConsumer; }
namespace SimTK { class State; }

namespace OpenSim {

/**
 * A `ForceProducer` is an abstract `OpenSim::Force` that can emit (produce)
 * its forces one-by-one into a virtual `OpenSim::ForceConsumer`.
 *
 * The benefit of this is that it enables arbitrary external code to directly
 * introspect each force before it gets resolved to the underlying body-/generalized-force
 * vector that `OpenSim::Force::computeForce` uses. This can be useful for
 * visualizing/dumping user data (e.g. because user-written `OpenSim::ExternalForce`s
 * produce point-forces) or debugging (because it's easier to debug forces if they
 * come one-at-a-time rather than trying to figure out which parts of downstream code
 * touched which parts of a `SimTK::Vector_<SimTK::SpatialVec>` during
 * `OpenSim::Force::computeForce`).
 */
class OSIMSIMULATION_API ForceProducer : public Force {
    OpenSim_DECLARE_ABSTRACT_OBJECT(ForceProducer, Force);

public:

    /**
     * Requests that this `ForceProducer` emits the forces that it wants to apply
     * to `state` into the provided `consumer`.
     */
    void produceForces(
        const SimTK::State& state,
        ForceConsumer& consumer) const
    {
        implProduceForces(state, consumer);
    }

    /**
     * Inhereted from `OpenSim::Force`.
     *
     * `ForceProducer` overrides `OpenSim::Force::computeForce` with a default
     * implementation that internally consumes the forces produced by `produceForces`
     * to mutate the provided `bodyForces` as-required by the `OpenSim::Force` API.
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
     * Implementations should provide each force that this component wants to apply to `state`
     * to the `consumer` by calling `consumer`'s highest-available consumption function
     * (e.g. prefer `ForceConsumer::consumePointForce` over `ForceConsumer::consumeBodyForce`
     *  where applicable - it enables code-reuse and introspection).
     */
    virtual void implProduceForces(
        const SimTK::State& state,
        ForceConsumer& consumer
    ) const = 0;
};

}

#endif  // OPENSIM_FORCE_PRODUCER_H_
