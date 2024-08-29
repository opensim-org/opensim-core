#ifndef OPENSIM_FORCE_APPLIER_H_
#define OPENSIM_FORCE_APPLIER_H_

/* -------------------------------------------------------------------------- *
 *                         OpenSim: ForceApplier.h                            *
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

#include <OpenSim/Simulation/Model/ForceConsumer.h>

#include <OpenSim/Simulation/osimSimulationDLL.h>

#include <SimTKcommon/internal/MassProperties.h>  // for `SimTK::SpatialVec`
#include <SimTKcommon/internal/Vector_.h>         // for `SimTK::Vector_`

namespace SimTK { class SimbodyMatterSubsystem; }

namespace OpenSim
{

/**
 * A `ForceApplier` is a concrete `ForceConsumer` implementation that:
 *
 * - Converts any point forces into body forces
 * - Applies body forces to a `SimTK::Vector_<SimTK::SpatialVec>`
 * - Applies generalized (mobility) forces to a `SimTK::Vector`
 *
 * The `ForceApplier` is mostly useful as an internal class for adapting
 * the `ForceProducer`/`ForceConsumer` APIs to the `simbody` / `OpenSim::Force`
 * API. In effect, a `ForceApplier` class concretely implements "undefined
 * virtual consumption" (`OpenSim::ForceConsumer`) as "applies forces to a
 * multibody system" (`OpenSim::Force`).
 */
class OSIMSIMULATION_API ForceApplier final : public ForceConsumer {
public:
    explicit ForceApplier(
        const SimTK::SimbodyMatterSubsystem& matter,
        SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
        SimTK::Vector& generalizedForces) :

        _matter{&matter},
        _bodyForces{&bodyForces},
        _generalizedForces{&generalizedForces}
    {}
private:

    void implConsumeGeneralizedForce(const SimTK::State&, const Coordinate&, double) final;
    void implConsumeBodySpatialVec(const SimTK::State&, const PhysicalFrame&, const SimTK::SpatialVec&) final;
    void implConsumePointForce(const SimTK::State&, const PhysicalFrame&, const SimTK::Vec3&, const SimTK::Vec3&) final;

    const SimTK::SimbodyMatterSubsystem* _matter;
    SimTK::Vector_<SimTK::SpatialVec>* _bodyForces;
    SimTK::Vector* _generalizedForces;
};

}  // namespace OpenSim

#endif // OPENSIM_FORCE_APPLIER_H_