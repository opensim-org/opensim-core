#ifndef MOCO_DISCRETEFORCES_H
#define MOCO_DISCRETEFORCES_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DiscreteForces.h                                             *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <simbody/internal/Force_DiscreteForces.h>

#include "../osimMocoDLL.h"

namespace OpenSim {

class OSIMMOCO_API DiscreteForces : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(DiscreteForces, ModelComponent);

public:
    DiscreteForces() : ModelComponent() {}

    void setAllGeneralizedForces(SimTK::State& s, 
            const SimTK::Vector& generalizedForces) const;
    void setAllBodyForces(SimTK::State& s, 
            const SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInG) const;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    mutable SimTK::Force::DiscreteForces m_discrete_forces;

}; // class DiscreteForces

} // namespace OpenSim

#endif // MOCO_DISCRETEFORCES_H

