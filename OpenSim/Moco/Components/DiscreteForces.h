#ifndef OPENSIM_DISCRETEFORCES_H
#define OPENSIM_DISCRETEFORCES_H
/* -------------------------------------------------------------------------- *
 * OpenSim: DiscreteForces.h                                                  *
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

#include <OpenSim/Moco/osimMocoDLL.h>

namespace OpenSim {

/** This class is a thin wrapper to Simbody's SimTK::Force::DiscreteForces
class. Adding this component to a Model will add corresponding slots in the
State for discrete variables holding the most recent value of the forces set
through the component. Discrete variables are *not* updated through the same
realization mechanisms as typical continuous variables are, and so must be
manually updated before realizing to acceleration whenever their values
change. However, this class is mainly used within MocoProblemRep and classes
derived from MocoSolver to handle constraint forces, and is not intended for
use outside of Moco. The wrapper to OpenSim is necessary so that the
discrete variables appear in the State whenever initSystem() is called on
a model. */
class OSIMMOCO_API DiscreteForces : public ModelComponent {
    OpenSim_DECLARE_CONCRETE_OBJECT(DiscreteForces, ModelComponent);

public:
    DiscreteForces() : ModelComponent() {}

    void setAllForces(SimTK::State& s, 
            const SimTK::Vector& generalizedForces,
            const SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInG) const;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    mutable SimTK::Force::DiscreteForces m_discrete_forces;

}; // class DiscreteForces

} // namespace OpenSim

#endif // OPENSIM_DISCRETEFORCES_H
