/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MomentArmSolver.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "MomentArmSolver.h"
#include "Model/PointForceDirection.h"
#include "Model/Model.h"

using namespace std;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the MomentArmSolver 
 *
 */
MomentArmSolver::MomentArmSolver(const Model &model) : Solver(model)
{
    setAuthors("Ajay Seth");
    _stateCopy = model.getWorkingState();

    // Get the body forces equivalent of the point forces of the path
    _bodyForces = getModel().getSystem().getRigidBodyForces(
            _stateCopy, SimTK::Stage::Instance);
    // get the right size coupling vector
    _coupling = _stateCopy.getU();
}

/*********************************************************************************
Solve for moment arm, r = d(path_length)/d(theta)

Ideally we want to compute r without perturbing theta values and recomputing
lengths because it is inexact, very sensitive to perturbations that may cause a 
path (via) point to appear or disappear, and inefficient to recompute lengths, 
especially for complex paths involving wrapping.

Refer to Moment-arm Theory document by Michael Sherman for details.

**********************************************************************************/
double MomentArmSolver::solve(const SimTK::State& state,
        const Coordinate& aCoord, const AbstractGeometryPath& path) const {
    //Local modifiable copy of the state
    SimTK::State& s_ma = _stateCopy;
    s_ma.updQ() = state.getQ();

    // compute the coupling between coordinates due to constraints
    _coupling = computeCouplingVector(s_ma, aCoord);

    // set speeds to zero
    s_ma.updU() = 0;

    // zero out all the forces
    _bodyForces *= 0;
    _generalizedForces = 0;

    // apply a tension of unity to the bodies of the path
    SimTK::Vector pathDependentMobilityForces(s_ma.getNU(), 0.0);
    path.addInEquivalentForces(s_ma, 1.0, _bodyForces, pathDependentMobilityForces);

    //_bodyForces.dump("bodyForces from addInEquivalentForcesOnBodies");

    // Convert body spatial forces F to equivalent mobility forces f based on 
    // geometry (no dynamics required): f = ~J(q) * F.
    getModel().getMultibodySystem().getMatterSubsystem()
        .multiplyBySystemJacobianTranspose(s_ma, _bodyForces, _generalizedForces);

    _generalizedForces += pathDependentMobilityForces;
    // Moment-arm is the effective torque (since tension is 1) at the 
    // coordinate of interest taking into account the generalized forces also 
    // acting on other coordinates that are coupled via constraint.
    return ~_coupling*_generalizedForces;
}

double MomentArmSolver::solve(const SimTK::State& state,
        const Coordinate& aCoord,
        const Array<PointForceDirection*>& pfds) const {
    //const clock_t start = clock();

    //Local modifiable copy of the state
    SimTK::State& s_ma = _stateCopy;
    s_ma.updQ() = state.getQ();

    // compute the coupling between coordinates due to constraints
    _coupling = computeCouplingVector(s_ma, aCoord);

    // set speeds to zero
    s_ma.updU() = 0;

    int n = pfds.getSize();
    // Apply body forces along the geometry described by pfds due to a tension of 1N
    for(int i=0; i<n; i++) {
        getModel().getMatterSubsystem().
            addInStationForce(s_ma, 
                pfds[i]->frame().getMobilizedBodyIndex(), 
                pfds[i]->point(), pfds[i]->direction(), _bodyForces);
    }

    //_bodyForces.dump("bodyForces from PointForceDirections");

    // Convert body spatial forces F to equivalent mobility forces f based on 
    // geometry (no dynamics required): f = ~J(q) * F.
    getModel().getMultibodySystem().getMatterSubsystem()
        .multiplyBySystemJacobianTranspose(s_ma, _bodyForces, _generalizedForces);

    // Moment-arm is the effective torque (since tension is 1) at the 
    // coordinate of interest taking into account the generalized forces also 
    // acting on other coordinates that are coupled via constraint.
    return ~_coupling*_generalizedForces;
}

SimTK::Vector MomentArmSolver::computeCouplingVector(SimTK::State &state, 
        const Coordinate &coordinate) const
{
    // make sure copy of the state is realized to at least instance
    getModel().getMultibodySystem().realize(state, SimTK::Stage::Instance);

    // unlock the coordinate if it is locked
    coordinate.setLocked(state, false);

    // Calculate coupling matrix C to determine the influence of other coordinates 
    // (mobilities) on the coordinate of interest due to constraints
    state.updU() = 0;
    // Light-up speed of coordinate of interest and see how other coordinates
    // affected by constraints respond
    coordinate.setSpeedValue(state, 1);
    getModel().getMultibodySystem().realize(state, SimTK::Stage::Velocity);

    // Satisfy all the velocity constraints.
    getModel().getMultibodySystem().projectU(state, 1e-10);
    
    // Now calculate C. by checking how speeds of other coordinates change
    // normalized by how much the speed of the coordinate of interest changed 
    return state.getU() / coordinate.getSpeedValue(state);
}

} // end of namespace OpenSim
