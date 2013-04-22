/* -------------------------------------------------------------------------- *
 *                       OpenSim:  MomentArmSolver.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
using namespace SimTK;

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
	_bodyForces = Vector_<SpatialVec>(getModel().getNumBodies(), SpatialVec(0));
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
double MomentArmSolver::solve(const State &s, const Coordinate &aCoord,
							  const Array<PointForceDirection *> &pfds)
{
	//const clock_t start = clock();

	//Local modifiable copy of the state
	_stateCopy.updQ() = s.getQ();
	_stateCopy.updU() = s.getU();
	State& s_ma = _stateCopy;

	//const clock_t copyT = clock();
	//cout << "MomentArmSolver::solve State copy time:" << 1000*(copyT-start)/CLOCKS_PER_SEC << "ms\n" <<endl;

	getModel().getMultibodySystem().realize(s_ma, SimTK::Stage::Instance);

	aCoord.setLocked(s_ma, false);

	double angle = aCoord.getValue(s_ma);	

	//const clock_t init = clock();
	//cout << "MomentArmSolver::solve realize instance in:" << 1000*(init-copyT)/CLOCKS_PER_SEC << "ms\n" <<endl;


	// Calculate coupling matrix C to determine the influence of other coordinates 
	// (mobilities) on the coordinate of interest due to constraints

    s_ma.updU() = 0;
	// Light-up speed of coordinate of interest and see how other coordinates
	// affected by constraints respond
    aCoord.setSpeedValue(s_ma, 1);

	getModel().getMultibodySystem().realize(s_ma, SimTK::Stage::Velocity);


    // Satisfy all the velocity constraints.
    getModel().getMultibodySystem().projectU(s_ma, 1e-10);
	
	
	// Now calculate C. by checking how speeds of other coordinates change
	// normalized by how much the speed of the coordinate of interest changed 
    _coupling = s_ma.getU() / aCoord.getSpeedValue(s_ma);

	//const clock_t computeC = clock();
	//cout << "MomentArmSolver::solve compute C time:" << 1000*(computeC-init)/CLOCKS_PER_SEC << "ms\n" <<endl;


	angle = aCoord.getValue(s_ma);

	// Reset speeds to zero
    s_ma.updU() = 0;
	
			
	// Apply body forces along the geometry described by pfds due to a tension of 1N
	for(int i=0; i < pfds.getSize(); i++) {
		getModel().getMatterSubsystem().addInStationForce(s_ma, SimTK::MobilizedBodyIndex(pfds[i]->body().getIndex()), 
												   pfds[i]->point(), pfds[i]->direction(), _bodyForces);
	}

	// Convert body spatial forces F to equivalent mobility forces f based on 
    // geometry (no dynamics required): f = ~J(q) * F.
	getModel().getMultibodySystem().getMatterSubsystem()
        .multiplyBySystemJacobianTranspose(s_ma, _bodyForces, _generalizedForces);

	//const clock_t torqueT = clock();;
	//cout << "MomentArmSolver::solve  compute effective torque in:" << 1000*(torqueT-computeC)/CLOCKS_PER_SEC << "ms\n" <<endl;

	// Moment-arm is the effective torque (since tension is 1) at the 
    // coordinate of interest taking into account the generalized forces also 
    // acting on other coordinates that are coupled via constraint.
	return ~_coupling*_generalizedForces;
}

} // end of namespace OpenSim