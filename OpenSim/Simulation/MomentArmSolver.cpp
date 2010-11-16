/* MomentArmSolver.cpp 
* Author: Ajay Seth 
* Copyright (c)  2010 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
 * @param aOptimizationTarget The target that IK will minimize
 * @param aIKTrial Parameters specified in input file to control IK.
 */
MomentArmSolver::MomentArmSolver(const Model &model) : Solver(model)
{
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
	//Local modifiable copy of the state
	State s_ma = s;

	aCoord.setLocked(s_ma, false);
	aCoord.setClamped(s_ma, false);

	double angle = aCoord.getValue(s_ma);

	_model.getMultibodySystem().realize(s_ma, SimTK::Stage::Instance);

	// Calculate coupling matrix C to determine the influence of other coordinates 
	// (mobilities) on the coordinate of interest due to constraints
    // First declare dummies for the call to project().
    const Vector yWeights(s_ma.getNY(), 1); 
    const Vector cWeights(s_ma.getNMultipliers(), 1);
    Vector yErrEst;

    s_ma.updU() = 0;
	// Light-up speed of coordinate of interest and see how other coordinates
	// affected by constraints respond
    aCoord.setSpeedValue(s_ma, 1);

	_model.getMultibodySystem().realize(s_ma, SimTK::Stage::Velocity);

    _model.getMultibodySystem().project(s_ma, 1e-10, 
        yWeights, cWeights, yErrEst, System::ProjectOptions::VelocityOnly);
	
	// Now calculate C. by checking how speeds of other coordinates change
	// normalized by how much the speed of the coordinate of interest changed 
    const Vector C = s_ma.getU() / aCoord.getSpeedValue(s_ma);

	angle = aCoord.getValue(s_ma);

	// Reset speeds to zero
    s_ma.updU() = 0;
	
	// Get the body forces equivalent of the point forces of the path
	Vector_<SpatialVec> bodyForces(_model.getNumBodies(), SpatialVec(0));
			
	// Apply body forces along the geometry described by pfds due to a tension of 1N
	for(int i=0; i < pfds.getSize(); i++) {
		_model.getMatterSubsystem().addInStationForce(s_ma, SimTK::MobilizedBodyIndex(pfds[i]->body().getIndex()), 
												   pfds[i]->point(), pfds[i]->direction(), bodyForces);
	}

	//Mapping spatial forces to generalized forces
	Vector generalizedForces;

	// Convert body forces to equivalent mobility forces based on geometry (no dynamics required)
	_model.getMultibodySystem().getMatterSubsystem().calcInternalGradientFromSpatial(s_ma, bodyForces, generalizedForces);

	// Moment-arm is the effective torque (since tension is 1) at the coordinate of interest taking into account
	// the generalized forces also acting on other coordinates that are coupled via constraint
	return ~C*generalizedForces;
}

} // end of namespace OpenSim