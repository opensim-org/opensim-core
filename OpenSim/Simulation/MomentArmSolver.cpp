/* MomentArmSolver.cpp 
* Copyright (c)  2006 Stanford University
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

//______________________________________________________________________________
/**
 * This is the heart of the MomentArmSolver, th

 */
/*********************************************************************************
Solve for moment arms

Problem statement by Michael Sherman

s = scalar muscle tension
F(s) = set of spatial forces produced by s
fa = generalized force equivalent of all other active forces, e.g. gravity and
     forces applied by other muscles
C = velocity dependent centrifugal forces (as generalized forces)

We want to know Tau i(s), the generalized force that arises at the ith generalized
coordinate as a result of s, then the moment arm ri of the muscle about the ith
coordinate is ri=Tau i(s)/s.

Theory

We have equations of motion for a constraint system:
(a)    M*udot  + GT*lambda + C = JT*F(s) + fa

From (a):
(1)   M*udot  = JT*F(s) – GT*lambda + (fa–C)

With s==0, F(s)==0, we have
(2)   M*udot = –GT*lambda0 + (fa–C)

With s==1, we have
(3)   M*udot  = JT*F(1) – GT*lambda1 + (fa–C)

Subtract (2) from (3):
(4)   M(udot1–udot0) = JT*F(1) – GT*(lambda1–lambda0)
           = Tau (s)

because that’s the change in the generalized forces that was produced by the applied
tension s. And since we set s to 1, r=Tau (s)/s=Tau (s) so this reads out directly as the
set of generalized moment arms for this muscle.

Note: because calculate of lambda is dependent on mass properties, in constrained cases the
generalized moment arm will be exact only if the mass properties are correct; I believe
the results will still be reasonable with assumed mass properties but that needs to be
tested. In unconstrained cases Tau (s)=JTF(1) which is independent of mass properties.

Algorithm
(1)   Make sure model has non-singular mass properties (modify temporarily if necessary).
(2)   Turn off muscle (s=0)
(3)   Evaluate  with realize(Acceleration)
(4)   Turn on muscle (s=1)
(5)   Evaluate  with realize(Acceleration)
(6)   Calculate Tau (s)=M(–) using calcMV()
(7)   Report whichever element or elements of Tau were requested.

**********************************************************************************/
Vector MomentArmSolver::solve(const State &s, const Array<PointForceDirection *> &pfds)
{
	//Local modifiable copy of the state
	State s_ma = s;
	
	const ForceSet &modelForces = _model.getForceSet();
	// Disable all the Forces in the model for this solver
	for(int i=0; i < modelForces.getSize(); i++) {
		modelForces[i].setDisabled(s_ma, true);
	}
	// Also disable gravity
	_model.getGravityForce().disable(s_ma);
	
	// Get the accelerations with no forces applied.
	_model.getMultibodySystem().realize(s_ma, SimTK::Stage::Acceleration);
	SimTK::Vector uDotZero = s_ma.getUDot();

	// Get the cached set of body forces
	Vector_<SpatialVec>& bodyForces = _model.getMultibodySystem().updRigidBodyForces(s_ma,Stage::Dynamics);
			
	// Apply body forces along the geometry described by pfds
	for(int i=0; i < pfds.getSize(); i++) {
		_model.getMatterSubsystem().addInStationForce(s_ma, SimTK::MobilizedBodyIndex(pfds[i]->body().getIndex()), 
												   pfds[i]->point(), pfds[i]->direction(), bodyForces);
	}

	// Force a re-evaluation for the acceleration to take into account added in forces
	s_ma.invalidateAll(Stage::Acceleration);

	// Get the accelerations with unity force applied.
	_model.getMultibodySystem().realize(s_ma,SimTK::Stage::Acceleration);

	// Get the accelerations when this actuator has unit force.
	_model.getMultibodySystem().realize(s_ma, SimTK::Stage::Acceleration);
	SimTK::Vector uDotOne = s_ma.getUDot();

	// Calculate the differences in the accelerations.
	SimTK::Vector uDotDiff = uDotOne-uDotZero;

	// Get the generalized forces corresponding to the accelerations. These are the moment arms
	// because the force is 1.0.
	SimTK::Vector momentArms(s_ma.getNQ());
	_model.getMatterSubsystem().calcMV(s_ma, uDotDiff, momentArms);

	return momentArms;
}

} // end of namespace OpenSim