#ifndef __InverseDynamicsSolver_h__
#define __InverseDynamicsSolver_h__
// InverseDynamicsSolver.h
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
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

#include "Solver.h"

namespace OpenSim {

class FunctionSet;

//=============================================================================
//=============================================================================
/**
 * Solve for the generalized coordinate coordinate forces (1 per degrees-of-freedom)
 * That satisfy the unconstrained equations of motion given kinemtics: q, u, u_dot

 * The InverseDynamics equation: Tau = M*u_dot-G(q)-C(q,u)-A(q,u,t,x)
 *
 * The InverseDynamicsSolver utiliizes efficient methods in Simbody(TM) to compute
 * the generalized forces, Tau, without explicitly forming the Mass matrix, M.
 * System gravity, G, centrifugal and coriolis, C, forces are computed internally.
 * Caller provides q,u,t (supplied by the State), the desired u_dot and the 
 * applied loads, A. If applied loads are from forces in the model, these loads
 * are automatically accounted for unless explicitly provided.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API InverseDynamicsSolver: public Solver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~InverseDynamicsSolver() {};

	/** Construct an InverseDynamics solver with the coordinate references as the goal
	    of the InverseDynamics and (optional)constraint weight. Defual is infitinet
		constraint weighting (i.e. rigidly enforced) during InverseDynamics. */
	InverseDynamicsSolver(const Model &model);
	
	/** Solve the inverse dynamics system of equations for generalized coordinate forces, Tau. 
	    Applied loads are computed by the model from the state.	 */
	virtual SimTK::Vector solve(SimTK::State &s, const SimTK::Vector &udot = SimTK::Vector(0));

	/** Solve the inverse dynamics system of equations for generalized coordinate forces, Tau. 
	    Applied loads are explicity provided as generalized coordinate forces (MobilityForces)
		and/or a Vector of Spatial-body forces */
	virtual SimTK::Vector solve(const SimTK::State &s, const SimTK::Vector &udot, 
		const SimTK::Vector &appliedMobilityForces, 
		const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces);
	
	/** Solve the inverse dynamics system of equations for generalized coordinate forces, Tau. 
	    Now the state is updated from known coordinates, q, as functions of time.
		Coordinate functions must be twice differentiable. 
		NOTE: forces with internal states should be removed/disabled prior to solving if default state
		      is inadequate */
	virtual SimTK::Vector solve(SimTK::State &s, const FunctionSet &Qs, double time);
	/** Same as above but for a given time series populate a Vector (trajectory)of generalized-coordinate forces */
	virtual void solve(SimTK::State &s, const FunctionSet &Qs, const SimTK::Array_<double> &times, 
		SimTK::Array_<SimTK::Vector> &genForceTrajectory);

//=============================================================================
};	// END of class InverseDynamicsSolver
//=============================================================================
} // namespace

#endif // __InverseDynamicsSolver_h__
