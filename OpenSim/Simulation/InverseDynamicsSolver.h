#ifndef OPENSIM_InverseDynamicsSolver_H_
#define OPENSIM_InverseDynamicsSolver_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  InverseDynamicsSolver.h                      *
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

#include "Solver.h"

namespace OpenSim {

class FunctionSet;

//=============================================================================
//=============================================================================
/**
 * Solve for the generalized coordinate coordinate forces (1 per degree-of-freedom)
 * that satisfy the unconstrained equations of motion given kinemtics: q, u, u_dot

 * The InverseDynamics equation: Tau = M*u_dot-G(q)-C(q,u)-A(q,u,t,x)
 *
 * The InverseDynamicsSolver utiliizes efficient methods in Simbody(TM) to compute
 * the generalized forces, Tau, without explicitly forming the Mass matrix, M.
 * System gravity, G, centrifugal and coriolis, C, forces are computed internally.
 * Caller provides q,u,t (supplied by the State), the desired u_dot and the 
 * applied loads, A. If applied loads are from forces in the model, these loads
 * are automatically accounted for unless explicitly disabled.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API InverseDynamicsSolver: public Solver {
OpenSim_DECLARE_CONCRETE_OBJECT(InverseDynamicsSolver, Solver);

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
	/** Construct an InverseDynamics solver applied to the provided model */
	InverseDynamicsSolver(const Model& model);
	
	/** Solve the inverse dynamics system of equations for generalized coordinate forces, Tau. 
	    Applied loads are computed by the model from the state.	 */
	virtual SimTK::Vector solve(const SimTK::State& s, const SimTK::Vector& udot = SimTK::Vector(0));

	/** Solve the inverse dynamics system of equations for generalized coordinate forces, Tau. 
	    Applied loads are explicity provided as generalized coordinate forces (MobilityForces)
		and/or a Vector of Spatial-body forces */
	virtual SimTK::Vector solve(const SimTK::State& s, const SimTK::Vector& udot, 
		const SimTK::Vector& appliedMobilityForces, 
		const SimTK::Vector_<SimTK::SpatialVec>& appliedBodyForces);
	
	/** Solve the inverse dynamics system of equations for generalized coordinate forces, Tau. 
	    Now the state is updated from known coordinates, q, as functions of time.
		Coordinate functions must be twice differentiable. 
		NOTE: forces with internal states should be removed/disabled prior to solving if default state
		      is inappropriate */
	virtual SimTK::Vector solve(SimTK::State& s, const FunctionSet& Qs, double time);
#ifndef SWIG
    /** Same as above but for a given time series populate an Array (trajectory) of generalized-coordinate
	    forces (Vectors) */
	virtual void solve(SimTK::State& s, const FunctionSet& Qs, const SimTK::Array_<double>& times, 
		SimTK::Array_<SimTK::Vector> &genForceTrajectory);
#endif
//=============================================================================
};	// END of class InverseDynamicsSolver
//=============================================================================
} // namespace

#endif // OPENSIM_InverseDynamicsSolver_H_
