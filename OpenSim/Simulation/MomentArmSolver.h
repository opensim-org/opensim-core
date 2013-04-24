#ifndef OPENSIM_MomentArmSolver_H_
#define OPENSIM_MomentArmSolver_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  MomentArmSolver.h                         *
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

class GeometryPath;
class PointForceDirection;
class Coordinate;

//=============================================================================
//=============================================================================
/**
 * Solve for the effective moment arms at all degrees-of-freedom due to one or
 * more point forces.  This may result from the underlying geometry of a Force 
 * or Actuator with a complex path (like ligaments and muscles) but this solver
 * is only concerned with the set of points and unit forces that maps a scalar
 * force value (like tension) to the resulting generalized force.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API MomentArmSolver: public Solver {
OpenSim_DECLARE_CONCRETE_OBJECT(MomentArmSolver, Solver);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	explicit MomentArmSolver(const Model& model);
	virtual ~MomentArmSolver() {}

	/** Solve for the effective moment-arm about the all coordinates (q) based 
    on the geometric distribution of forces described by GeometryPath. 
	@param  s				    current state of the model
	@param  path	            path for which to calculate moment-arms
	@return Vector ma			resulting moment-arms for each mobility in model
								has the same order and length as state.getU()
	*/
	SimTK::Vector solve(const SimTK::State& s, const GeometryPath &path);

	/** Solve for the effective moment-arm about the specified coordinate based 
    on the geometric distribution of forces described by the list of 
    PointForceDirections. */
	double solve(const SimTK::State& s, const Coordinate &aCoord, 
		const Array<PointForceDirection *> &pfds);

private:
	// Internal state of the solver initialized as a copy of the default state
	SimTK::State _stateCopy;

	// Keep preallocated vector of the generalized forces
	SimTK::Vector _generalizedForces;

	// Keep preallocated vector of the Body_Forces
	SimTK::Vector_<SimTK::SpatialVec> _bodyForces;

	// Keep preallocated vector of the coupling constraint factors
	SimTK::Vector _coupling;


public:


//=============================================================================
};	// END of class MomentArmSolver
//=============================================================================
} // namespace

#endif // OPENSIM_MomentArmSolver_H_
