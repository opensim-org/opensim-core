#ifndef __InverseKinematicsSolver_h__
#define __InverseKinematicsSolver_h__
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  InverseKinematicsSolver.h                     *
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

#include "AssemblySolver.h"
#include <OpenSim/Common/Set.h>

namespace OpenSim {

class CoordinateReference;
class MarkersReference;

//=============================================================================
//=============================================================================
/**
 * Solve for the coordinates (degrees-of-freedom) of the model that satisfy the
 * set of constraints imposed on the model as well as set of desired coordinate
 * values.  The InverseKinematics provides the option to convert the problem to  
 * an approximate one where the constraint violations are treated as penalties
 * to be minimized rather than strictly enforced. This can speed up the time
 * solution and can be used to seed the constrained problem near to a solution.
 *
 * The InverseKinematics objective: 
 *   min J = sum(Wm*(m_i-md_i)^T*(m_i-md_i)) + sum(Wq_i*(q_i-qd_i)^2)) + [Wc*sum(c_err)^2]
 * where m_i and md_i are the model and desired marker coordinates (Vec3) 
 * iff Wc == Infinity, second term is not included, but
 *  A is subject to the constraint equations:  G(q)-Go = 0
 *
 * When the model (and the number of goals) is guaranteed not to change and the 
 * the initial state is close to the InverseKinematics solution (from initial assemble(),
 * then track() is a efficient method for updating the configuration to track
 * the small change to the desired coorindate value.
 *
 * See SimTK::Assembler for more algorithmic details of the underlying solver.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API InverseKinematicsSolver: public AssemblySolver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:

	// The marker reference values and weightings
	MarkersReference &_markersReference;

	// Markers collectively form a single assembly condition for the SimTK::Assembler
	SimTK::Markers *_markerAssemblyCondition;


//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~InverseKinematicsSolver() {}

	InverseKinematicsSolver(const Model &model, MarkersReference &markersReference,
							SimTK::Array_<CoordinateReference> &coordinateReferences,
							double constraintWeight = SimTK::Infinity);
	
	/** Assemble a model configuration that meets the InverseKinematics conditions  
	    (desired values and constraints) starting from an initial state that  
		does not have to satisfy the constraints. */
	//virtual void assemble(SimTK::State &s);

	/** Obtain a model configuration that meets the InverseKinematics conditions  
	    (desired values and constraints) given a state that satisfies or
		is close to satisfying the constraints. Note there can be no change
		in the number of constrainst or desired coordinates. Desired
		coordinate values can and should be updated between repeated calls
		to track a desired trajectory of coordinate values. */
	//virtual void track(SimTK::State &s);

	/** Change the weighting of a marker to take affect when assemble or track is called next. 
		Update a marker's weight by name. */
	void updateMarkerWeight(const std::string &markerName, double value);
	/** Update a marker's weight by its index. */
	void updateMarkerWeight(int markerIndex, double value);
	/** Update all markers weights by order in the markersReference passed in to
	    construct the solver. */
	void updateMarkerWeights(const SimTK::Array_<double> &weights);

	/** Compute and return the spatial location of a marker in ground. */
	SimTK::Vec3 computeCurrentMarkerLocation(const std::string &markerName);
	SimTK::Vec3 computeCurrentMarkerLocation(int markerIndex);
	/** Compute and return the spatial locations of all markers in ground. */
	void computeCurrentMarkerLocations(SimTK::Array_<SimTK::Vec3> &markerLocations);

	/** Compute and return the distance error between model marker and observation. */
	double computeCurrentMarkerError(const std::string &markerName);
	double computeCurrentMarkerError(int markerIndex);
	/** Compute and return the distance errors between all model markers and their observations. */
	void computeCurrentMarkerErrors(SimTK::Array_<double> &markerErrors);

	/** Compute and return the squared-distance error between model marker and observation. 
	    This is cheaper than calling the error and squaring it, since distance from norm-2 */
	double computeCurrentSquaredMarkerError(const std::string &markerName);
	double computeCurrentSquaredMarkerError(int markerIndex);
	/** Compute and return the distance errors between all model marker and observations. */
	void computeCurrentSquaredMarkerErrors(SimTK::Array_<double> &markerErrors);
	/** Marker locations and errors may be computed in an order that is different
	    from tasks file or listed in the model. Retrun the corresponding marker
		name for an index in the list of marker locations/errors returned by the
		solver. */
	std::string getMarkerNameForIndex(int markerIndex) const;

protected:
	/** Internal method to convert the CoordinateReferences into goals of the 
		assembly solver. Subclasses, can add oveeride  to include other goals  
		such as point of interest matching (Marker tracking). This method is
		automatically called by assemble. */
	virtual void setupGoals(SimTK::State &s);
	/** Internal method to update the time, reference values and/or their 
		weights that define the goals, based on the passed in state */
	virtual void updateGoals(const SimTK::State &s);

private:
	// Non-accessible cache of the marker values to be matched at a given state
	SimTK::Array_<SimTK::Vec3> _markerValues;


//=============================================================================
};	// END of class InverseKinematicsSolver
//=============================================================================
} // namespace

#endif // __InverseKinematicsSolver_h__
