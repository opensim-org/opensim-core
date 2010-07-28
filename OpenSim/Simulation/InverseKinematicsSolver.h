#ifndef __InverseKinematicsSolver_h__
#define __InverseKinematicsSolver_h__
// InverseKinematicsSolver.h
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
	const MarkersReference &_markersReference;

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
