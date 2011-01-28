#ifndef __AssemblySolver_h__
#define __AssemblySolver_h__
// AssemblySolver.h
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
#include <OpenSim/Common/Set.h>

namespace OpenSim {

class CoordinateReference;

//=============================================================================
//=============================================================================
/**
 * Solve for the coordinates (degrees-of-freedom) of the model that satisfy the
 * set of constraints imposed on the model as well as set of desired coordinate
 * values.  The AssembleSolver provides the option to convert the problem to an 
 * approximate one where the constraint violations are treated as penalties to
 * to be minimized rather than strictly enforced. This can speed up the time
 * solution and can be used to seed the constrained problem near to a solution.
 *
 * The assembly objective: min A = sum(Wq_i*(q_i-qd_i)^2)) + [Wc*sum(c_err)^2]
 * iff Wc == Infinity, second term is not included, but
 *  A is subject to the constraint equations:  G(q)-Go = 0
 *
 * When the model (and the number of goals) is guaranteed not to change and the 
 * the initial state is close to the assembly solution (from initial assembly(),
 * then track() is a efficient method for updating the configuration to track
 * the small change to the desired coorindate value.
 *
 * See SimTK::Assembler for more algorithmic details of the underlying solver.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API AssemblySolver: public Solver
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:

	// The assembly solution accuracy
	double _accuracy;

	// Weight for built-in constraints to be satisfied
	double _constraintWeight;

	// The coordinates reference value and weighting
	SimTK::Array_<CoordinateReference> &_coordinateReferences;

	// Underlying SimTK::Assembler that will perform the assembly
	SimTK::Assembler *_assembler;

	SimTK::Array_<SimTK::QValue*> _coordinateAssemblyConditions;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~AssemblySolver();

	/** Construct an Assembly solver with the coordinate references as the goal
	    of the assembly and (optional)constraint weight. Defual is infitinet
		constraint weighting (i.e. rigidly enforced) during assembly. */
	AssemblySolver(const Model &model, 
					SimTK::Array_<CoordinateReference> &coordinateReferences,
				   double constraintWeight = SimTK::Infinity);
	
	/** Set the unitless accuracy of the assembly solution, which is dictates to how
	    many significant digits the solution should be resolved to.*/
	void setAccuracy(double accuracy) {_accuracy = accuracy; }

	/** Set the relative weighting for constraints. Use Infinity to identify the 
	    strict enforcement of constraints, otherwise any positive weighting will
		append the constraint errors to the assembly cost which the solver will
		minimize.*/
	void setConstraintWeight(double weight) {_constraintWeight = weight; }
	
	/** Specify which coordinates to match, each with a desired value and a
	    relative weighting. */
	void setCoordinateReferences(Array<CoordinateReference> &coordinateReferences);
	const SimTK::Array_<CoordinateReference>& getCoordinateReferences() const {return _coordinateReferences; };
	/** Once a set of coordinates has been specified its reference value and weight
		can be updated directly */
	void updateCoordinateReference(const std::string &coordName, double value, double weight=1.0);

	/** Assemble a model configuration that meets the assembly conditions  
	    (desired values and constraints) starting from an initial state that  
		does not have to satisfy the constraints. */
	virtual void assemble(SimTK::State &s);

	/** Obtain a model configuration that meets the assembly conditions  
	    (desired values and constraints) given a state that satisfies or
		is close to satisfying the constraints. Note there can be no change
		in the number of constrainst or desired coordinates. Desired
		coordinate values can and should be updated between repeated calls
		to track a desired trajectory of coordinate values. */
	virtual void track(SimTK::State &s);

protected:
	/** Internal method to convert the CoordinateReferences into goals of the 
		assembly solver. Subclasses, can add and override to include other goals  
		such as point of interest matching (Marker tracking). This method is
		automatically called by assemble. */
	virtual void setupGoals(SimTK::State &s);
	/** Internal method to update the time, reference values and/or their 
		weights that define the goals, based on the passed in state. This method
		is called at the end of setupGoals() and beginning of track()*/
	virtual void updateGoals(const SimTK::State &s);

//=============================================================================
};	// END of class AssemblySolver
//=============================================================================
} // namespace

#endif // __AssemblySolver_h__
