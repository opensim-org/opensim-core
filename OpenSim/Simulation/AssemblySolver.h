#ifndef OPENSIM_ASSEMBLY_SOLVER_H_
#define OPENSIM_ASSEMBLY_SOLVER_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  AssemblySolver.h                         *
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

#include "Solver.h"
#include "OpenSim/Simulation/CoordinateReference.h"
#include "simbody/internal/Assembler.h"

namespace SimTK { 
class QValue;
class State;
}

namespace OpenSim {

class Model;

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
 * the small change to the desired coordinate value.
 *
 * See SimTK::Assembler for more algorithmic details of the underlying solver.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API AssemblySolver: public Solver {
OpenSim_DECLARE_CONCRETE_OBJECT(AssemblySolver, Solver);
//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------

    /** Construct an Assembly solver with the coordinate references as the goal
        of the assembly and (optional)constraint weight. Default is infinity
        constraint weighting (i.e. rigidly enforced) during assembly. */
    AssemblySolver(const Model &model, 
                   const SimTK::Array_<CoordinateReference> &coordinateReferences,
                   double constraintWeight = SimTK::Infinity);

    virtual ~AssemblySolver() {}

    /** %Set the unitless accuracy of the assembly solution, which dictates
        the number of significant digits the solution should be resolved to.
        Note, setting the accuracy will invalidate the AssemblySolver and one
        must call assemble() before being able to track().*/
    void setAccuracy(double accuracy);

    /** %Set the relative weighting for constraints. Use Infinity to identify the 
        strict enforcement of constraints, otherwise any positive weighting will
        append the constraint errors to the assembly cost which the solver will
        minimize.*/
    void setConstraintWeight(double weight) {_constraintWeight = weight; }
    
    /** Specify which coordinates to match, each with a desired value and a
        relative weighting. */
    const SimTK::Array_<CoordinateReference>& getCoordinateReferences() const
    {   return _coordinateReferencesp; };
    /** Once a set of coordinates has been specified its reference value and 
        weight can be updated directly */
    void updateCoordinateReference(const std::string &coordName, double value, 
                                   double weight=1.0);

    /** Assemble a model configuration that meets the assembly conditions 
        (desired values and constraints) and accuracy, starting from an initial
        state that does not have to satisfy the constraints. */
    virtual void assemble(SimTK::State &s);

    /** Obtain a model configuration that meets the assembly conditions 
        (desired values and constraints) given a state that satisfies or
        is close to satisfying the constraints. Note there can be no change
        in the number of constraints or desired coordinates. Desired
        coordinate values can and should be updated between repeated calls
        to track a desired trajectory of coordinate values. Use assemble()
        first to obtain the first solution and use track() to efficiently
        find a nearby solution due to a small change in the desired value.*/
    virtual void track(SimTK::State &s);

    /** Read access to the underlying SimTK::Assembler. */
    const SimTK::Assembler& getAssembler() const;

protected:
    /** Internal method to convert the CoordinateReferences into goals of the 
        assembly solver. Subclasses, can add and override to include other goals  
        such as point of interest matching (Marker tracking). This method is
        automatically called by assemble. */
    virtual void setupGoals(SimTK::State &s);
    /** Internal method to update the time, reference values and/or their 
        weights that define the goals, based on the passed in state. This method
        is called at the end of setupGoals() and beginning of track()*/
    virtual void updateGoals(SimTK::State &s);

    /** Write access to the underlying SimTK::Assembler. */
    SimTK::Assembler& updAssembler();

private:

    // The assembly solution accuracy
    double _accuracy;

    // Weight for built-in constraints to be satisfied
    double _constraintWeight;

    // The coordinates reference value and weighting. This is full copy rather than ref.
    SimTK::Array_<CoordinateReference> _coordinateReferencesp;

    // Underlying SimTK::Assembler that will perform the assembly
    SimTK::ResetOnCopy< std::unique_ptr<SimTK::Assembler>> _assembler;

    SimTK::Array_<SimTK::QValue*> _coordinateAssemblyConditions;
//=============================================================================
};  // END of class AssemblySolver
//=============================================================================
} // namespace

#endif // OPENSIM_ASSEMBLY_SOLVER_H_
