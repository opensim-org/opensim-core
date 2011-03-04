/* AssemblySolver.cpp 
* Author: Ajay Seth 
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

#include "AssemblySolver.h"
#include "Model/Model.h"
#include "CoordinateReference.h"
#include <OpenSim/Common/Constant.h>

using namespace std;
using namespace SimTK;

namespace OpenSim {

//______________________________________________________________________________
/**
 * An implementation of the AssemblySolver 
 *
 * @param model to assemble
 */
	AssemblySolver::AssemblySolver(const Model &model, SimTK::Array_<CoordinateReference> &coordinateReferences,
				   double constraintWeight) : Solver(model),
				   _coordinateReferences(coordinateReferences)
{
	_assembler = NULL;
	
	_constraintWeight = constraintWeight;

	// default accuracy
	_accuracy = 1e-4;

	// Get model coordinates
	const CoordinateSet &modelCoordSet = _model.getCoordinateSet();

	SimTK::Array_<CoordinateReference>::iterator p;

	// Cycle through coordinate references
	for(p = _coordinateReferences.begin(); p!=_coordinateReferences.end(); p++) {
		if(p){
			//Find if any references that are empty and throw them away
			if(p->getName() == "" || p->getName() == "unknown"){
				//Get rid of the corresponding reference too
				p = _coordinateReferences.erase(p);
			}
			// Otherwise an error if the coordinate does not exist for this model
			else if ( !modelCoordSet.contains(p->getName())){
				throw(Exception("AssemblySolver: Model does not contain coordinate "+p->getName()+"."));
			}
		}
	}
}

AssemblySolver::~AssemblySolver()
{
	SimTK::Array_<CoordinateReference>::iterator p;
	for(p = _coordinateReferences.begin(); p!=_coordinateReferences.end(); p++)
	{ 
		p = _coordinateReferences.erase(p);
		--p;
	}

	delete _assembler;
}

/** Internal method to convert the CoordinateReferences into goals of the 
		assembly solver. Subclasses, override and call base to include other goals  
		such as point of interest matching (Marker tracking). This method is
		automatically called by assemble. */
void AssemblySolver::setupGoals(SimTK::State &s)
{
	// wipe-out the previous SimTK::Assembler
	delete _assembler;
	_assembler = new SimTK::Assembler(_model.getMultibodySystem());
	_assembler->setAccuracy(_accuracy);

	// Define weights on constraints. Note can be spefified SimTK::Infinity to strictly enforce constraint
	// otherwise the weighted constraint error becomes a goal.
	_assembler->setSystemConstraintsWeight(_constraintWeight);

	// clear any old coordinate goals
	_coordinateAssemblyConditions.clear();

	// Get model coordinates
	const CoordinateSet &modelCoordSet = _model.getCoordinateSet();

	SimTK::Array_<CoordinateReference>::iterator p;

	// Cycle through coordinate references
	for(p = _coordinateReferences.begin(); p!=_coordinateReferences.end(); p++) {
		if(p){
			CoordinateReference *coordRef = p;
			const Coordinate &coord = modelCoordSet.get(coordRef->getName());
			if(coord.getLocked(s)){
				//cout << "AssemblySolver: coordinate " << coord.getName() << " is locked/prescribed and will be excluded." << endl;
				_assembler->lockQ(coord.getBodyIndex(), SimTK::MobilizerQIndex(coord.getMobilityIndex()));
				//No longer need the lock on
				//coord.setLocked(s, false);
				
				//Get rid of the corresponding reference too
				_coordinateReferences.erase(p);
				p--; //decrement since erase automatically points to next in the list
			}
			else{
				// Make this reference and its current value a goal of the Assembler
				SimTK::QValue *coordGoal = new SimTK::QValue(coord.getBodyIndex(), SimTK::MobilizerQIndex(coord.getMobilityIndex()),
														 coordRef->getValue(s) );
				// keep a handle to the goal so we can update
				_coordinateAssemblyConditions.push_back(coordGoal);
				// Add coordinate matching goal to the ik objective
				SimTK::AssemblyConditionIndex acIx = _assembler->adoptAssemblyGoal(coordGoal, coordRef->getWeight(s));
			}
		}
	}

	unsigned int nqrefs = _coordinateReferences.size(), nqgoals = _coordinateAssemblyConditions.size();
	//Should have a one-to-one matched arrays
	if(nqrefs != nqgoals)
		throw Exception("AsemblySolver::setupGoals() has a mismatch between number of references and goals.");
}

/** Once a set of coordinates has been specified its target value can
    be updated directly */
void AssemblySolver::updateCoordinateReference(const std::string &coordName, double value, double weight)
{
	SimTK::Array_<CoordinateReference>::iterator p;

	// Cycle through coordinate references
	for(p = _coordinateReferences.begin(); p!=_coordinateReferences.end(); p++) {
		if(p->getName() == coordName){
			p->setValueFunction(*new Constant(value));
			p->setWeight(weight);
			return;
		}
	}		
}


/** Internal method to update the time, reference values and/or their 
		weights that define the goals, based on the passed in state. */
void AssemblySolver::updateGoals(const SimTK::State &s)
{
	unsigned int nqrefs = _coordinateReferences.size();
	for(unsigned int i=0; i<nqrefs; i++){
		//update goal values from reference.
		_coordinateAssemblyConditions[i]->setValue(_coordinateReferences[i].getValue(s));
		//_assembler->setAssemblyConditionWeight(_coordinateAssemblyConditions[i]->
	}
}

//______________________________________________________________________________
/**
 * Assemble the model such that it satisfies configuration goals and constraints
 * The input state is used to initialize the assembly and then is updated to 
 * return the resulting assembled configuration.
 */
void AssemblySolver::assemble(SimTK::State &state)
{
	// Make a working copy of the state that will be used to set the internal state of the solver
	// This is necessary because we may wish to disable redundant constraints, but do not want this
	// to effect the state of constraints the user expects
	SimTK::State s = state;
	
	// Make sure goals are up-to-date.
	setupGoals(s);

	// Let assembler perform some internal setup
	_assembler->initialize(s);

	/*
	printf("UNASSEMBLED CONFIGURATION (err=%g, cost=%g, qerr=%g)\n",
        _assembler->calcCurrentErrorNorm(), _assembler->calcCurrentGoal(), max(abs(_assembler->getInternalState().getQErr())));
	cout << "Model numQs: " << _model.getNumCoordinates() << " Assembler num freeQs: " << _assembler->getNumFreeQs() << endl;
	*/

	try{
		// Now do the assembly and return the updated state.
		_assembler->assemble();
		// Update the q's in the state passed in
		_assembler->updateFromInternalState(state);

		/*
		printf("ASSEMBLED CONFIGURATION (acc=%g tol=%g err=%g, cost=%g, qerr=%g)\n",
			_assembler->getAccuracyInUse(), _assembler->getErrorToleranceInUse(), 
			_assembler->calcCurrentErrorNorm(), _assembler->calcCurrentGoal(),
			max(abs(_assembler->getInternalState().getQErr())));
		printf("# initializations=%d\n", _assembler->getNumInitializations());
		printf("# assembly steps: %d\n", _assembler->getNumAssemblySteps());
		printf(" evals: goal=%d grad=%d error=%d jac=%d\n",
			_assembler->getNumGoalEvals(), _assembler->getNumGoalGradientEvals(),
			_assembler->getNumErrorEvals(), _assembler->getNumErrorJacobianEvals());
		*/
	}
	catch (std::exception ex)
    {
		std::string msg = "AssemblySolver::assemble() Failed: ";
		msg += ex.what();
		throw Exception(msg);
    }
}

/** Obtain a model configuration that meets the assembly conditions  
    (desired values and constraints) given a state that satisfies or
	is close to satisfying the constraints. Note there can be no change
	in the number of constrainst or desired coordinates. Desired
	coordinate values can and should be updated between repeated calls
	to track a desired trajectory of coordinate values. */
void AssemblySolver::track(SimTK::State &s)
{

	// move the target locations or angles, etc... just do not change number of goals
	// and their type (constrained vs. weighted)

	if(_assembler && _assembler->isInitialized()){
		updateGoals(s);
	}
	else{
		throw Exception("AssemblySolver::track() failed: assemble() must be called first.");
	}

	try{
		// Now do the assembly and return the updated state.
		_assembler->track(s.getTime());

		// update the state from the result of the assembler 
		_assembler->updateFromInternalState(s);

		/*
		printf("Tracking: t= %f (acc=%g tol=%g err=%g, cost=%g, qerr=%g)\n", s.getTime(),
			_assembler->getAccuracyInUse(), _assembler->getErrorToleranceInUse(), 
			_assembler->calcCurrentErrorNorm(), _assembler->calcCurrentGoal(),
			max(abs(_assembler->getInternalState().getQErr())));
		*/
	}
	catch (SimTK::Exception::Base ex)
    {
		std::cout << "AssemblySolver::track() attempt Failed: " << ex.what() << std::endl;
		throw Exception("AssemblySolver::track() attempt failed.");
    }
}

} // end of namespace OpenSim