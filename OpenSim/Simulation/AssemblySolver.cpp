/* -------------------------------------------------------------------------- *
 *                        OpenSim:  AssemblySolver.cpp                        *
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

#include "AssemblySolver.h"
#include "OpenSim/Simulation/Model/Model.h"
#include <OpenSim/Common/Constant.h>
#include "simbody/internal/AssemblyCondition_QValue.h"

using namespace std;
using namespace SimTK;

namespace OpenSim {

class Coordinate;
class CoordinateSet;

//______________________________________________________________________________
AssemblySolver::AssemblySolver
   (const Model &model, const SimTK::Array_<CoordinateReference> &coordinateReferences,
    double constraintWeight) : Solver(model),
    _coordinateReferencesp(coordinateReferences)
{
    setAuthors("Ajay Seth");
    _assembler = NULL;
    
    _constraintWeight = constraintWeight;

    // default accuracy
    _accuracy = 1e-4;

    // Get model coordinates
    const CoordinateSet& modelCoordSet = getModel().getCoordinateSet();

    SimTK::Array_<CoordinateReference>::iterator p;

    // Cycle through coordinate references
    for(p = _coordinateReferencesp.begin(); 
        p != _coordinateReferencesp.end(); p++) 
    {
        if(p){
            //Find if any references that are empty and throw them away
            if(p->getName() == "" || p->getName() == "unknown"){
                //Get rid of the corresponding reference too
                p = _coordinateReferencesp.erase(p);
            }
            // Otherwise an error if the coordinate does not exist for this model
            else if ( !modelCoordSet.contains(p->getName())){
                throw(Exception("AssemblySolver: Model does not contain coordinate "+p->getName()+"."));
            }
        }
    }
}

void AssemblySolver::setAccuracy(double accuracy)
{
    _accuracy = accuracy;
    // Changing the accuracy invalidates the existing SimTK::Assembler
    _assembler.reset();
}

/* Internal method to convert the CoordinateReferences into goals of the 
   assembly solver. Subclasses, override and call base to include other goals  
   such as point of interest matching (Marker tracking). This method is
   automatically called by assemble. */
void AssemblySolver::setupGoals(SimTK::State &s)
{
    // wipe-out the previous SimTK::Assembler
    _assembler.reset(new SimTK::Assembler(getModel().getMultibodySystem()));
    _assembler->setAccuracy(_accuracy);

    // Define weights on constraints. Note can be specified SimTK::Infinity to strictly enforce constraint
    // otherwise the weighted constraint error becomes a goal.
    _assembler->setSystemConstraintsWeight(_constraintWeight);

    // clear any old coordinate goals
    _coordinateAssemblyConditions.clear();

    // Get model coordinates
    const CoordinateSet& modelCoordSet = getModel().getCoordinateSet();

    // Restrict solution to set range of any of the coordinates that are clamped
    for(int i=0; i<modelCoordSet.getSize(); ++i){
        const Coordinate& coord = modelCoordSet[i];
        if(coord.getClamped(s)){
            _assembler->restrictQ(coord.getBodyIndex(), 
                MobilizerQIndex(coord.getMobilizerQIndex()),
                coord.getRangeMin(), coord.getRangeMax());
        }
    }

    SimTK::Array_<CoordinateReference>::iterator p;

    // Cycle through coordinate references
    for(p = _coordinateReferencesp.begin(); 
        p != _coordinateReferencesp.end(); p++) {
        if(p){
            CoordinateReference *coordRef = p;
            const Coordinate &coord = modelCoordSet.get(coordRef->getName());
            if(coord.getLocked(s)){
                _assembler->lockQ(coord.getBodyIndex(), SimTK::MobilizerQIndex(coord.getMobilizerQIndex()));
                //No longer need the lock on
                coord.setLocked(s, false);
                
                //Get rid of the corresponding reference too
                _coordinateReferencesp.erase(p);
                p--; //decrement since erase automatically points to next in the list
            }
            else if(!(coord.get_is_free_to_satisfy_constraints())) {
                // Make this reference and its current value a goal of the Assembler
                SimTK::QValue *coordGoal = new SimTK::QValue(coord.getBodyIndex(), SimTK::MobilizerQIndex(coord.getMobilizerQIndex()),
                                                         coordRef->getValue(s) );
                // keep a handle to the goal so we can update
                _coordinateAssemblyConditions.push_back(coordGoal);
                // Add coordinate matching goal to the ik objective
                _assembler->adoptAssemblyGoal(coordGoal, coordRef->getWeight(s));
            }
        }
    }

    unsigned int nqrefs  = _coordinateReferencesp.size(), 
                 nqgoals = _coordinateAssemblyConditions.size();
    //Should have a one-to-one matched arrays
    if(nqrefs != nqgoals)
        throw Exception("AsemblySolver::setupGoals() has a mismatch between number of references and goals.");
}

/* Once a set of coordinates has been specified its target value can
    be updated directly */
void AssemblySolver::updateCoordinateReference(const std::string &coordName, double value, double weight)
{
    SimTK::Array_<CoordinateReference>::iterator p;

    // Cycle through coordinate references
    for(p = _coordinateReferencesp.begin(); 
        p != _coordinateReferencesp.end(); p++) {
        if(p->getName() == coordName){
            p->setValueFunction(Constant(value));
            p->setWeight(weight);
            return;
        }
    }       
}


/* Internal method to update the time, reference values and/or their 
        weights that define the goals, based on the passed in state. */
void AssemblySolver::updateGoals(SimTK::State &s)
{
    unsigned int nqrefs = _coordinateReferencesp.size();
    for(unsigned int i=0; i<nqrefs; i++){
        //update goal values from reference.
        _coordinateAssemblyConditions[i]->setValue
           ((_coordinateReferencesp)[i].getValue(s));
        //_assembler->setAssemblyConditionWeight(_coordinateAssemblyConditions[i]->
    }
}

//______________________________________________________________________________
/*
 * Assemble the model such that it satisfies configuration goals and constraints
 * The input state is used to initialize the assembly and then is updated to 
 * return the resulting assembled configuration.
 */
void AssemblySolver::assemble(SimTK::State &state)
{
    // Make a working copy of the state that will be used to set the internal 
    // state of the solver. This is necessary because we may wish to disable 
    // redundant constraints, but do not want this to affect the state of 
    // constraints the user expects
    SimTK::State s = state;
    
    // Make sure goals are up-to-date.
    setupGoals(s);

    // Let assembler perform some internal setup
    _assembler->initialize(s);
    
    // Useful to include through debug message/log in the future
    log_debug("UNASSEMBLED CONFIGURATION (normerr={}, maxerr={}, cost={})",
        _assembler->calcCurrentErrorNorm(),
        max(abs(_assembler->getInternalState().getQErr())),
        _assembler->calcCurrentGoal());
    log_debug("Model numQs: {} Assembler num freeQs: {}",  
        _assembler->getInternalState().getNQ(),  _assembler->getNumFreeQs());

    try{
        // Now do the assembly and return the updated state.
        _assembler->assemble();
        // Update the q's in the state passed in
        _assembler->updateFromInternalState(s);
        state.updQ() = s.getQ();
        state.updU() = s.getU();

        // Get model coordinates
        const CoordinateSet& modelCoordSet = getModel().getCoordinateSet();
        // Make sure the locks in original state are restored
        for(int i=0; i< modelCoordSet.getSize(); ++i){
            bool isLocked = modelCoordSet[i].getLocked(state);
            if(isLocked)
                modelCoordSet[i].setLocked(state, isLocked);
        }
        // TODO: Useful to include through debug message/log in the future
        log_debug("ASSEMBLED CONFIGURATION (acc={} tol={} normerr={}, maxerr={}, cost={})",
            _assembler->getAccuracyInUse(), _assembler->getErrorToleranceInUse(), 
            _assembler->calcCurrentErrorNorm(), max(abs(_assembler->getInternalState().getQErr())),
            _assembler->calcCurrentGoal());
        log_debug("# initializations={}", _assembler->getNumInitializations());
        log_debug("# assembly steps: {}", _assembler->getNumAssemblySteps());
        log_debug(" evals: goal={} grad={} error={} jac={}",
            _assembler->getNumGoalEvals(), _assembler->getNumGoalGradientEvals(),
            _assembler->getNumErrorEvals(), _assembler->getNumErrorJacobianEvals());
    }
    catch (const std::exception& ex)
    {
        std::string msg = "AssemblySolver::assemble() Failed: ";
        msg += ex.what();
        throw Exception(msg);
    }
}

/* Obtain a model configuration that meets the assembly conditions  
    (desired values and constraints) given a state that satisfies or
    is close to satisfying the constraints. Note there can be no change
    in the number of constraints or desired coordinates. Desired
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
        throw Exception(
            "AssemblySolver::track() failed: assemble() must be called first.");
    }

    // TODO: Useful to include through debug message/log in the future
    log_debug("UNASSEMBLED(track) CONFIGURATION (normerr={}, maxerr={}, cost={})",
        _assembler->calcCurrentErrorNorm(), 
        max(abs(_assembler->getInternalState().getQErr())), 
        _assembler->calcCurrentGoal() );
    log_debug("Model numQs: {}  Assembler num freeQs: {}",
        _assembler->getInternalState().getNQ(), _assembler->getNumFreeQs());

    try{
        // Now do the assembly and return the updated state.
        _assembler->track(s.getTime());

        // update the state from the result of the assembler 
        _assembler->updateFromInternalState(s);
        
        // TODO: Useful to include through debug message/log in the future
        log_debug("Tracking: t= {} (acc={} tol={} normerr={}, maxerr={}, cost={})", 
            s.getTime(),
            _assembler->getAccuracyInUse(), _assembler->getErrorToleranceInUse(), 
            _assembler->calcCurrentErrorNorm(), max(abs(_assembler->getInternalState().getQErr())),
            _assembler->calcCurrentGoal()); 
    }
    catch (const std::exception& ex)
    {
        log_info( "AssemblySolver::track() attempt Failed: {}", ex.what());
        throw Exception("AssemblySolver::track() attempt failed.");
    }
}

const SimTK::Assembler& AssemblySolver::getAssembler() const
{
    OPENSIM_THROW_IF(!_assembler, Exception,
        "AssemblySolver::getAssembler() has no underlying Assembler to return.\n"
        "AssemblySolver::setupGoals() must be called first.");
    return *_assembler;
}

SimTK::Assembler& AssemblySolver::updAssembler()
{
    OPENSIM_THROW_IF(!_assembler, Exception,
        "AssemblySolver::updAssembler() has no underlying Assembler to return.\n"
        "AssemblySolver::setupGoals() must be called first.");
    return *_assembler;
}


} // end of namespace OpenSim
