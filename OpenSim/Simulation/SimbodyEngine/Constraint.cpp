/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Constraint.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Constraint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
//using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Constraint::Constraint() 
{
    setNull();
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Constraint::~Constraint()
{
}

//_____________________________________________________________________________
/**
 * Set the data members of this Constraint to their null values.
 */
void Constraint::setNull(void)
{
    setAuthors("Frank Anderson, Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Constraint::constructProperties(void)
{
    constructProperty_isDisabled(false);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Constraint.
 */
void Constraint::extendConnectToModel(Model& aModel)
{
    Super::extendConnectToModel(aModel);
}

void Constraint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);
    SimTK::Constraint& simConstraint = 
        _model->updMatterSubsystem().updConstraint(_index);

    // Otherwise we have to change the status of the constraint
    if(get_isDisabled())
        simConstraint.disable(s);
    else
        simConstraint.enable(s);
}

void Constraint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);
    set_isDisabled(isDisabled(state));
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update an existing Constraint with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aConstraint Constraint to update from
 */
void Constraint::updateFromConstraint(SimTK::State& s, const Constraint &aConstraint)
{
    setDisabled(s, aConstraint.isDisabled(s));
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// DISABLE
//-----------------------------------------------------------------------------

//_____________________________________________________________________________
/**
 * Get whether or not this Constraint is disabled.
 * Simbody multibody system instance is realized every time the isDisabled
 * changes, BUT multiple sets to the same value have no cost.
 *
 * @param isDisabled If true the constraint is disabled; if false the constraint is enabled.
 */
bool Constraint::isDisabled(const SimTK::State& s) const
{
    return _model->updMatterSubsystem().updConstraint(_index).isDisabled(s);
}

//_____________________________________________________________________________
/**
 * Set whether or not this Constraint is disabled.
 * Simbody multibody system instance is realized every time the isDisabled
 * changes, BUT multiple sets to the same value have no cost.
 *
 * @param isDisabled If true the constraint is disabled; if false the constraint is enabled.
 */
bool Constraint::setDisabled(SimTK::State& s, bool isDisabled)
{
    SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);
    bool modelConstraintIsDisabled = simConstraint.isDisabled(s);

    // Check if we already have the correct enabling of the constraint then do nothing 
    if(isDisabled == modelConstraintIsDisabled)
        return true;

    // Otherwise we have to change the status of the constraint
    if(isDisabled)
        simConstraint.disable(s);
    else
        simConstraint.enable(s);

    _model->updateAssemblyConditions(s);
    set_isDisabled(isDisabled);
    
    return true;
}


//-----------------------------------------------------------------------------
// FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Ask the constraint for the forces it is imposing on the system
 * Simbody multibody system must be realized to at least position
 * Returns: the bodyForces on those bodies being constrained (constrainedBodies)
 *              a SpatialVec (6 components) describing resulting torque and force
 *          mobilityForces acting along constrained mobilities  
 *
 * @param state State of model
 * @param bodyForcesInAncestor is a Vector of SpatialVecs contain constraint forces
 * @param mobilityForces is a Vector of forces that act along the constrained
 *         mobilitities associated with this constraint
 */
void Constraint::calcConstraintForces(const SimTK::State& s, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
                                      SimTK::Vector& mobilityForces) const
{
    SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);
    if(!simConstraint.isDisabled(s)){
        SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(s);
        simConstraint.calcConstraintForcesFromMultipliers(s, multipliers, bodyForcesInAncestor, mobilityForces);
    }
}

/** 
 * Methods to query a Constraint forces for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
Array<std::string> Constraint::getRecordLabels() const
{
    SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);
    const SimTK::State &ds = _model->getWorkingState();

    // number of bodies being directly constrained
    int ncb = simConstraint.getNumConstrainedBodies();
    // number of mobilities being directly constrained
    int ncm = simConstraint.getNumConstrainedU(ds);

    const BodySet &bodies = _model->getBodySet();
    
    Array<std::string> labels("");

    for(int i=0; i<ncb; ++i){
        const SimTK::MobilizedBody &b = simConstraint.getMobilizedBodyFromConstrainedBody(SimTK::ConstrainedBodyIndex(i));
        const SimTK::MobilizedBodyIndex &bx =  b.getMobilizedBodyIndex();
        Body *bod = NULL;
        for(int j=0; j<bodies.getSize(); ++j ){
            if(bodies[j].getMobilizedBodyIndex() == bx){
                bod = &bodies[j];
                break;
            }
        }
        if(bod == NULL){
            throw Exception("Constraint "+getName()+" does not have an idenitfiable body index.");
        }
        string prefix = getName()+"_"+bod->getName();
        labels.append(prefix+"_Fx");
        labels.append(prefix+"_Fy");
        labels.append(prefix+"_Fz");
        labels.append(prefix+"_Mx");
        labels.append(prefix+"_My");
        labels.append(prefix+"_Mz");
    }

    char c[2] = "";
    for(int i=0; i<ncm; ++i){
        sprintf(c, "%d", i);
        labels.append(getName()+"_mobility_F"+c);
    }
    
    return labels;
}

/**
 * Given SimTK::State object extract all the values necessary to report constraint forces, application 
 * location frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
Array<double> Constraint::getRecordValues(const SimTK::State& state) const
{
    // EOMs are solved for accelerations (udots) and constraint multipliers (lambdas)
    // simulataneously, so system must be realized to acceleration
    _model->getMultibodySystem().realize(state, SimTK::Stage::Acceleration);
    SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);

    // number of bodies being directly constrained
    int ncb = simConstraint.getNumConstrainedBodies();
    // number of mobilities being directly constrained
    int ncm = simConstraint.getNumConstrainedU(state);

    SimTK::Vector_<SimTK::SpatialVec> bodyForcesInAncestor(ncb);
    bodyForcesInAncestor.setToZero();
    SimTK::Vector mobilityForces(ncm, 0.0);

    Array<double> values(0.0,6*ncb+ncm);

    calcConstraintForces(state, bodyForcesInAncestor, mobilityForces);
    
    for(int i=0; i<ncb; ++i){
        for(int j=0; j<3; ++j){
            // Simbody constraints have reaction moments first and OpenSim reports forces first
            // so swap them here
            values[i*6+j] = (bodyForcesInAncestor(i)[1])[j]; // moments on constrained body i
            values[i*6+3+j] = (bodyForcesInAncestor(i)[0])[j]; // forces on constrained body i
        }
    }
    for(int i=0; i<ncm; ++i){
        values[6*ncb+i] = mobilityForces[i];
    }

    return values;
};
