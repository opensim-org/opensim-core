// Constraint.cpp
// Author: Frank C. Anderson, Ajay Seth
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "Constraint.h"
#include <OpenSim/Simulation/Model/Model.h>

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
	setupProperties();
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
 * Copy constructor.
 *
 * @param aConstraint Constraint to be copied.
 */
Constraint::Constraint(const Constraint &aConstraint) :
   ModelComponent(aConstraint)
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Constraint to another.
 *
 * @param aConstraint Constraint to be copied.
 */
void Constraint::copyData(const Constraint &aConstraint)
{
	//_isDisabled = aConstraint._isDisabled;
	_isDisabledProp.setValue(aConstraint._isDisabledProp.getValueBool());
	_model = aConstraint._model;
	// A copy is no longer a live Constraint with an underlying SimTK::Constraint
	// The system must be created, at which time the constraint will be assigned an index
	// corresponding to a valid system SimTK::Constraint.
	_index.invalidate();
}

//_____________________________________________________________________________
/**
 * Set the data members of this Constraint to their null values.
 */
void Constraint::setNull(void)
{
	setType("Constraint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Constraint::setupProperties(void)
{
	_isDisabledProp.setName("isDisabled");
	_isDisabledProp.setValue(false);
	_propertySet.append(&_isDisabledProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this Constraint.
 */
void Constraint::setup(Model& aModel)
{
	ModelComponent::setup(aModel);
}

void Constraint::initState(SimTK::State& s) const
{
	SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);

	// Otherwise we have to change the status of the constraint
	if(_isDisabledProp.getValueBool())
		simConstraint.disable(s);
	else
		simConstraint.enable(s);
}

void Constraint::setDefaultsFromState(const SimTK::State& state)
{
    _isDisabledProp.setValue(isDisabled(state));
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Constraint& Constraint::operator=(const Constraint &aConstraint)
{
	// BASE CLASS
	Object::operator=(aConstraint);

	copyData(aConstraint);

	return(*this);
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

	_isDisabledProp.setValue(isDisabled);
	
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
 *				a SpatialVec (6 components) describing resulting torque and force
 *			mobilityForces acting along constrained mobilities	
 *
 * @param state State of model
 * @param bodyForcesInParent is a Vector of SpatialVecs contain constraint forces
 * @param mobilityForces is a Vector of forces that act along the constrained
 *         mobilitities associated with this constraint
 */
void Constraint::calcConstraintForces(const SimTK::State& s, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInParent, 
									  SimTK::Vector& mobilityForces)
{
	SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);
	simConstraint.calcConstraintForcesFromMultipliers( s, simConstraint.getMultipliersAsVector(s), 
		                                               bodyForcesInParent, mobilityForces);
}

/** 
 * Methods to query a Constraint forces for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
Array<std::string> Constraint::getRecordLabels() const
{
	SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_index);
	const SimTK::State &ds = _model->getMultibodySystem().getDefaultState();

	// number or multipliers of each stage (position, velocity, acceleration)
	int mp, mv, ma, nlambda;
	simConstraint.getNumConstraintEquationsInUse(ds, mp, mv, ma);

	nlambda = mp + mv + ma;

	Array<std::string> labels("");
	
	for(int i=0; i<nlambda; i++){
		char c[2];
//		itoa(i,c,10); not supported by g++
		sprintf(c, "%d", i); 
		labels.append(getName()+"_multiplier"+c);
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
	SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(state);

	Array<double> values(0.0, multipliers.size());
	
	for(int i=0; i<multipliers.size(); i++){
		values[i]=multipliers[i];
	}

	return values;
};
