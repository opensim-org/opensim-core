/* -------------------------------------------------------------------------- *
 *                  OpenSim:  RollingOnSurfaceConstraint.cpp                  *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <math.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "RollingOnSurfaceConstraint.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
RollingOnSurfaceConstraint::~RollingOnSurfaceConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
RollingOnSurfaceConstraint::RollingOnSurfaceConstraint() :
	UnilateralConstraint(),
	_rollingBodyName(_rollingBodyNameProp.getValueStr()),
	_surfaceBodyName(_surfaceBodyNameProp.getValueStr()),
	_surfaceNormal(_surfaceNormalProp.getValueDblVec()),
	_surfaceHeight(_surfaceHeightProp.getValueDbl()),
	_coulombFrictionCoefficient(_coulombFrictionCoefficientProp.getValueDbl()),
	_surfaceContactRadius(_surfaceContactRadiusProp.getValueDbl())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint RollingOnSurfaceConstraint to be copied.
 */
RollingOnSurfaceConstraint::RollingOnSurfaceConstraint(const RollingOnSurfaceConstraint &aConstraint) :
   UnilateralConstraint(aConstraint),
	_rollingBodyName(_rollingBodyNameProp.getValueStr()),
	_surfaceBodyName(_surfaceBodyNameProp.getValueStr()),
	_surfaceNormal(_surfaceNormalProp.getValueDblVec()),
	_surfaceHeight(_surfaceHeightProp.getValueDbl()),
	_coulombFrictionCoefficient(_coulombFrictionCoefficientProp.getValueDbl()),
	_surfaceContactRadius(_surfaceContactRadiusProp.getValueDbl())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Copy data members from one RollingOnSurfaceConstraint to another.
 *
 * @param aConstraint RollingOnSurfaceConstraint to be copied.
 */
void RollingOnSurfaceConstraint::copyData(const RollingOnSurfaceConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);

	_rollingBodyName = aConstraint._rollingBodyName;
	_surfaceBodyName = aConstraint._surfaceBodyName;
	_surfaceNormal = aConstraint._surfaceNormal;
	_surfaceHeight = aConstraint._surfaceHeight;
	_coulombFrictionCoefficient = aConstraint._coulombFrictionCoefficient;
	_surfaceContactRadius = aConstraint._surfaceContactRadius;
	_defaultUnilateralConditions = aConstraint._defaultUnilateralConditions;
}

//_____________________________________________________________________________
/**
 * Set the data members of this RollingOnSurfaceConstraint to their null values.
 */
void RollingOnSurfaceConstraint::setNull()
{
	_defaultUnilateralConditions = std::vector<bool>(4, false);
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void RollingOnSurfaceConstraint::setupProperties()
{
	// Body 1 name
	_rollingBodyNameProp.setName("rolling_body");
	_propertySet.append(&_rollingBodyNameProp);

	// Body 2 name
	_surfaceBodyNameProp.setName("surface_body");
	_surfaceBodyNameProp.setValue("ground");
	_propertySet.append(&_surfaceBodyNameProp);

	// Surface parameters
	_surfaceNormalProp.setName("surface_normal");
	_surfaceNormalProp.setValue(Vec3(0,1.0,0));
	_propertySet.append(&_surfaceNormalProp);

	_surfaceHeightProp.setName("surface_height");
	_surfaceHeightProp.setValue(0.0);
	_propertySet.append(&_surfaceNormalProp);
	
	_coulombFrictionCoefficientProp.setName("friction_coefficient");
	_coulombFrictionCoefficientProp.setValue(0.5);
	_propertySet.append(&_coulombFrictionCoefficientProp);

	_surfaceContactRadiusProp.setName("contact_radius");
	_surfaceContactRadiusProp.setValue(0.01);
	_propertySet.append(&_surfaceContactRadiusProp);
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this RollingOnSurfaceConstraint.
 */
void RollingOnSurfaceConstraint::connectToModel(Model& aModel)
{
	Super::connectToModel(aModel);

	string errorMessage;

		// Look up the two bodies being constrained together by name in the
	// dynamics engine and might as well keep a pointer to them
	_rollingBody = &_model->updBodySet().get(_rollingBodyName);
	_surfaceBody = &_model->updBodySet().get(_surfaceBodyName);

	if (!_rollingBody) {
		errorMessage = "Invalid RollingBody (" + _rollingBodyName + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!_surfaceBody) {
		errorMessage = "Invalid SurfaceBody (" + _surfaceBodyName + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
}

void RollingOnSurfaceConstraint::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody roller = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_rollingBody->getIndex());
	SimTK::MobilizedBody surface = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_surfaceBody->getIndex());
	
	// Add a ficticious massless body to be the "Case" reference body coincident with surface for the no-slip constraint
	SimTK::MobilizedBody::Weld  cb(surface, SimTK::Body::Massless());

	// Constrain the roller to the surface
	SimTK::Constraint::PointInPlane contactY(surface, SimTK::UnitVec3(_surfaceNormal), _surfaceHeight, roller,Vec3(0));
	SimTK::Constraint::ConstantAngle contactTorqueAboutY(surface, SimTK::UnitVec3(1,0,0), roller, SimTK::UnitVec3(0,0,1));
	// Constrain the roller to roll on surface and not slide 
	SimTK::Constraint::NoSlip1D contactPointXdir(cb, SimTK::Vec3(0), SimTK::UnitVec3(1,0,0), surface, roller);
	SimTK::Constraint::NoSlip1D contactPointZdir(cb, SimTK::Vec3(0), SimTK::UnitVec3(0,0,1), surface, roller);

	 // Beyond the const Component get the index so we can access the SimTK::Constraint later
	RollingOnSurfaceConstraint* mutableThis = const_cast<RollingOnSurfaceConstraint *>(this);
	// Make sure that there is nothing in the list of constraint indices
	mutableThis->_indices.clear();
	// Get the index so we can access the SimTK::Constraint later
	mutableThis->_indices.push_back(contactY.getConstraintIndex());
	mutableThis->_indices.push_back(contactTorqueAboutY.getConstraintIndex());
	mutableThis->_indices.push_back(contactPointXdir.getConstraintIndex());
	mutableThis->_indices.push_back(contactPointZdir.getConstraintIndex());

	mutableThis->_numConstraintEquations = _indices.size();

	// For compound constraints, the bodies and/or mobilities involved must be characterized by
	// the first "master" constraint, which dictates the behavior of the other constraints
	// For example, enabling and disabling. This enables a compound constraint to be treated
	// like a single constraint for the purpose of enabling/disabling and getting output
	mutableThis->_index = _indices[0];
}

void RollingOnSurfaceConstraint::initStateFromProperties(SimTK::State& state) const
{
    Super::initStateFromProperties(state);

	// All constraints treated the same as default behavior at initilization
	for(int i=0; i < _numConstraintEquations; i++){
		SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
		// initialize the status of the constraint
		if(_defaultUnilateralConditions[i]){
			simConstraint.enable(state);
		}
		else{
			simConstraint.disable(state);
		}
	}
}

void RollingOnSurfaceConstraint::setPropertiesFromState(const SimTK::State& state)
{
    Super::setPropertiesFromState(state);

    _isDisabledProp.setValue(isDisabled(state));
	for(int i=0; i < _numConstraintEquations; i++){
		SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
		// initialize the status of the constraint
		_defaultUnilateralConditions[i] = !simConstraint.isDisabled(state); 
	}
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
RollingOnSurfaceConstraint& RollingOnSurfaceConstraint::operator=(const RollingOnSurfaceConstraint &aConstraint)
{
	UnilateralConstraint::operator=(aConstraint);
	copyData(aConstraint);
	return(*this);
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the weld constraint */
void RollingOnSurfaceConstraint::setRollingBodyByName(std::string aBodyName)
{
	_rollingBodyName = aBodyName;
}

void RollingOnSurfaceConstraint::setSurfaceBodyByName(std::string aBodyName)
{
	_surfaceBodyName = aBodyName;
}

/** Set the point of contact on the rolling body that will be in contact with the surface */
void RollingOnSurfaceConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
	// Get the individual underlying constraints
	SimTK::Constraint::PointInPlane &contactY = (SimTK::Constraint::PointInPlane &)_model->updMatterSubsystem().updConstraint(_indices[0]);
	SimTK::Constraint::ConstantAngle &contactTorqueAboutY = (SimTK::Constraint::ConstantAngle &)_model->updMatterSubsystem().updConstraint(_indices[1]);
	SimTK::Constraint::NoSlip1D &contactPointXdir = (SimTK::Constraint::NoSlip1D &)_model->updMatterSubsystem().updConstraint(_indices[2]);
	SimTK::Constraint::NoSlip1D &contactPointZdir = (SimTK::Constraint::NoSlip1D &)_model->updMatterSubsystem().updConstraint(_indices[3]);

	// The contact point coordinates in the surface body frame 
	Vec3 spoint;

	// make sure we are at the position stage
	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);

	// For external forces we assume w.r.t. ground
	_model->getSimbodyEngine().transformPosition(s, *_rollingBody, point, *_surfaceBody, spoint);

	// The contact point coordinates in the surface body frame 
	contactY.setDefaultPlaneNormal(UnitVec3(_surfaceNormal));
	contactY.setDefaultPlaneHeight(~spoint*_surfaceNormal);
	// And the point in the follower (roller) frame
	contactY.setDefaultFollowerPoint(point);

	// Assume that torsion constraint is always normal to surface so find corresponding vector
	// in the rolling body
	Vec3 normalInRoller(0);
	Vec3 baseAxisInRoller(0);
	UnitVec3 surfaceBase = contactTorqueAboutY.getDefaultBaseAxis();
	_model->getSimbodyEngine().transform(s, *_surfaceBody, _surfaceNormal, *_rollingBody, normalInRoller);
	_model->getSimbodyEngine().transform(s, *_surfaceBody, surfaceBase, *_rollingBody, baseAxisInRoller);

	contactTorqueAboutY.setDefaultFollowerAxis(UnitVec3(baseAxisInRoller%normalInRoller));

	// Set the point of no slip for this instant
	contactPointXdir.setDefaultContactPoint(spoint);
	contactPointZdir.setDefaultContactPoint(spoint);
}

std::vector<bool> RollingOnSurfaceConstraint::unilateralConditionsSatisfied(const SimTK::State &state)
{
	std::vector<bool> conditionsSatisfied(4,false);
	int mp, mv, ma;
	SimTK::Vector lambda;

	// The reaction forces necessary for resolving the unilateral conditions
	double normalForce = 0;
	double normalTorque = 0;
	SimTK::Vec3 rollForce(0);
	double tangentialForce = 0;

	// Get the individual underlying constraints
	SimTK::Constraint&contactY = _model->updMatterSubsystem().updConstraint(_indices[0]);
	SimTK::Constraint&contactTorqueAboutY = _model->updMatterSubsystem().updConstraint( _indices[1]);
	SimTK::Constraint&contactPointXdir = _model->updMatterSubsystem().updConstraint(_indices[2]);
	SimTK::Constraint&contactPointZdir = _model->updMatterSubsystem().updConstraint(_indices[3]);

	// Constraint conditions only matter if the constraint is enabled.
	if(!contactY.isDisabled(state)){
		// Obtain the surface constraint force in the normal direction.
		contactY.getNumConstraintEquationsInUse(state, mp, mv, ma);
		lambda = contactY.getMultipliersAsVector(state);
		normalForce = -lambda[0];

		// Obtain the surface constraint torque about the surface normal.
		if(!contactTorqueAboutY.isDisabled(state)){
			contactTorqueAboutY.getNumConstraintEquationsInUse(state, mp, mv, ma);
			lambda = contactTorqueAboutY.getMultipliersAsVector(state);
			normalTorque = lambda[0];
		}

		// Obtain the rolling force components.
		if(!contactPointXdir.isDisabled(state)){
			contactPointXdir.getNumConstraintEquationsInUse(state, mp, mv, ma);
			lambda = contactPointXdir.getMultipliersAsVector(state);
			rollForce[0] = lambda[0];
		}
		if(!contactPointZdir.isDisabled(state)){
			contactPointZdir.getNumConstraintEquationsInUse(state, mp, mv, ma);
			lambda = contactPointZdir.getMultipliersAsVector(state);
			rollForce[2] = lambda[0];
		}
	}

	// get the magnitude of the rolling force 
	tangentialForce = rollForce.norm();
	
	// All internal constraints depend on the normal force: no force no constraint
	// That is what makes this constraint unilateral
	if(normalForce > 0.0){
		conditionsSatisfied[0] = true;

		// Now check that rolling forces are not exceeding normal force according to Coulomb friction
		if(tangentialForce <= _coulombFrictionCoefficient*normalForce){
			conditionsSatisfied[2] = true;
			conditionsSatisfied[3] = true;

			// If we are not slipping then constraint will not allow rotation of roller about normal
			// as long the required torque does not exceed the torque capacity of the tangential force.
			if( normalTorque <= _surfaceContactRadius*_coulombFrictionCoefficient*normalForce )
				conditionsSatisfied[1] = true;
		}
	}
	
	// Cache the conditions until the next reevaluation
	_defaultUnilateralConditions = conditionsSatisfied;

	return conditionsSatisfied;
}

//-----------------------------------------------------------------------------
// DISABLE
//-----------------------------------------------------------------------------

//_____________________________________________________________________________
/**
 * Get whether or not the RollingOnSurfaceConstraint is disabled.
 * Simbody multibody system instance is realized every time the isDisabled
 * changes, BUT multiple sets to the same value have no cost.
 *
 * @param isDisabled If true the constraint is disabled; if false the constraint is enabled.
 */
bool RollingOnSurfaceConstraint::isDisabled(const SimTK::State &state) const
{
	// The parent constraint in is the plane constraint, so check its value
	return _model->updMatterSubsystem().updConstraint(_indices[0]).isDisabled(state);
}

//_____________________________________________________________________________
/**
 * Set whether or not the RollingOnSurfaceConstraint is disabled.
 * Since the constraint is composed of multiple constraints, this method can  
 * diable the constraints, but enabling is not guaranteed. For example, if
 * the unilateral conditions are violated the constraint will be disabled.
 *
 * @param isDisabled If true the constraint is disabled; if false the constraint is enabled.
 */
bool RollingOnSurfaceConstraint::setDisabled(SimTK::State& state, bool isDisabled)
{
	// All constraints treated the same as default behavior i.e. at initilization
	std::vector<bool> shouldBeOn(_numConstraintEquations, !isDisabled);

	// If dynamics has been realized, then this is an attempt to enable/disable the constraint
	// during a computation and not an initialization, in which case we must check the 
	// unilateral conditions for each constraint
	if(state.getSystemStage() > Stage::Dynamics)
		shouldBeOn = unilateralConditionsSatisfied(state);

	return setDisabled(state, isDisabled, shouldBeOn);
}


bool RollingOnSurfaceConstraint::setDisabled(SimTK::State& state, bool isDisabled, std::vector<bool> shouldBeOn)
{

	for(int i=0; i < _numConstraintEquations; i++){
		SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
		bool isConstraintOn = !simConstraint.isDisabled(state);

		// Check if we already have the correct enabling of the constraint then do nothing 
		if(shouldBeOn[i] == isConstraintOn)
			continue;

		// Otherwise we have to change the status of the constraint
		if(shouldBeOn[i]){
			simConstraint.enable(state);
		}
		else{
			simConstraint.disable(state);
		}
	}

	//Update the property accordingly
	_isDisabledProp.setValue(isDisabled);

	// Return whether or not constraint is in the state the caller wanted
	// The first constraint is the "master" so its state is what we care about
	return isDisabled == _model->updMatterSubsystem().updConstraint(_indices[0]).isDisabled(state);
}


//-----------------------------------------------------------------------------
// FORCES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Ask the RollingOnSurfaceConstraint for the forces it is imposing on the system
 * Simbody multibody system must be realized to at least position
 * Returns: the bodyForces on those bodies being constrained (constrainedBodies)
 *				a SpatialVec (6 components) describing resulting torque and force
 *			mobilityForces acting along constrained mobilities	
 *
 * @param state State of model
 * @param bodyForcesInAncestor is a Vector of SpatialVecs contain constraint forces
 * @param mobilityForces is a Vector of forces that act along the constrained
 *         mobilitities associated with this constraint
 */
void RollingOnSurfaceConstraint::calcConstraintForces(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
									  SimTK::Vector& mobilityForces) const
{
	SimTK::Vector_<SimTK::SpatialVec> bfs;
	SimTK::Vector mfs;

	for(int i=0; i < _numConstraintEquations; i++){
		
		SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
		int ncb = simConstraint.getNumConstrainedBodies();
		int mp, mv, ma;

		if(!simConstraint.isDisabled(state)){
			simConstraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
			SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(state);
			simConstraint.calcConstraintForcesFromMultipliers(state, multipliers, bfs, mfs);
			
			int sbi = -1;
			int rbi = -1;
			int anc = simConstraint.getAncestorMobilizedBody().getMobilizedBodyIndex();
			
			for(int j=0; j< ncb; j++){
				if(_surfaceBody->getIndex() == simConstraint.getMobilizedBodyFromConstrainedBody(ConstrainedBodyIndex(j)).getMobilizedBodyIndex())
					sbi = j;
				if(_rollingBody->getIndex() == simConstraint.getMobilizedBodyFromConstrainedBody(ConstrainedBodyIndex(j)).getMobilizedBodyIndex())
					rbi = j;
			}

			/*
			cout << "Constraint " << i << "  forces:" << endl;
			cout << " Surf body index: " << sbi << " Expressed in: " << anc << endl;
			cout << " Roll body index: " << rbi << " Expressed in: " << anc << endl;
			bfs.dump(" Constrint body forces:");
			*/
			bodyForcesInAncestor[0] += bfs[sbi];
			bodyForcesInAncestor[1] += bfs[rbi];
			mobilityForces += mfs;
		}
	}
}

/** 
 * Methods to query a Constraint forces for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
/*
Array<std::string> RollingOnSurfaceConstraint::getRecordLabels() const
{
	Array<std::string> labels("");
	// Lagrange multipliers that enforce the constraints
	//SimTK::Constraint&contactY = _model->updMatterSubsystem().updConstraint(_indices[0]);
	//SimTK::Constraint&contactTorqueAboutY = _model->updMatterSubsystem().updConstraint( _indices[1]);
	//SimTK::Constraint&contactPointXdir = _model->updMatterSubsystem().updConstraint(_indices[2]);
	//SimTK::Constraint&contactPointZdir = _model->updMatterSubsystem().updConstraint(_indices[3]);
	labels.append(getName()+"_Fx");
	labels.append(getName()+"_Fy");
	labels.append(getName()+"_Fz");
	labels.append(getName()+"_My");

	return labels;
}
*/

/**
 * Given SimTK::State object extract all the values necessary to report constraint forces, application 
 * location frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
/*
Array<double> RollingOnSurfaceConstraint::getRecordValues(const SimTK::State& state) const
{
	// EOMs are solved for accelerations (udots) and constraint multipliers (lambdas)
	// simulataneously, so system must be realized to acceleration
	_model->getMultibodySystem().realize(state, SimTK::Stage::Acceleration);

	Array<double> values(0.0, _indices.size());

	// the individual underlying constraints
	//SimTK::Constraint&contactY = _model->updMatterSubsystem().updConstraint(_indices[0]);
	//SimTK::Constraint&contactTorqueAboutY = _model->updMatterSubsystem().updConstraint( _indices[1]);
	//SimTK::Constraint&contactPointXdir = _model->updMatterSubsystem().updConstraint(_indices[2]);
	//SimTK::Constraint&contactPointZdir = _model->updMatterSubsystem().updConstraint(_indices[3]);

	int order[4] = {2, 0, 3, 1};
	for(unsigned int i=0; i<_indices.size(); ++i){
		SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[order[i]]);
		SimTK::Vector multipliers = simConstraint.getMultipliersAsVector(state);
		
		if(multipliers.size()){
			values[i] = multipliers[0];
		}
	}

	return values;
};
*/