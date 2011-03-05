// RollingOnSurfaceConstraint.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this OpenSim::Body.
 */
Object* RollingOnSurfaceConstraint::copy() const
{
	RollingOnSurfaceConstraint *constraint = new RollingOnSurfaceConstraint(*this);
	return(constraint);
}
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
	setType("RollingOnSurfaceConstraint");
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
void RollingOnSurfaceConstraint::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	UnilateralConstraint::setup(aModel);

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

void RollingOnSurfaceConstraint::createSystem(SimTK::MultibodySystem& system) const
{
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
}

void RollingOnSurfaceConstraint::initState(SimTK::State& state) const
{
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

void RollingOnSurfaceConstraint::setDefaultsFromState(const SimTK::State& state)
{
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
void RollingOnSurfaceConstraint::setContactPointOnSurfaceBody(const SimTK::State &s, Vec3 point)
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
 * @param bodyForcesInParent is a Vector of SpatialVecs contain constraint forces
 * @param mobilityForces is a Vector of forces that act along the constrained
 *         mobilitities associated with this constraint
 */
void RollingOnSurfaceConstraint::calcConstraintForces(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInParent, 
									  SimTK::Vector& mobilityForces)
{
	SimTK::Vector_<SimTK::SpatialVec> bfs;
	SimTK::Vector mfs;

	for(int i=0; i < _numConstraintEquations; i++){
		
		SimTK::Constraint& simConstraint = _model->updMatterSubsystem().updConstraint(_indices[i]);
		int ncb = simConstraint.getNumConstrainedBodies();
		int mp, mv, ma;

		if(!simConstraint.isDisabled(state)){
			simConstraint.getNumConstraintEquationsInUse(state, mp, mv, ma);
			simConstraint.calcConstraintForcesFromMultipliers(state, simConstraint.getMultipliersAsVector(state), bfs, mfs);
			
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
			bodyForcesInParent[0] += bfs[sbi];
			bodyForcesInParent[1] += bfs[rbi];
			mobilityForces += mfs;
		}
	}
}

/** 
 * Methods to query a Constraint forces for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
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

/**
 * Given SimTK::State object extract all the values necessary to report constraint forces, application 
 * location frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
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