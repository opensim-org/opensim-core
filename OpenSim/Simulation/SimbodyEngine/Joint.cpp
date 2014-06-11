/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Joint.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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
#include "Joint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/ScaleSet.h>

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
Joint::~Joint()
{
	if(!_body.empty())
		_body->_joint = NULL;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Joint::Joint() :
	ModelComponent()
{
	setNull();
	constructProperties();
}

/**
 * API constructor.
 */
Joint::Joint(const std::string &name, 
	OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
	OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, 
	bool reverse) :	ModelComponent()
{
	setNull();
	constructProperties();

	setBody(body);
	setParentBody(parent);

	set_parent_body(parent.getName());
	set_location_in_parent(locationInParent);
	set_orientation_in_parent(orientationInParent);
	set_location(locationInBody);
	set_orientation(orientationInBody);
	set_reverse(reverse);

	Object::setName(name);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Set the data members of this Joint to their null values.
 */
void Joint::setNull()
{
	setAuthors("Frank C. Anderson, Peter Loan, Ajay Seth");
	_parentBody = NULL;
	_body = NULL;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Joint::constructProperties(void)
{
	// Parent body name
	constructProperty_parent_body("");

	// Location in parent
	SimTK::Vec3 origin(0.0, 0.0, 0.0);
	constructProperty_location_in_parent(origin);

	// Orientation in parent
	constructProperty_orientation_in_parent(origin);

	// Location in child
	constructProperty_location(origin);

	// Orientation in child
	constructProperty_orientation(origin);

	// Generalized coordinates
	constructProperty_CoordinateSet(CoordinateSet());

	// Transform direction (parent->child or child->parent)
	constructProperty_reverse(false);
}

//_____________________________________________________________________________

void Joint::connectToModel(Model& aModel) {

	Super::connectToModel(aModel);

	string errorMessage;

	const std::string& parentName = get_parent_body();

	// Look up the parent and child bodies by name in the
	if (!aModel.updBodySet().contains(parentName)) {
		errorMessage += "Invalid parent body (" + parentName + ") specified in joint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	setParentBody(aModel.updBodySet().get(parentName));

	CoordinateSet& coordinateSet = upd_CoordinateSet();
	coordinateSet.invokeConnectToModel(aModel);

	for(int i = 0; i< coordinateSet.getSize(); i++)
		coordinateSet[i].setJoint(*this);

	const SimTK::Vec3& orientation = get_orientation();
	const SimTK::Vec3& location = get_location();

	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence, orientation[0],XAxis, orientation[1],YAxis, orientation[2],ZAxis);
	SimTK::Transform childTransform(rotation, location);
	_jointFrameInBody = childTransform;

	const SimTK::Vec3& orientationInParent = get_orientation_in_parent();
	const SimTK::Vec3& locationInParent = get_location_in_parent();

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence, orientationInParent[0],XAxis, orientationInParent[1],YAxis, orientationInParent[2],ZAxis);
	SimTK::Transform parentTransform(parentRotation, locationInParent);
	_jointFrameInParent = parentTransform;
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void Joint::setBody(OpenSim::Body& aBody)
{
	_body = (Body*) &aBody;
}

const OpenSim::Body& Joint::getBody() const
{
	return *_body;
}

OpenSim::Body& Joint::updBody() {
	if(!_body) { throw OpenSim::Exception("Joint::updBody : Body is null"); }
	return *_body;
}

//-----------------------------------------------------------------------------
// LOCATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the location of this joint in its body.
 *
 * @param aLocation New location expressed in the local body frame.
 */
void Joint::setLocation(const SimTK::Vec3& aLocation)
{
	if (_model != NULL)
		_model->invalidateSystem();

    set_location(aLocation);
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its child body.
 *
 * @param rLocation Current location expressed in the local body frame.
 */
void Joint::getLocation(SimTK::Vec3& rLocation) const
{
	// TODO: Return a reference instead of void.
	rLocation = get_location();
}

//-----------------------------------------------------------------------------
// ORIENTATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the orientation of this joint in its body.
 *
 * @param aOrientation New orientation expressed in the local body frame in
 * body-fixed X-Y-Z Euler angles.
 */
void Joint::setOrientation(const SimTK::Vec3& aOrientation)
{
	if (_model != NULL)
		_model->invalidateSystem();

    set_orientation(aOrientation);
}
//_____________________________________________________________________________
/**
 * Get the orientation of this joint in its body.
 *
 * @param rOrientation Current orientation of the joint expressed in the local
 * body frame in body-fixed X-Y-Z Euler angles.
 */
void Joint::getOrientation(SimTK::Vec3 &rOrientation) const
{
	// TODO: Return a reference instead of void.
	rOrientation = get_orientation();
}

//-----------------------------------------------------------------------------
// PARENT BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name of the joint's parent body.
 *
 * @param Name of the parent body.
 */
void Joint::setParentName(const string& aName)
{
	set_parent_body(aName);
}
//_____________________________________________________________________________
/**
 * Get the name of the joint's parent body.
 *
 * @return Name of the parent body.
 */
string Joint::getParentName() const
{
	return get_parent_body();
}
//_____________________________________________________________________________
/**
 * Set the parent body to which this joint attaches.
 *
 * @param aParentBody Parent body to which this joint attaches.
 */
void Joint::setParentBody(OpenSim::Body& aBody)
{
	_parentBody = (Body*) &aBody;
}
//_____________________________________________________________________________

const OpenSim::Body& Joint::getParentBody() const
{
    if (_parentBody.empty())
        throw Exception("Joint::getParentBody() : Joint has not been initialized");
	return *_parentBody;
}

OpenSim::Body& Joint::updParentBody() {

	if (!_parentBody)
        throw Exception("Joint::getParentBody() : Joint has not been initialized");
	return *_parentBody;
}

//-----------------------------------------------------------------------------
// LOCATION IN PARENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the location of this joint in its parent body.
 *
 * @param aLocation New location expressed in the parent body frame.
 */
void Joint::setLocationInParent(const SimTK::Vec3& aLocation)
{
	if (_model != NULL)
		_model->invalidateSystem();

    set_location_in_parent(aLocation);
}
//_____________________________________________________________________________
/**
 * Get the location of this joint in its parent body.
 *
 * @param rLocation Currnt location expressed in the parent body frame.
 */
void Joint::getLocationInParent(SimTK::Vec3& rLocation) const
{
	rLocation=get_location_in_parent();
}

//-----------------------------------------------------------------------------
// ORIENTATION IN PARENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the orientation of this joint in its parent body.
 *
 * @param aOrientation New orientation expressed in the parent body frame in
 * body-fixed X-Y-Z Euler angles.
 */
void Joint::setOrientationInParent(const SimTK::Vec3& aOrientation)
{
	if (_model != NULL)
		_model->invalidateSystem();

    set_orientation_in_parent(aOrientation);
}
//_____________________________________________________________________________
/**
 * Get the orientation of this joint in its parent body.
 *
 * @param rOrientation Current orientation expressed in the parent body frame
 * in body-fixed X-Y-Z Euler angles.
 */
void Joint::getOrientationInParent(SimTK::Vec3& rOrientation) const
{
	rOrientation = get_orientation_in_parent();
}
//_____________________________________________________________________________
/**
 * Check if a coordinate is used by the Joint.
 *
 * @param aCoordinate Coordinate to look for in joint.
 * @return True if the coordinate is used.
 */
bool Joint::isCoordinateUsed(Coordinate& aCoordinate) const
{
	const CoordinateSet& coordinateSet = get_CoordinateSet();
	int i, size = coordinateSet.getSize();
	for(i=0; i<size; i++) {
		if(&coordinateSet.get(i) == &aCoordinate) return true;
	}

	return false;
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________

void Joint::scale(const ScaleSet& scaleSet)
{
	SimTK::Vec3 parentFactors(1.0);
	SimTK::Vec3 bodyFactors(1.0);

	// SCALING TO DO WITH THE PARENT BODY -----
	// Joint kinematics are scaled by the scale factors for the
	// parent body, so get those body's factors
	const string& parentName = getParentBody().getName();
	const string& bodyName = getBody().getName();
	// Get scale factors
	bool found_p = false;
	bool found_b = false; 
	for (int i=0; i<scaleSet.getSize(); i++) {
		Scale& scale = scaleSet.get(i);
		if (!found_p & scale.getSegmentName() == parentName) {
			scale.getScaleFactors(parentFactors);
			found_p = true;
		}
		if (!found_b & scale.getSegmentName() == bodyName) {
			scale.getScaleFactors(bodyFactors);
			found_b = true;
		}
		if(found_p & found_b)
			break;
	}

	SimTK::Vec3& location = upd_location();
	SimTK::Vec3& locationInParent = upd_location_in_parent();

	for(int i=0; i<3; i++){
		locationInParent[i]*= parentFactors[i];
		location[i]*= bodyFactors[i];
	}
    if (_model != NULL)
        _model->invalidateSystem();
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
/** Verify that parent for this Joint is valid */
void Joint::checkParentBody()
{
	if(!(_parentBody) || !SimTK::MobilizedBodyIndex::isValid(getMobilizedBodyIndex(_parentBody))){
		throw(Exception("Joint :: Attempted to connect to a parent body that was itself not connected."));
	}
}

/** Construct coordinates according to the mobilities of the Joint */
void Joint::constructCoordinates()
{
	CoordinateSet& coordinateSet = upd_CoordinateSet();

	// Check how many coordinates are already defined if any
	int ncoords = coordinateSet.getSize();

	for(int i = ncoords; i< numCoordinates() ; i++){
		std::stringstream name;
		name << getName() << "_coord_" << i;
		Coordinate *coord = new Coordinate();
		coord->setName(name.str());
		coordinateSet.adoptAndAppend(coord);
	}
}

SimTK::MobilizedBodyIndex Joint::
	getMobilizedBodyIndex(OpenSim::Body *aBody) const
{
	return aBody->_index;
} 

void Joint::setMobilizedBodyIndex(OpenSim::Body *aBody, 
									SimTK::MobilizedBodyIndex index) const
{ 
	aBody->_index = index;
}


// TODO: note that child must invoke Joint::addToSystem()
// *after* it creates its mobilized body; that is an API bug.
void Joint::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);

	// Each coordinate needs to know it's body index and mobility index.
	const CoordinateSet& coordinateSet = get_CoordinateSet();

	int nq = coordinateSet.getSize();

	for(int iq=0;iq<nq;iq++) {
		// Coordinate
		Coordinate &q = (Coordinate&)coordinateSet.get(iq);
		q._bodyIndex = getMobilizedBodyIndex(_body);
		// The mobility index is the same as the order in which the coordinate appears in the coordinate set.
		// The functions (and their dependencies) determine how the coordinates gets used when constructing
		// the transform from parent to child.
		q._mobilizerQIndex = SimTK::MobilizerQIndex(iq);
	}
}

void Joint::initStateFromProperties(SimTK::State& s) const
{
	Super::initStateFromProperties(s);

	const CoordinateSet& coordinateSet = get_CoordinateSet();
    for (int i = 0; i < coordinateSet.getSize(); i++)
        coordinateSet.get(i).initStateFromProperties(s);
}

void Joint::setPropertiesFromState(const SimTK::State& state)
{
	Super::setPropertiesFromState(state);

	const CoordinateSet& coordinateSet = get_CoordinateSet();
    for (int i = 0; i < coordinateSet.getSize(); i++)
        coordinateSet.get(i).setPropertiesFromState(state);
}


//=============================================================================
// Computation
//=============================================================================
/* Calculate the equivalent spatial force, FB_G, acting on the body connected by this joint at 
   its location B, expressed in ground.  */
SimTK::SpatialVec Joint::calcEquivalentSpatialForce(const SimTK::State &s, const SimTK::Vector &mobilityForces) const
{
	// The number of mobilities for the entire system.
	int nm = _model->getMatterSubsystem().getNumMobilities();

	if(nm != mobilityForces.size()){
		throw Exception("Joint::calcEquivalentSpatialForce(): input mobilityForces does not match model's mobilities");
	}

	const SimTK::MobilizedBodyIndex &mbx = _body->getIndex();
	SimTK::Array_<SimTK::MobilizedBodyIndex> mbds;

	const CoordinateSet& coordinateSet = get_CoordinateSet();

	for(int i=0; i<coordinateSet.getSize(); ++i){
		if(coordinateSet[i].getBodyIndex() != mbx){
			if(mbds.size()){
				if(coordinateSet[i].getBodyIndex() > *mbds.end())
					mbds.push_back(coordinateSet[i].getBodyIndex());
			}
		}
	}

	SimTK::SpatialVec FB_G = calcEquivalentSpatialForceForMobilizedBody(s, mbx, mobilityForces);
	SimTK::SpatialVec FBx_G;

	for(unsigned int i=0; i<mbds.size(); ++i){
		FBx_G = calcEquivalentSpatialForceForMobilizedBody(s, mbds[i], mobilityForces);
		const SimTK::MobilizedBody &B = _model->updMatterSubsystem().getMobilizedBody(mbx);
		const SimTK::MobilizedBody &b = _model->updMatterSubsystem().getMobilizedBody(mbds[i]);
		const SimTK::MobilizedBody &G = _model->getMatterSubsystem().getGround();

		SimTK::Vec3 r_BG = B.expressVectorInAnotherBodyFrame(s, B.getOutboardFrame(s).p(), G);
		SimTK::Vec3 r_bG = b.expressVectorInAnotherBodyFrame(s, b.getOutboardFrame(s).p(), G);

		// Torques add and include term due to offset in forces
		FB_G += shiftForceFromTo(FBx_G, r_bG, r_BG);
	}

	return FB_G;
}

/** Joints only produce power when internal constraint forces have components along
	the mobilities of the joint (for example to satisfy prescribed motion). In 
    which case the joint power is the constraint forces projected onto the mobilities
	multiplied by the mobilities (internal coordinate velocities). Only constraints
	internal to the joint are accounted for, not external constrainst that effect
	joint motion. */
double Joint::calcPower(const SimTK::State &s) const
{
	const CoordinateSet &coords = getCoordinateSet();
	int nc = coords.getSize();

	double power = 0;
	for(int i=0; i<nc; ++i){
		if (coords[i].isPrescribed(s)){
			// get the reaction force for this coordinate prescribed motion constraint
			const SimTK::Constraint &pc = _model->updMultibodySystem().updMatterSubsystem().getConstraint(coords[i]._prescribedConstraintIndex);
			power += pc.calcPower(s);
		}
	}

	return power;
}

//=============================================================================
// Helper
//=============================================================================
/* Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified by index 
   acting at its mobilizer frame B, expressed in ground.  */
SimTK::SpatialVec Joint::calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const
{
	// Get the mobilized body
	const SimTK::MobilizedBody mbd    = _model->updMatterSubsystem().getMobilizedBody(mbx);
	const SimTK::UIndex        ustart = mbd.getFirstUIndex(s);
	const int                  nu     = mbd.getNumU(s);

    if (nu == 0) // No mobility forces (weld joint?).
        return SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0));

	// Construct the H (joint jacobian, joint transition) matrrix for this mobilizer
	SimTK::Matrix transposeH_PB_w(nu, 3);
	SimTK::Matrix transposeH_PB_v(nu, 3);
	// from individual columns
	SimTK::SpatialVec Hcol;
	
	// To obtain the joint Jacobian, H_PB (H_FM in Simbody) need to be realized to at least position
	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);

	SimTK::Vector f(nu, 0.0);
	for(int i =0; i<nu; ++i){
		f[i] = mobilityForces[ustart + i];
		// Get the H matrix for this Joint by constructing it from the operator H*u
		Hcol = mbd.getH_FMCol(s, SimTK::MobilizerUIndex(i));
		const SimTK::Vector hcolw(Hcol[0]);
		const SimTK::Vector hcolv(Hcol[1]);

		transposeH_PB_w[i] = ~hcolw;
		transposeH_PB_v[i] = ~hcolv;
	}

	// Spatial force and torque vectors
	SimTK::Vector Fv(3, 0.0), Fw(3, 0.0);

	// Solve the pseudoinverse problem of Fv = pinv(~H_PB_G_v)*f;
	SimTK::FactorQTZ pinvForce(transposeH_PB_v);

	//if rank = 0, body force cannot contribute to the mobility force
	if(pinvForce.getRank() > 0)
		pinvForce.solve(f, Fv);
	
	// Now solve the pseudoinverse for torque for any unaccounted f: Fw = pinv(~H_PB_G_w)*(f - ~H_PB_G_v*Fv);
	SimTK::FactorQTZ pinvTorq(transposeH_PB_w);

	//if rank = 0, body torque cannot contribute to the mobility force
	if(pinvTorq.getRank() > 0)
		pinvTorq.solve(f, Fw);
	
	// Now we have two solution with either the body force Fv or body torque accounting for some or all of f
	SimTK::Vector fv =  transposeH_PB_v*Fv;
	SimTK::Vector fw =  transposeH_PB_w*Fw; 

	// which to choose? Choose the more effective as fx.norm/Fx.norm
	if(fv.norm() > SimTK::SignificantReal){ // if body force can contributes at all
		// if body torque can contribute too and it is more effective
		if(fw.norm() > SimTK::SignificantReal){
			if (fw.norm()/Fw.norm() > fv.norm()/Fv.norm() ){ 
				// account for f using torque, Fw, so compute Fv with remainder
				pinvForce.solve(f-fw, Fv);		
			}else{
				// account for f using force, Fv, first and Fw from remainder
				pinvTorq.solve(f-fv, Fw);
			}
		}
		// else no torque contribution and Fw should be zero
	}
	// no force contribution but have a torque
	else if(fw.norm() > SimTK::SignificantReal){
		// just Fw
	}
	else{
		// should be the case where gen force is zero.
		assert(f.norm() < SimTK::SignificantReal);
	}

	// Transform from parent joint frame, P in the parent body, Po
	const SimTK::Rotation R_PPo = (mbd.getInboardFrame(s).R());

	// Re-express forces in ground, first by describing force in the parent, Po, frame instead of joint frame
	SimTK::Vec3 vecFw = R_PPo*SimTK::Vec3::getAs(&Fw[0]);
	SimTK::Vec3 vecFv = R_PPo*SimTK::Vec3::getAs(&Fv[0]);

	// to apply spatial forces on bodies they must be expressed in ground
	vecFw = mbd.getParentMobilizedBody().expressVectorInAnotherBodyFrame(s, vecFw, _model->getMatterSubsystem().getGround());
	vecFv = mbd.getParentMobilizedBody().expressVectorInAnotherBodyFrame(s, vecFv, _model->getMatterSubsystem().getGround());

	// Package resulting torque and force as a spatial vec
	SimTK::SpatialVec FB_G(vecFw, vecFv);

	return FB_G;

}
