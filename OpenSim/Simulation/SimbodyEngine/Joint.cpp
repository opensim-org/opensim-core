/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Joint.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Frank C. Anderson                                                       *
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
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
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
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Joint::Joint() : Super()
{
	setNull();
	constructInfrastructure();
}

/**
 * API constructor.
 */
Joint::Joint(const std::string &name, 
	const OpenSim::Body& parent, 
	const SimTK::Vec3& locationInParent, const SimTK::Vec3& orientationInParent,
	const OpenSim::Body& child, 
	const SimTK::Vec3& locationInChild, const SimTK::Vec3& orientationInChild, 
	bool reverse) : Super()
{
	setNull();
	constructInfrastructure();

	set_location_in_parent(locationInParent);
	set_orientation_in_parent(orientationInParent);
	set_location_in_child(locationInChild);
	set_orientation_in_child(orientationInChild);
	set_reverse(reverse);

	updConnector<Body>("parent_body").set_connected_to_name(parent.getName());
	updConnector<Body>("child_body").set_connected_to_name(child.getName());

	setName(name);
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
	setAuthors("Ajay Seth");
}

//_____________________________________________________________________________
/**
 * Construct properties and initialize their default values.
 */
void Joint::constructProperties()
{
	// Location in parent
	SimTK::Vec3 origin(0.0, 0.0, 0.0);
	constructProperty_location_in_parent(origin);

	// Orientation in parent
	constructProperty_orientation_in_parent(origin);

	// Location in child
	constructProperty_location_in_child(origin);

	// Orientation in child
	constructProperty_orientation_in_child(origin);

	// Generalized coordinates
	constructProperty_CoordinateSet(CoordinateSet());

	// Transform direction (parent->child or child->parent)
	constructProperty_reverse(false);
}

void Joint::constructConnectors()
{
	constructConnector<Body>("parent_body");
	constructConnector<Body>("child_body");
}

void Joint::setParentBodyName(const std::string& name)
{
	updConnector<Body>("parent_body").set_connected_to_name(name);
}
const std::string& Joint::getParentBodyName() const
{
	return getConnector<Body>("parent_body").get_connected_to_name();
}

void Joint::setChildBodyName(const std::string& name)
{
	updConnector<Body>("child_body").set_connected_to_name(name);
}
const std::string& Joint::getChildBodyName() const
{
	return getConnector<Body>("child_body").get_connected_to_name();
}


void Joint::finalizeFromProperties()
{
	CoordinateSet& coordinateSet = upd_CoordinateSet();

	//start from a clear slate
	clearComponents();
	// add all coordinates listed under this joint as 
	// subcomponents as long as the number of coordinates
	// does not exceed the number of dofs.
	SimTK_ASSERT1(numCoordinates() == coordinateSet.getSize(), 
		"%s list of coordinates does not match Joint degrees-of-freedom.",
                  getConcreteClassName().c_str());
	for (int i = 0; i< coordinateSet.getSize(); ++i){
		coordinateSet[i].setJoint(*this);
		addComponent(&coordinateSet[i]);
	}

	const SimTK::Vec3& orientation = get_orientation_in_child();
	const SimTK::Vec3& location = get_location_in_child();

	// CHILD TRANSFORM
	Rotation rotation(BodyRotationSequence,
		orientation[0], XAxis,
		orientation[1], YAxis,
		orientation[2], ZAxis);

	SimTK::Transform childTransform(rotation, location);
	_jointFrameInChild = childTransform;

	const SimTK::Vec3& orientationInParent = get_orientation_in_parent();
	const SimTK::Vec3& locationInParent = get_location_in_parent();

	// PARENT TRANSFORM
	Rotation parentRotation(BodyRotationSequence,
		orientationInParent[0], XAxis,
		orientationInParent[1], YAxis,
		orientationInParent[2], ZAxis);

	SimTK::Transform parentTransform(parentRotation, locationInParent);
	_jointFrameInParent = parentTransform;

	// now let base invoke on subcomponents
	Super::finalizeFromProperties();
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// CHILD BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void Joint::setChildBody(OpenSim::Body& body)
{
	updConnector<Body>("child_body").connect(body);
}

const OpenSim::Body& Joint::getChildBody() const
{
	return getConnector<Body>("child_body").getConnectee();
}

//-----------------------------------------------------------------------------
// CHILD LOCATION
//-----------------------------------------------------------------------------
void Joint::setLocationInChild(const SimTK::Vec3& location)
{
	set_location_in_child(location);
}

const SimTK::Vec3& Joint::getLocationInChild() const
{
	return get_location_in_child();
}

//-----------------------------------------------------------------------------
// CHILD ORIENTATION
//-----------------------------------------------------------------------------
void Joint::setOrientationInChild(const SimTK::Vec3& aOrientation)
{
    set_orientation_in_child(aOrientation);
}

const SimTK::Vec3& Joint::getOrientationInChild() const
{
	return get_orientation_in_child();
}

//-----------------------------------------------------------------------------
// PARENT BODY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Set the parent body to which this joint attaches.
 *
 * @param aParentBody Parent body to which this joint attaches.
 */
void Joint::setParentBody(OpenSim::Body& body)
{
	updConnector<Body>("parent_body").connect(body);
}
//_____________________________________________________________________________

const OpenSim::Body& Joint::getParentBody() const
{
	return getConnector<Body>("parent_body").getConnectee();
}

//-----------------------------------------------------------------------------
// LOCATION IN PARENT
//-----------------------------------------------------------------------------
void Joint::setLocationInParent(const SimTK::Vec3& aLocation)
{
    set_location_in_parent(aLocation);
}

const SimTK::Vec3& Joint::getLocationInParent() const
{
	return get_location_in_parent();
}

//-----------------------------------------------------------------------------
// ORIENTATION IN PARENT
//-----------------------------------------------------------------------------
void Joint::setOrientationInParent(const SimTK::Vec3& aOrientation)
{

    set_orientation_in_parent(aOrientation);
}

const SimTK::Vec3& Joint::getOrientationInParent() const
{
	return get_orientation_in_parent();
}
//_____________________________________________________________________________
/**
 * Check if a coordinate is used by the Joint.
 *
 * @param aCoordinate Coordinate to look for in joint.
 * @return True if the coordinate is used.
 */
bool Joint::isCoordinateUsed(const Coordinate& aCoordinate) const
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
	const string& bodyName = getChildBody().getName();
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

	SimTK::Vec3& location = upd_location_in_child();
	SimTK::Vec3& locationInParent = upd_location_in_parent();

	for(int i=0; i<3; i++){
		locationInParent[i]*= parentFactors[i];
		location[i]*= bodyFactors[i];
	}
}

/** Construct coordinates according to the mobilities of the Joint */
void Joint::constructCoordinates()
{
	CoordinateSet& coordinateSet = upd_CoordinateSet();
	// When this Joint is destroyed so should all its coordinates.
	coordinateSet.setMemoryOwner(true);

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

const SimTK::MobilizedBodyIndex Joint::
	getMobilizedBodyIndex(const OpenSim::Body& body) const
{
		return body._index;
} 

void Joint::setChildMobilizedBodyIndex(const SimTK::MobilizedBodyIndex index) const
{ 
	getChildBody()._index = index;
}


void Joint::doAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::doAddToSystem(system);
	/* TODO: Useful to include through debug message/log in the future
	cout << getConcreteClassName() << ":'" << getName() << "' connects parent '";
	cout << getParentBodyName() << "'[" << getParentBody().getIndex() << "] and child '";
	cout << getChildBodyName() << "'[" << getChildBody().getIndex() << "]" << endl;
     */
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

	const SimTK::MobilizedBodyIndex &mbx = getChildBody().getMobilizedBodyIndex();
	// build a unique list of underlying MobilizedBodies that are involved
	// with this Joint in addition to and not including that of the child body

	std::set<SimTK::MobilizedBodyIndex> mbds;

	const CoordinateSet& coordinateSet = get_CoordinateSet();

	for(int i=0; i<coordinateSet.getSize(); ++i){
		const MobilizedBodyIndex& coordsMbx = coordinateSet[i].getBodyIndex();
		if (coordsMbx != mbx){
			mbds.insert(coordsMbx);
		}
	}
	
	SimTK::SpatialVec FB_G = calcEquivalentSpatialForceForMobilizedBody(s, mbx, mobilityForces);
	SimTK::SpatialVec FBx_G;

	std::set<SimTK::MobilizedBodyIndex>::const_iterator it = mbds.begin();

	const SimTK::MobilizedBody &G = getModel().getMatterSubsystem().getGround();
	const SimTK::MobilizedBody &B = getModel().getMatterSubsystem().getMobilizedBody(mbx);
	SimTK::Vec3 r_BG =
		B.expressVectorInAnotherBodyFrame(s, B.getOutboardFrame(s).p(), G);

	while(it != mbds.end()){
		FBx_G = calcEquivalentSpatialForceForMobilizedBody(s, *it, mobilityForces);

		const SimTK::MobilizedBody &b = 
			getModel().getMatterSubsystem().getMobilizedBody(*it);

		
		SimTK::Vec3 r_bG = 
			b.expressVectorInAnotherBodyFrame(s, b.getOutboardFrame(s).p(), G);

		// Torques add and include term due to offset in forces
		FB_G += FBx_G; // shiftForceFromTo(FBx_G, r_bG, r_BG);
		++it;
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
/* Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified 
   by index acting at its mobilizer frame B, expressed in ground.  */
SimTK::SpatialVec Joint::calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, 
	const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const
{
	// Get the mobilized body
	const SimTK::MobilizedBody mbd    = getModel().getMatterSubsystem().getMobilizedBody(mbx);
	const SimTK::UIndex        ustart = mbd.getFirstUIndex(s);
	const int                  nu     = mbd.getNumU(s);

	const SimTK::MobilizedBody ground = getModel().getMatterSubsystem().getGround();

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

	// The spatial forces above are expresseded in the joint frame of the parent
	// Transform from parent joint frame, P into the parent body frame, Po
	const SimTK::Rotation R_PPo = (mbd.getInboardFrame(s).R());

	// Re-express forces in ground, first by describing force in the parent, Po, 
	// frame instead of joint frame
	SimTK::Vec3 vecFw = R_PPo*SimTK::Vec3::getAs(&Fw[0]);
	SimTK::Vec3 vecFv = R_PPo*SimTK::Vec3::getAs(&Fv[0]);

	//Force Acting on joint frame, B,  in child body expressed in Parent body, Po
	SimTK::SpatialVec FB_Po(vecFw, vecFv);

	const MobilizedBody parent = mbd.getParentMobilizedBody();
	// to apply spatial forces on bodies they must be expressed in ground
	vecFw = parent.expressVectorInAnotherBodyFrame(s, FB_Po[0], ground);
	vecFv = parent.expressVectorInAnotherBodyFrame(s, FB_Po[1], ground);

	// Package resulting torque and force as a spatial vec
	SimTK::SpatialVec FB_G(vecFw, vecFv);

	return FB_G;
}

void Joint::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	int documentVersion = versionNumber;
	bool converting = false;
	if (documentVersion < XMLDocument::getLatestVersion()){
		if (documentVersion<30500){
			// This used to be called "Force" back then
			XMLDocument::renameChildNode(aNode, "location", "location_in_child"); // body_B -> body
			XMLDocument::renameChildNode(aNode, "orientation", "orientation_in_child"); // direction_A -> direction
		}
	}
	Super::updateFromXMLNode(aNode, versionNumber);
}

int Joint::assignSystemIndicesToBodyAndCoordinates(
	const SimTK::MobilizedBody& mobod,
	const OpenSim::Body* mobilized,
	const int& numMobilities,
	const int& startingCoordinateIndex) const
{
	// If not OpenSim body provided as the one being mobilized assume it is 
	// and intermedidate body and ignore.
	if (mobilized){
		// Index can only be assigned to a parent or child body connected by this
		// Joint

		SimTK_ASSERT3( ( (mobilized == &getParentBody()) || 
					     (mobilized == &getChildBody()) ||
					     (mobilized == _slaveBodyForParent) ||
					     (mobilized == _slaveBodyForChild) ),
			"%s::'%s' - Cannot assign underlying system index to a Body '%s', "
			"which is not a parent or child Body of this Joint.",
                      getConcreteClassName().c_str(),
                      getName().c_str(), mobilized->getName().c_str());

		// ONLY the base Joint can do this assignment
		mobilized->_index = mobod.getMobilizedBodyIndex();
	}
	int nc = numCoordinates();
	SimTK_ASSERT3(numMobilities <= (nc - startingCoordinateIndex),
		"%s attempted to create an underlying SimTK::MobilizedBody that "
		"supplies %d mobilities but only %d required.",
                  getConcreteClassName().c_str(),
                  numMobilities, nc - startingCoordinateIndex);

	const CoordinateSet& coords = get_CoordinateSet();

	int j = startingCoordinateIndex;
	for (int iq = 0; iq < numMobilities; ++iq){
		if (j < nc){ // assign
			coords[j]._mobilizerQIndex = SimTK::MobilizerQIndex(iq);
			coords[j]._bodyIndex = mobod.getMobilizedBodyIndex();
			j++;
		}
		else{
			std::string msg = getConcreteClassName() +
				" creating MobilizedBody with more mobilities than declared Coordinates.";
			throw Exception(msg);
		}
	}
	return j;
}

/* Return the equivalent (internal) SimTK::Rigid::Body for a given parent OR
child OpenSim::Body. Not guaranteed to be valid until after doAddToSystem on
Body has be called  */
const SimTK::Body::Rigid& Joint::getParentInternalRigidBody() const
{
	if (_slaveBodyForParent){
		return _slaveBodyForParent->getInternalRigidBody();
	}
	return getParentBody().getInternalRigidBody();
}
const SimTK::Body::Rigid& Joint::getChildInternalRigidBody() const
{
	if (_slaveBodyForChild){
		return _slaveBodyForChild->getInternalRigidBody();
	}
	return getChildBody().getInternalRigidBody();
}
