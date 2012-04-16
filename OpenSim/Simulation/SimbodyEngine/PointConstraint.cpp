// PointConstraint.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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

#include "PointConstraint.h"
#include "SimbodyEngine.h"

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
PointConstraint::~PointConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PointConstraint::PointConstraint() :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint PointConstraint to be copied.
 */
PointConstraint::PointConstraint(const PointConstraint &aConstraint) :
   Constraint(aConstraint),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}

PointConstraint::PointConstraint(OpenSim::Body& body1, SimTK::Vec3& locationBody1, OpenSim::Body& body2, SimTK::Vec3& locationBody2) :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec())
{
	setNull();
	setupProperties();
	_body1Name = body1.getName();
	_locationInBody1 = locationBody1;
	_body2Name = body2.getName();
	_locationInBody2 = locationBody2;

}
//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Copy data members from one PointConstraint to another.
 *
 * @param aConstraint PointConstraint to be copied.
 */
void PointConstraint::copyData(const PointConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	_body1Name = aConstraint._body1Name;
	_body2Name = aConstraint._body2Name;
	_locationInBody1 = aConstraint._locationInBody1;
	_locationInBody2 = aConstraint._locationInBody2;

}

//_____________________________________________________________________________
/**
 * Set the data members of this PointConstraint to their null values.
 */
void PointConstraint::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointConstraint::setupProperties()
{
	// Body 1 name
	_body1NameProp.setName("body_1");
	_propertySet.append(&_body1NameProp);

	// Body 2 name
	_body2NameProp.setName("body_2");
	_propertySet.append(&_body2NameProp);

	//Default location and orientation (rotation sequence)
	SimTK::Vec3 origin(0.0, 0.0, 0.0);

	// Location in Body 1 
	_locationInBody1Prop.setName("location_body_1");
	_locationInBody1Prop.setValue(origin);
	_propertySet.append(&_locationInBody1Prop);

	// Location in Body 2 
	_locationInBody2Prop.setName("location_body_2");
	_locationInBody2Prop.setValue(origin);
	_propertySet.append(&_locationInBody2Prop);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this PointConstraint.
 */
void PointConstraint::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	Constraint::setup(aModel);

	// Look up the two bodies being connected together by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_body1Name)) {
		errorMessage = "Invalid point constraint body1 (" + _body1Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_body2Name)) {
		errorMessage = "Invalid point constraint body2 (" + _body2Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_body1 = &aModel.updBodySet().get(_body1Name);
	_body2 = &aModel.updBodySet().get(_body2Name);
}

void PointConstraint::createSystem(SimTK::MultibodySystem& system) const
{
	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_body1->getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_body2->getIndex());

    // Now create a Simbody Constraint::Point
    SimTK::Constraint::Ball simtkPoint(b1, _locationInBody1, b2, _locationInBody2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	PointConstraint* mutableThis = const_cast<PointConstraint *>(this);
	mutableThis->_index  = simtkPoint.getConstraintIndex();
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
PointConstraint& PointConstraint::operator=(const PointConstraint &aConstraint)
{
	Constraint::operator=(aConstraint);
	copyData(aConstraint);
	return(*this);
}

//=============================================================================
// SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Following methods set attributes of the point constraint */
void PointConstraint::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void PointConstraint::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location for point on body 1*/
void PointConstraint::setBody1PointLocation(Vec3 location)
{
	_locationInBody1 = location;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Ball &simConstraint = (SimTK::Constraint::Ball &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultPointOnBody1(_locationInBody1);
	}
}

/** Set the location for point on body 2*/
void PointConstraint::setBody2PointLocation(Vec3 location)
{
	_locationInBody2 = location;
	//if there is a live SimTK::system, we need to push this change down to the underlying constraint.
	if(int(_index) != SimTK::InvalidIndex){
		SimTK::Constraint::Ball &simConstraint = (SimTK::Constraint::Ball &)_model->updMatterSubsystem().updConstraint(_index);
		simConstraint.setDefaultPointOnBody2(_locationInBody2);
	}
}

/** Set the point locations */
void PointConstraint::setContactPointForInducedAccelerations(const SimTK::State &s, Vec3 point)
{
	// The contact point coordinates in the surface body frame 
	Vec3 spoint;

	// make sure we are at the position stage
	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);

	// For external forces we assume point position vector is defined wrt foot (i.e., _body2)
	// because we are passing it in from a prescribed force.
	// We must also get that point position vector wrt ground (i.e., _body1)
	
	cout<<"Body 1 is" << _body1->getName() <<endl;
	cout<<"Body 2 is" << _body2->getName() <<endl;
	_model->getSimbodyEngine().transformPosition(s, *_body2, point, *_body1, spoint);
	setBody1PointLocation(spoint);
	setBody2PointLocation(point);
}

