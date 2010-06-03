// WeldConstraint.cpp
// Author: Frank C. Anderson, Peter Loan, Ajay Seth
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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

#include "WeldConstraint.h"
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
WeldConstraint::~WeldConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
WeldConstraint::WeldConstraint() :
	Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec3()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec3()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec3()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec3())
{
	setNull();
	setupProperties();
}

WeldConstraint::WeldConstraint(const std::string &name, OpenSim::Body& body1, SimTK::Vec3 locationInBody1, SimTK::Vec3 orientationInBody1,
							   OpenSim::Body& body2, SimTK::Vec3 locationInBody2, SimTK::Vec3 orientationInBody2) : Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec3()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec3()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec3()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec3())
{
	setName(name);
	_body1Name = body1.getName();
	_body2Name = body2.getName();
	_locationInBody1 = locationInBody1;
	_orientationInBody1 = orientationInBody1;
	_locationInBody2 = locationInBody2;
	_orientationInBody2 = orientationInBody2;
}

WeldConstraint::WeldConstraint(const std::string &name, OpenSim::Body& body1, SimTK::Transform transformInBody1, 
							   OpenSim::Body& body2, SimTK::Transform transformInBody2) : Constraint(),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec3()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec3()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec3()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec3())
{
	setName(name);
	_body1Name = body1.getName();
	_body2Name = body2.getName();
	_locationInBody1 = transformInBody1.p();
	_orientationInBody1 = transformInBody1.R().convertRotationToBodyFixedXYZ();
	_locationInBody2 = transformInBody2.p();
	_orientationInBody2 = transformInBody2.R().convertRotationToBodyFixedXYZ();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint WeldConstraint to be copied.
 */
WeldConstraint::WeldConstraint(const WeldConstraint &aConstraint) :
   Constraint(aConstraint),
	_body1Name(_body1NameProp.getValueStr()),
	_body2Name(_body2NameProp.getValueStr()),
	_locationInBody1(_locationInBody1Prop.getValueDblVec3()),
	_orientationInBody1(_orientationInBody1Prop.getValueDblVec3()),
	_locationInBody2(_locationInBody2Prop.getValueDblVec3()),
	_orientationInBody2(_orientationInBody2Prop.getValueDblVec3())
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
Object* WeldConstraint::copy() const
{
	WeldConstraint *constraint = new WeldConstraint(*this);
	return(constraint);
}
//_____________________________________________________________________________
/**
 * Copy data members from one WeldConstraint to another.
 *
 * @param aConstraint WeldConstraint to be copied.
 */
void WeldConstraint::copyData(const WeldConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	_body1Name = aConstraint._body1Name;
	_body2Name = aConstraint._body2Name;
	_locationInBody1 = aConstraint._locationInBody1;
	_orientationInBody1 = aConstraint._orientationInBody1;
	_locationInBody2 = aConstraint._locationInBody2;
	_orientationInBody2 = aConstraint._orientationInBody2;
}

//_____________________________________________________________________________
/**
 * Set the data members of this WeldConstraint to their null values.
 */
void WeldConstraint::setNull()
{
	setType("WeldConstraint");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void WeldConstraint::setupProperties()
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

	// Orientation in Body 1 
	_orientationInBody1Prop.setName("orientation_body_1");
	_orientationInBody1Prop.setValue(origin);
	_propertySet.append(&_orientationInBody1Prop);

	// Location in Body 2 
	_locationInBody2Prop.setName("location_body_2");
	_locationInBody2Prop.setValue(origin);
	_propertySet.append(&_locationInBody2Prop);

	// Orientation in Body 2 
	_orientationInBody2Prop.setName("orientation_body_2");
	_orientationInBody2Prop.setValue(origin);
	_propertySet.append(&_orientationInBody2Prop);

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this WeldConstraint.
 */
void WeldConstraint::setup(Model& aModel)
{
	string errorMessage;

	// Base class
	Constraint::setup(aModel);

	// Look up the two bodies being welded together by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_body1Name)) {
		errorMessage = "Invalid weld body1 (" + _body1Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_body2Name)) {
		errorMessage = "Invalid weld body2 (" + _body2Name + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_body1 = &aModel.updBodySet().get(_body1Name);
	_body2 = &aModel.updBodySet().get(_body2Name);
}

void WeldConstraint::createSystem(SimTK::MultibodySystem& system) const
{
	// Get underlying mobilized bodies
	SimTK::MobilizedBody b1 = _model->updMatterSubsystem().getMobilizedBody(_body1->getIndex());
	SimTK::MobilizedBody b2 = _model->updMatterSubsystem().getMobilizedBody(_body2->getIndex());
	// Build the transforms
	SimTK::Rotation r1; r1.setRotationToBodyFixedXYZ(_orientationInBody1);
	SimTK::Rotation r2; r2.setRotationToBodyFixedXYZ(_orientationInBody2);
	SimTK::Transform inb1(r1, _locationInBody1);
	SimTK::Transform inb2(r2, _locationInBody2);

    // Now create a Simbody Constraint::Weld
    SimTK::Constraint::Weld simtkWeld(b1, inb1, b2, inb2);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	WeldConstraint* mutableThis = const_cast<WeldConstraint *>(this);
	mutableThis->_index = simtkWeld.getConstraintIndex();
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
WeldConstraint& WeldConstraint::operator=(const WeldConstraint &aConstraint)
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
 * Following methods set attributes of the weld constraint */
void WeldConstraint::setBody1ByName(std::string aBodyName)
{
	_body1Name = aBodyName;
}

void WeldConstraint::setBody2ByName(std::string aBodyName)
{
	_body2Name = aBodyName;
}

/** Set the location and orientation (optional) for weld on body 1*/
void WeldConstraint::setBody1WeldLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody1 = location;
	_orientationInBody1 = orientation;
}

/** Set the location and orientation (optional) for weld on body 2*/
void WeldConstraint::setBody2WeldLocation(Vec3 location, Vec3 orientation)
{
	_locationInBody2 = location;
	_orientationInBody2 = orientation;
}
