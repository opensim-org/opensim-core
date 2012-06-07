// PointOnLineConstraint.cpp
// Author: Samuel Hamner
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

#include "PointOnLineConstraint.h"
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
PointOnLineConstraint::~PointOnLineConstraint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PointOnLineConstraint::PointOnLineConstraint() :
	Constraint(),
	_lineBodyName(_lineBodyNameProp.getValueStr()),
	_lineDirection(_lineDirectionProp.getValueDblVec()),
	_pointOnLine(_pointOnLineProp.getValueDblVec()),
	_followerBodyName(_followerBodyNameProp.getValueStr()),
	_pointOnFollower(_pointOnFollowerProp.getValueDblVec())
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aConstraint PointOnLineConstraint to be copied.
 */
PointOnLineConstraint::PointOnLineConstraint(const PointOnLineConstraint &aConstraint) :
   Constraint(aConstraint),
	_lineBodyName(_lineBodyNameProp.getValueStr()),
	_lineDirection(_lineDirectionProp.getValueDblVec()),
	_pointOnLine(_pointOnLineProp.getValueDblVec()),
	_followerBodyName(_followerBodyNameProp.getValueStr()),
	_pointOnFollower(_pointOnFollowerProp.getValueDblVec())
{
	setNull();
	setupProperties();
	copyData(aConstraint);
}

//_____________________________________________________________________________
/**
 * Convenience (API) constructor.
 */
   PointOnLineConstraint::PointOnLineConstraint(OpenSim::Body& lineBody, Vec3 lineDirection, Vec3 pointOnLine,
			OpenSim::Body& followerBody, Vec3 followerPoint) :
	Constraint(),
	_lineBodyName(_lineBodyNameProp.getValueStr()),
	_lineDirection(_lineDirectionProp.getValueDblVec()),
	_pointOnLine(_pointOnLineProp.getValueDblVec()),
	_followerBodyName(_followerBodyNameProp.getValueStr()),
	_pointOnFollower(_pointOnFollowerProp.getValueDblVec())
{
	setNull();
	setupProperties();
	_lineBodyName = lineBody.getName();
	_lineDirection = lineDirection;
	_pointOnLine = pointOnLine;
	_followerBodyName = followerBody.getName();
	_pointOnFollower = followerPoint;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================

//_____________________________________________________________________________
/**
 * Copy data members from one PointOnLineConstraint to another.
 *
 * @param aConstraint PointOnLineConstraint to be copied.
 */
void PointOnLineConstraint::copyData(const PointOnLineConstraint &aConstraint)
{
	Constraint::copyData(aConstraint);
	_lineBodyName = aConstraint._lineBodyName;
	_lineDirection = aConstraint._lineDirection;
	_pointOnLine = aConstraint._pointOnLine;
	_followerBodyName = aConstraint._followerBodyName;
	_pointOnFollower = aConstraint._pointOnFollower;
}

//_____________________________________________________________________________
/**
 * Set the data members of this PointOnLineConstraint to their null values.
 */
void PointOnLineConstraint::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointOnLineConstraint::setupProperties()
{

	// Line Body Name
	_lineBodyNameProp.setName("line_body");
	_propertySet.append(&_lineBodyNameProp);

	// Line Direction
	_lineDirectionProp.setName("line_direction_vec");
	_propertySet.append(&_lineDirectionProp);

	//Default location and orientation (rotation sequence)
	SimTK::Vec3 origin(0.0, 0.0, 0.0);

	// Default Point On Line
	_pointOnLineProp.setName("point_on_line");
	_pointOnLineProp.setValue(origin);
	_propertySet.append(&_pointOnLineProp);

	// Follower Body
	_followerBodyNameProp.setName("follower_body");
	_propertySet.append(&_followerBodyNameProp);

	// Point On Follower Body 
	_pointOnFollowerProp.setName("point_on_follower");
	_pointOnFollowerProp.setValue(origin);
	_propertySet.append(&_pointOnFollowerProp);

}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this PointOnLineConstraint.
 */
void PointOnLineConstraint::connectToModel(Model& aModel) {
	Super::connectToModel(aModel);

	string errorMessage;
	// Look up the two bodies being constrained together by name in the
	// model and might as well keep a pointer to them
	if (!aModel.updBodySet().contains(_lineBodyName)) {
		errorMessage = "Invalid line body (" + _lineBodyName + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	if (!aModel.updBodySet().contains(_followerBodyName)) {
		errorMessage = "Invalid follower body (" + _followerBodyName + ") specified in Constraint " + getName();
		throw (Exception(errorMessage.c_str()));
	}
	_lineBody = &aModel.updBodySet().get(_lineBodyName);
	_followerBody = &aModel.updBodySet().get(_followerBodyName);
}

void PointOnLineConstraint::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

	// Get underlying mobilized bodies
	SimTK::MobilizedBody lb = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_lineBody->getIndex());
	SimTK::MobilizedBody fb = _model->updMatterSubsystem().getMobilizedBody((MobilizedBodyIndex)_followerBody->getIndex());

	// Normalize Line Direction
	SimTK::UnitVec3 normLineDirection(_lineDirection.normalize());

    // Now create a Simbody Constraint::PointOnLine
	//PointOnLine(MobilizedBody& lineBody_B, const UnitVec3& defaultLineDirection_B, const Vec3& defaultPointOnLine_B,
	//           MobilizedBody& followerBody_F, const Vec3& defaultFollowerPoint_F);
    SimTK::Constraint::PointOnLine simtkPointOnLine(lb, normLineDirection, _pointOnLine, fb, _pointOnFollower);
    
    // Beyond the const Component get the index so we can access the SimTK::Constraint later
	PointOnLineConstraint* mutableThis = const_cast<PointOnLineConstraint *>(this);
	mutableThis->_index = simtkPointOnLine.getConstraintIndex();
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
PointOnLineConstraint& PointOnLineConstraint::operator=(const PointOnLineConstraint &aConstraint)
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
 * Following methods set attributes of the point on line constraint */
void PointOnLineConstraint::setLineBodyByName(std::string aBodyName)
{
	_lineBodyName = aBodyName;
}

void PointOnLineConstraint::setFollowerBodyByName(std::string aBodyName)
{
	_followerBodyName = aBodyName;
}

/** Set the line direction for the point on line constraint*/
void PointOnLineConstraint::setLineDirection(Vec3 direction)
{
	_lineDirection = direction;
}

/** Set the location of the point on the line*/
void PointOnLineConstraint::setPointOnLine(Vec3 point)
{
	_pointOnLine = point;
}

/** Set the location of the point on the follower body*/
void PointOnLineConstraint::setPointOnFollower(Vec3 point)
{
	_pointOnFollower = point;
}
