// TransformAxis.cpp
// Author: Peter Loan, Frank C. Anderson
/*
 * Copyright (c)  2006-2007, Stanford University. All rights reserved. 
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
#include "TransformAxis.h"
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>

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
 * Default constructor.
 */
TransformAxis::TransformAxis() :
	_isRotation(_isRotationProp.getValueBool()),
	_axis(_axisProp.getValueDblVec3())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
TransformAxis::~TransformAxis()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof TransformAxis to be copied.
 */
TransformAxis::TransformAxis(const TransformAxis &aDof) :
	AbstractTransformAxis(aDof),
	_isRotation(_isRotationProp.getValueBool()),
	_axis(_axisProp.getValueDblVec3())
{
	setNull();
	setupProperties();
	copyData(aDof);
}

//_____________________________________________________________________________
/**
 * Copy this dof and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this TransformAxis.
 */
Object* TransformAxis::copy() const
{
	TransformAxis *transform = new TransformAxis(*this);
	return(transform);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one TransformAxis to another.
 *
 * @param aDof TransformAxis to be copied.
 */
void TransformAxis::copyData(const TransformAxis &aTransform)
{
	_isRotation = aTransform._isRotation;
	_axis = aTransform._axis;
}

//_____________________________________________________________________________
/**
 * Set the data members of this TransformAxis to their null values.
 */
void TransformAxis::setNull()
{
	setType("TransformAxis");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void TransformAxis::setupProperties()
{
	// Is this a rotation or translation transform?
	_isRotationProp.setName("is_rotation");
	_isRotationProp.setValue(true);
	_propertySet.append(&_isRotationProp);

	// Transformation axis
	const SimTK::Vec3 defaultAxis(1.0, 0.0, 0.0);
	_axisProp.setName("axis");
	_axisProp.setValue(defaultAxis);
	_propertySet.append(&_axisProp);
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
TransformAxis& TransformAxis::operator=(const TransformAxis &aDof)
{
	// BASE CLASS
	AbstractTransformAxis::operator=(aDof);

	copyData(aDof);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the current value of the transform.
 *
 * @return Current value of the transform.
 */
void TransformAxis::
setMotionType(AbstractTransformAxis::MotionType aType) 
{
	if(aType==Rotational) {
		_isRotation = true;
	}else{
		_isRotation = false;
	}
}
//_____________________________________________________________________________
/**
 * Get the current value of the transform.
 *
 * @return Current value of the transform.
 */
AbstractTransformAxis::MotionType TransformAxis::
getMotionType() const 
{
	if(_isRotation)
		return Rotational;
	else
		return Translational;
}

//_____________________________________________________________________________
/**
 * Set the transformation axis.
 *
 * @param aAxis Transformation axis.
 */
void TransformAxis::
setAxis(const SimTK::Vec3& aAxis)
{
	_axis = aAxis;
}
//_____________________________________________________________________________
/**
 * Get the transformation axis.
 *
 * @param rAxis Transformation axis is returned here.
 */
void TransformAxis::
getAxis(SimTK::Vec3& rAxis) const
{
	rAxis = _axis;
}

//_____________________________________________________________________________
/**
 * Get the current value of the transform.
 *
 * @return Current value of the transform.
 */
double TransformAxis::getValue()
{
	if (_coordinate)
		return _function->evaluate(0, _coordinate->getValue(), 0.0, 0.0);
	else
		return _function->evaluate(0, 0.0, 0.0, 0.0);
}

