// SimmBody.cpp
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmBody.h"
#include "AbstractDynamicsEngine.h"
#include "SimmMacros.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmBody::SimmBody() :
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmBody::SimmBody(DOMElement *aElement) :
   AbstractBody(aElement),
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmBody::~SimmBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody SimmBody to be copied.
 */
SimmBody::SimmBody(const SimmBody &aBody) :
   AbstractBody(aBody),
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aBody);
}

//_____________________________________________________________________________
/**
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmBody.
 */
Object* SimmBody::copy() const
{
	SimmBody *body = new SimmBody(*this);
	return(body);
}

//_____________________________________________________________________________
/**
 * Copy this SimmBody and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmBody::SimmBody(DOMElement*) in order to establish the
 * relationship of the SimmBody object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmBody object. Finally, the data members of the copy are
 * updated using SimmBody::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmBody.
 */
Object* SimmBody::copy(DOMElement *aElement) const
{
	SimmBody *body = new SimmBody(aElement);
	*body = *this;
	body->updateFromXMLNode();
	return(body);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmBody to another.
 *
 * @param aBody SimmBody to be copied.
 */
void SimmBody::copyData(const SimmBody &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertia = aBody._inertia;
	_displayer = aBody._displayer; //? Do we need a dep copy here? when is this invoked?
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmBody to their null values.
 */
void SimmBody::setNull()
{
	setType("SimmBody");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmBody::setupProperties()
{
	_massProp.setName("mass");
	_massProp.setValue(0.0);
	_propertySet.append(&_massProp);

	const double defaultMC[] = {0.0, 0.0, 0.0};
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(3, defaultMC);
	_propertySet.append(&_massCenterProp);

	const double defaultInertia[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	_inertiaProp.setName("inertia");
	_inertiaProp.setValue(9, defaultInertia);
	_propertySet.append(&_inertiaProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmBody.
 */
void SimmBody::setup(AbstractDynamicsEngine* aEngine)
{
	// Base class
	AbstractBody::setup(aEngine);

	int i;
	for (i = 0; i < _displayer.getNumGeometryFiles(); i++)
		_displayer.addGeometry(new PolyhedralGeometry("bones/"+_displayer.getGeometryFileName(i)));

	_displayer.setOwner(this);
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
SimmBody& SimmBody::operator=(const SimmBody &aBody)
{
	// BASE CLASS
	AbstractBody::operator=(aBody);

	copyData(aBody);

	return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the mass of the body.
 *
 * @param aMass mass of body.
 * @return Whether mass was successfully changed.
 */
bool SimmBody::setMass(double aMass)
{
	if (aMass >= 0.0)
	{
		_mass = aMass;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Get the mass center of the body.
 *
 * @param rVec XYZ coordinates of mass center are returned here.
 */
void SimmBody::getMassCenter(double rVec[3]) const
{
	rVec[0] = _massCenter[0];
	rVec[1] = _massCenter[1];
	rVec[2] = _massCenter[2];
}
 
//_____________________________________________________________________________
/**
 * Set the mass center of the body.
 *
 * @param aVec XYZ coordinates of mass center.
 * @return Whether mass center was successfully changed.
 */
bool SimmBody::setMassCenter(double aVec[3])
{
	_massCenter[0] = aVec[0];
	_massCenter[1] = aVec[1];
	_massCenter[2] = aVec[2];

	return true;
}

//_____________________________________________________________________________
/**
 * Get the inertia matrix of the body.
 *
 * @param rInertia 3x3 inertia matrix.
 */
void SimmBody::getInertia(double rInertia[3][3]) const
{
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			rInertia[i][j] = _inertia[i*3 + j];
}

//_____________________________________________________________________________
/**
 * Set the inertia matrix of the body.
 *
 * @param aInertia 9-element inertia matrix.
 * @return Whether inertia matrix was successfully changed.
 */
bool SimmBody::setInertia(const Array<double>& aInertia)
{
	if (aInertia.getSize() >= 9)
	{
		for (int i = 0; i < 9; i++)
			_inertia[i] = aInertia[i];

		return true;
	}

	return false;
}

//=============================================================================
// BONES
//=============================================================================
//_____________________________________________________________________________
/**
 * Add a bone to the body.
 *
 * @param aBone bone to be added.
void SimmBody::addBone(VisibleObject* aBone)
{
	VisibleObject* newBone = new VisibleObject(*aBone);

	// note: _boneSet takes over ownership of newBone
	_boneSet.append(newBone);
}
 */

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the body.
 *
 * @param aScaleFactors XYZ scale factors.
 * @param aScaleMass whether or not to scale mass properties
 */
void SimmBody::scale(Array<double>& aScaleFactors, bool aScaleMass)
{
	int i;

	double oldScaleFactors[3];
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for (i = 0; i < 3; i++)
	{
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(aScaleFactors.get());

	if (aScaleMass)
		scaleInertialProperties(aScaleFactors);

	//for (i = 0; i < _boneSet.getSize(); i++)
		//_boneSet.get(i)->scale(aScaleFactors);
}

//_____________________________________________________________________________
/**
 * Scale the body's mass and inertia.
 *
 * @param aScaleFactors XYZ scale factors.
 */
void SimmBody::scaleInertialProperties(Array<double>& aScaleFactors)
{
	int i;

	/* If the mass is zero, then make the inertia tensor zero as well.
	 * If the X, Y, Z scale factors are equal, then you can scale the
	 * inertia tensor exactly by the square of the scale factor, since
	 * each element in the tensor is proportional to the square of one
	 * or more dimensional measurements. For determining if the scale
	 * factors are equal, ignore reflections-- look only at the
	 * absolute value of the factors.
	 */
	if (_mass <= ROUNDOFF_ERROR)
	{
		for (i = 0; i < 9; i++)
			_inertia[i] = 0.0;
	}
	else if (EQUAL_WITHIN_ERROR(DABS(aScaleFactors[0]), DABS(aScaleFactors[1])) &&
		      EQUAL_WITHIN_ERROR(DABS(aScaleFactors[1]), DABS(aScaleFactors[2])))
	{
		_mass *= DABS((aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]));

		for (i = 0; i < 9; i++)
			_inertia[i] *= (aScaleFactors[0] * aScaleFactors[0]);
	}
	else
	{
		/* If the scale factors are not equal, then assume that the segment
		 * is a cylinder and the inertia is calculated about one end of it.
		 */
		int axis;

		/* 1. Find the smallest diagonal component. This dimension is the axis
		 *    of the cylinder.
		 */
		if (_inertia[0] <= _inertia[4])
		{
			if (_inertia[0] <= _inertia[8])
				axis = 0;
			else
				axis = 2;
		}
		else if (_inertia[4] <= _inertia[8])
		{
			axis = 1;
		}
		else
		{
			axis = 2;
		}

		/* 2. The smallest inertial component is equal to 0.5 * mass * radius * radius,
		 *    so you can rearrange and solve for the radius.
		 */
		int oa;
		double radius, rad_sqr, length;
		double term = 2.0 * _inertia[axis * 3 + axis] / _mass;
		if (term < 0.0)
			radius = 0.0;
		else
			radius = sqrt(term);

		/* 3. Choose either of the other diagonal components and use it to solve for the
		*    length of the cylinder. This component is equal to:
		*    0.333 * mass * length * length  +  0.25 * mass * radius * radius
		*/
		if (axis == 0)
			oa = 1;
		else
			oa = 0;
		term = 3.0 * (_inertia[oa * 3 + oa] - 0.25 * _mass * radius * radius) / _mass;
		if (term < 0.0)
			length = 0.0;
		else
			length = sqrt(term);

		/* 4. Scale the mass, radius, and length, and recalculate the diagonal inertial terms. */
		_mass *= DABS((aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]));
		length *= DABS(aScaleFactors[axis]);

		if (axis == 0)
		{
			rad_sqr = radius * DABS(aScaleFactors[1]) * radius * DABS(aScaleFactors[2]);
			_inertia[0] = 0.5 * _mass * rad_sqr;
			_inertia[4] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[8] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
		}
		else if (axis == 1)
		{
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[2]);
			_inertia[0] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[4] = 0.5 * _mass * rad_sqr;
			_inertia[8] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
		}
		else
		{
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[1]);
			_inertia[0] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[4] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[8] = 0.5 * _mass * rad_sqr;
		}

		/* 5. Scale the cross terms, in case some are non-zero. */
		_inertia[1] *= DABS((aScaleFactors[0] * aScaleFactors[1]));
		_inertia[2] *= DABS((aScaleFactors[0] * aScaleFactors[2]));
		_inertia[3] *= DABS((aScaleFactors[1] * aScaleFactors[0]));
		_inertia[5] *= DABS((aScaleFactors[1] * aScaleFactors[2]));
		_inertia[6] *= DABS((aScaleFactors[2] * aScaleFactors[0]));
		_inertia[7] *= DABS((aScaleFactors[2] * aScaleFactors[1]));
	}
}

//=============================================================================
// ITERATORS FOR COMPONENTS
//=============================================================================
//_____________________________________________________________________________
/**
 * Make an iterator for the body's bone set.
 *
 * @return Pointer to the bone iterator.
BoneIterator* SimmBody::newBoneIterator() const
{
	return new BoneSetIterator(_boneSet);
}
 */

//=============================================================================
// I/O
//=============================================================================
void SimmBody::getScaleFactors(Array<double>& scales) const
{

	double scaleFactors[3];
	_displayer.getScaleFactors(scaleFactors);

	for (int i=0; i<3; i++)
		scales[i] = scaleFactors[i];

}

void SimmBody::peteTest() const
{
	cout << "Body: " << getName() << endl;
	cout << "   mass: " << _mass << endl;
	cout << "   massCenter: " << _massCenter << endl;
	cout << "   inertia: " << _inertia << endl;

	if (_wrapObjectSet.getSize() > 0) {
		int i;
		for (i = 0; i < _wrapObjectSet.getSize(); i++)
			_wrapObjectSet.get(i)->peteTest();
	} else {
		cout << "   no wrap objects" << endl;
	}
}
