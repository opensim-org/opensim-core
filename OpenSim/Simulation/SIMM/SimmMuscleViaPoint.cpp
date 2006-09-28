// SimmMuscleViaPoint.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
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
#include "SimmMuscleViaPoint.h"
#include "SimmModel.h"
#include "SimmKinematicsEngine.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMuscleViaPoint::SimmMuscleViaPoint() :
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();

}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMuscleViaPoint::SimmMuscleViaPoint(DOMElement *aElement) :
   SimmMusclePoint(aElement),
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setNull();

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMuscleViaPoint::~SimmMuscleViaPoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint SimmMuscleViaPoint to be copied.
 */
SimmMuscleViaPoint::SimmMuscleViaPoint(const SimmMuscleViaPoint &aPoint) :
   SimmMusclePoint(aPoint),
   _range(_rangeProp.getValueDblArray()),
	_coordinateName(_coordinateNameProp.getValueStr()),
	_coordinate(NULL)
{
	setupProperties();
	copyData(aPoint);
}
//_____________________________________________________________________________
/**
 * Copy this muscle via point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMuscleViaPoint.
 */
Object* SimmMuscleViaPoint::copy() const
{
	SimmMuscleViaPoint *pt = new SimmMuscleViaPoint(*this);
	return(pt);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMuscleViaPoint and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMuscleViaPoint::SimmMuscleViaPoint(DOMElement*) in order to establish the
 * relationship of the SimmMuscleViaPoint object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMuscleViaPoint object. Finally, the data members of the copy are
 * updated using SimmMuscleViaPoint::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMuscleViaPoint.
 */
Object* SimmMuscleViaPoint::copy(DOMElement *aElement) const
{
	SimmMuscleViaPoint *pt = new SimmMuscleViaPoint(aElement);
	*pt = *this;
	pt->updateFromXMLNode();
	return(pt);
}

void SimmMuscleViaPoint::copyData(const SimmMuscleViaPoint &aPoint)
{
	_range = aPoint._range;
	_coordinateName = aPoint._coordinateName;
	_coordinate = aPoint._coordinate;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMuscleViaPoint to their null values.
 */
void SimmMuscleViaPoint::setNull()
{
	setupProperties();
	setType("SimmMuscleViaPoint");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMuscleViaPoint::setupProperties()
{
	const double defaultRange[] = {0.0, 0.0};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_propertySet.append(&_rangeProp);

	_coordinateNameProp.setName("coordinate");
	_propertySet.append(&_coordinateNameProp);
}

SimmMuscleViaPoint& SimmMuscleViaPoint::operator=(const SimmMuscleViaPoint &aPoint)
{
	// BASE CLASS
	SimmMusclePoint::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void SimmMuscleViaPoint::writeSIMM(ofstream& out) const
{
	out << _attachment[0] << " " << _attachment[1] << " " << _attachment[2] << " segment " << _bodyName;
	out << " ranges 1 " << _coordinateName << " (" << _range[0] << ", " << _range[1] << ")" << endl;
}

//_____________________________________________________________________________
/**
 * Determine if this point is active by checking the value of the
 * coordinate that it is linked to.
 *
 * @return Whether or not this point is active.
 */
bool SimmMuscleViaPoint::isActive() const
{
	if (_coordinate)
	{
		double value = _coordinate->getValue();
		if (value >= _range[0] && value <= _range[1])
			return true;
	}

	return false;
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmMuscleViaPoint::setup(SimmModel* model, SimmKinematicsEngine* ke)
{
	// base class
	SimmMusclePoint::setup(model, ke);

	/* Look up the coordinate by name in the kinematics engine and
	 * store a pointer to it.
	 */
	_coordinate = dynamic_cast<SimmCoordinate *> (ke->getCoordinate(_coordinateName));
}

void SimmMuscleViaPoint::peteTest() const
{
	cout << "   MuscleViaPoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	cout << "      body: " << _body->getName() << endl;
	cout << "      range: " << getRange() << endl;
	cout << "      coordinate: " << _coordinate->getName() << endl;
}
