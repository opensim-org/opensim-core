// SimmMusclePoint.cpp
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
#include "SimmMusclePoint.h"
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
SimmMusclePoint::SimmMusclePoint() :
   _attachment(_attachmentProp.getValueDblArray()),
	_bodyName(_bodyNameProp.getValueStr())
{
	setNull();

}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMusclePoint::SimmMusclePoint(DOMElement *aElement) :
   VisibleObject(aElement),
   _attachment(_attachmentProp.getValueDblArray()),
	_bodyName(_bodyNameProp.getValueStr())
{
	setNull();

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMusclePoint::~SimmMusclePoint()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aPoint SimmMusclePoint to be copied.
 */
SimmMusclePoint::SimmMusclePoint(const SimmMusclePoint &aPoint) :
   VisibleObject(aPoint),
   _attachment(_attachmentProp.getValueDblArray()),
	_bodyName(_bodyNameProp.getValueStr())
{
	setupProperties();
	copyData(aPoint);
}
//_____________________________________________________________________________
/**
 * Copy this muscle point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMusclePoint.
 */
Object* SimmMusclePoint::copy() const
{
	SimmMusclePoint *pt = new SimmMusclePoint(*this);
	return(pt);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMusclePoint and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMusclePoint::SimmMusclePoint(DOMElement*) in order to establish the
 * relationship of the SimmMusclePoint object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMusclePoint object. Finally, the data members of the
 * copy are updated using SimmMusclePoint::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMusclePoint.
 */
Object* SimmMusclePoint::copy(DOMElement *aElement) const
{
	SimmMusclePoint *pt = new SimmMusclePoint(aElement);
	*pt = *this;
	pt->updateFromXMLNode();
	return(pt);
}

void SimmMusclePoint::copyData(const SimmMusclePoint &aPoint)
{
	_attachment = aPoint._attachment;
	_bodyName = aPoint._bodyName;
	_body = aPoint._body;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMusclePoint to their null values.
 */
void SimmMusclePoint::setNull()
{
	setupProperties();
	setType("SimmMusclePoint");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMusclePoint::setupProperties()
{
	const double defaultAttachment[] = {0.0, 0.0, 0.0};
	_attachmentProp.setName("location");
	_attachmentProp.setValue(3, defaultAttachment);
	_propertySet.append(&_attachmentProp);

	_bodyNameProp.setName("body");
	_propertySet.append(&_bodyNameProp);
}

SimmMusclePoint& SimmMusclePoint::operator=(const SimmMusclePoint &aPoint)
{
	// BASE CLASS
	VisibleObject::operator=(aPoint);

	copyData(aPoint);

	return(*this);
}

void SimmMusclePoint::scale(Array<double>& aScaleFactors)
{
	for (int i = 0; i < 3; i++)
		_attachment[i] *= aScaleFactors[i];
}

void SimmMusclePoint::writeSIMM(ofstream& out) const
{
	out << _attachment[0] << " " << _attachment[1] << " " << _attachment[2] << " segment " << _bodyName << endl;
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmMusclePoint::setup(SimmModel* model, SimmKinematicsEngine* ke)
{
	/* Look up the body by name in the kinematics engine and
	 * store a pointer to it.
	 */
	_body = ke->getBody(_bodyName);
}

void SimmMusclePoint::peteTest() const
{
	cout << "   MusclePoint: " << getName() << endl;
	cout << "      point: " << getAttachment() << endl;
	cout << "      body: " << _body->getName() << endl;
}
