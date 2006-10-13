// SimmMarker.cpp
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
#include "SimmMarker.h"
#include "SimmKinematicsEngine.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

Geometry *SimmMarker::_defaultGeometry = AnalyticGeometry::createSphere(0.01);
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMarker::SimmMarker() :
   _weight(_weightProp.getValueDbl()),
   _attachment(_attachmentProp.getValueDblArray()),
	_fixed(_fixedProp.getValueBool()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())

{
	setNull();
	_displayer.setOwner(this);
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMarker::SimmMarker(DOMElement *aElement) :
   Object(aElement),
   _weight(_weightProp.getValueDbl()),
   _attachment(_attachmentProp.getValueDblArray()),
	_fixed(_fixedProp.getValueBool()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();

	updateFromXMLNode();
	_displayer.setOwner(this);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMarker::~SimmMarker()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarker SimmMarker to be copied.
 */
SimmMarker::SimmMarker(const SimmMarker &aMarker) :
   Object(aMarker),
   _weight(_weightProp.getValueDbl()),
   _attachment(_attachmentProp.getValueDblArray()),
	_fixed(_fixedProp.getValueBool()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
  _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setupProperties();
	copyData(aMarker);
	_displayer.setOwner(this);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMarker and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMarker.
 */
Object* SimmMarker::copy() const
{
	SimmMarker *marker = new SimmMarker(*this);
	return(marker);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMarker and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMarker::SimmMarker(DOMElement*) in order to establish the
 * relationship of the SimmMarker object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMarker object. Finally, the data members of the copy are
 * updated using SimmMarker::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMarker.
 */
Object* SimmMarker::copy(DOMElement *aElement) const
{
	SimmMarker *marker = new SimmMarker(aElement);
	*marker = *this;
	marker->updateFromXMLNode();
	return(marker);
}

void SimmMarker::copyData(const SimmMarker &aMarker)
{
	_attachment = aMarker._attachment;
	_weight = aMarker._weight;
	_fixed = aMarker._fixed;
	_bodyName = aMarker._bodyName;
	_displayer = aMarker._displayer;
	_virtual = aMarker._virtual;
}

/* Update an existing marker with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 */
void SimmMarker::updateFromMarker(const SimmMarker &aMarker)
{
	if (!aMarker._attachmentProp.getUseDefault())
	{
		_attachment = aMarker._attachment;
		_attachmentProp.setUseDefault(false);
	}

	if (!aMarker._weightProp.getUseDefault())
	{
		_weight = aMarker._weight;
		_weightProp.setUseDefault(false);
	}

	if (!aMarker._fixedProp.getUseDefault())
	{
		_fixed = aMarker._fixed;
		_fixedProp.setUseDefault(false);
	}

	if (!aMarker._bodyNameProp.getUseDefault())
	{
		_bodyName = aMarker._bodyName;
		_bodyNameProp.setUseDefault(false);
	}
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmMarker to their null values.
 */
void SimmMarker::setNull()
{
	setupProperties();
	setType("SimmMarker");
	setName("");
	setVirtual(true);
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMarker::setupProperties()
{
	_weightProp.setName("weight");
	_weightProp.setValue(0.0);
	_propertySet.append(&_weightProp);

	const double defaultAttachment[] = {0.0, 0.0, 0.0};
	_attachmentProp.setName("location");
	_attachmentProp.setValue(3, defaultAttachment);
	_propertySet.append(&_attachmentProp);

	_fixedProp.setName("fixed");
	_fixedProp.setValue(false);
	_propertySet.append(&_fixedProp);

	_bodyNameProp.setName("body");
	_propertySet.append(&_bodyNameProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);

}

SimmMarker& SimmMarker::operator=(const SimmMarker &aMarker)
{
	// BASE CLASS
	Object::operator=(aMarker);

	copyData(aMarker);

	return(*this);
}

void SimmMarker::getOffset(double *aOffset) const
{
	if (aOffset)
	{
		for (int i = 0; i < 3; i++)
			aOffset[i] = _attachment[i];
	}
}

void SimmMarker::setOffset(double aPoint[3])
{
	for (int i = 0; i < 3; i++)
		_attachment[i] = aPoint[i];
}

void SimmMarker::scale(Array<double>& aScaleFactors)
{
	for (int i = 0; i < 3; i++)
		_attachment[i] *= aScaleFactors[i];
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmMarker::setup(SimmKinematicsEngine* aEngine)
{
	getBodyName();
	if (_bodyName != ""){
		SimmBody* ownerBody = aEngine->getBody(_bodyName);
		VisibleObject* ownerBodyDisplayer;
		if (ownerBody && (ownerBodyDisplayer = ownerBody->getDisplayer())){
			ownerBodyDisplayer->addDependent(&_displayer);

		}
		_displayer.setOwner(this);
		_displayer.addGeometry(_defaultGeometry);
		if (isVirtual()){	// Pink
			double defaultColor[3] = { 1.0, 0.0, 0.8 };
			_displayer.getVisibleProperties().setColor(defaultColor);
		}
		else {	// Experimental is blue
			double defaultColor[3] = { 0.0, 0.0, 1.0 };
			_displayer.getVisibleProperties().setColor(defaultColor);
		}
	}

}

const string* SimmMarker::getBodyName() const
{
	if (_bodyNameProp.getUseDefault())
		return NULL;

	return &_bodyName;
}

void SimmMarker::writeSIMM(ofstream& out) const
{
	out << "marker " << getName() << '\t' << _attachment[0] << " " << _attachment[1] << " " << _attachment[2] << " " << _weight;
	if (_fixed)
		out << " fixed";
	out << endl;
}

void SimmMarker::peteTest() const
{
	cout << "   Marker: " << getName() << endl;
	cout << "      location: " << _attachment << endl;
	cout << "      weight: " << _weight << endl;
	cout << "      fixed: " << ((_fixed) ? ("true") : ("false")) << endl;
}

