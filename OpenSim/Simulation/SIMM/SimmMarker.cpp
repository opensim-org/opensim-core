// SimmMarker.cpp
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
#include "SimmMarker.h"
#include "AbstractModel.h"
#include "AbstractDynamicsEngine.h"
#include "BodySet.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

Geometry *SimmMarker::_defaultGeometry = AnalyticGeometry::createSphere(0.01);
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmMarker::SimmMarker() :
   _offset(_offsetProp.getValueDblArray()),
   _weight(_weightProp.getValueDbl()),
	_fixed(_fixedProp.getValueBool()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
	_displayer.setOwner(this);
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMarker::SimmMarker(DOMElement *aElement) :
   AbstractMarker(aElement),
   _offset(_offsetProp.getValueDblArray()),
   _weight(_weightProp.getValueDbl()),
	_fixed(_fixedProp.getValueBool()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
	setupProperties();
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
   AbstractMarker(aMarker),
   _offset(_offsetProp.getValueDblArray()),
   _weight(_weightProp.getValueDbl()),
	_fixed(_fixedProp.getValueBool()),
	_bodyName(_bodyNameProp.getValueStr()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj())
{
	setNull();
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

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmMarker to another.
 *
 * @param aMarker SimmMarker to be copied.
 */
void SimmMarker::copyData(const SimmMarker &aMarker)
{
	_offset = aMarker._offset;
	_weight = aMarker._weight;
	_fixed = aMarker._fixed;
	_bodyName = aMarker._bodyName;
	_displayer = aMarker._displayer;
	_virtual = aMarker._virtual;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmMarker to their null values.
 */
void SimmMarker::setNull()
{
	setType("SimmMarker");
	setVirtual(true);
	_body = 0;
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
	_offsetProp.setName("location");
	_offsetProp.setValue(3, defaultAttachment);
	_propertySet.append(&_offsetProp);

	_fixedProp.setName("fixed");
	_fixedProp.setValue(false);
	_propertySet.append(&_fixedProp);

	_bodyNameProp.setName("body");
	_propertySet.append(&_bodyNameProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);
}

//_____________________________________________________________________________
/**
 * Perform some set up functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aEngine dynamics engine containing this SimmMarker.
 */
void SimmMarker::setup(AbstractDynamicsEngine* aEngine)
{
	if (_bodyName != "")
	{
		AbstractBody *oldBody = _body;
		_body = aEngine->getBodySet()->get(_bodyName);

		if (!_body)
		{
			string errorMessage = "Error: Body " + _bodyName + " referenced in marker " + getName() +
				" does not exist in model " +	aEngine->getModel()->getName();
			throw Exception(errorMessage);
		}
		// If marker jumped between bodies we need to fix display stuff
		if (oldBody && oldBody != _body){
			oldBody->getDisplayer()->removeDependent(&_displayer);
		}
		//TODO: make code below safe to call multiple times (since this setup() method may be
		//called multiple times for a simm marker).  Should also try to handle case where a marker
		// is deleted (e.g. in AbstractDynamicsEngine::updateMarkerSet) because then may end up with
		// stale pointers.
		VisibleObject* ownerBodyDisplayer;
		if (_body && (ownerBodyDisplayer = _body->getDisplayer())){
			if(! ownerBodyDisplayer->hasDependent(&_displayer)){	// Only if first time to be encountered 
				ownerBodyDisplayer->addDependent(&_displayer);
				_displayer.addGeometry(_defaultGeometry);
			}
		}
		_displayer.setOwner(this);
		double defaultColor[3] = { 0.0, 0.0, 1.0 };
		_displayer.getVisibleProperties().setColor(defaultColor);
		updateGeometry();
			
	}
	else
	{
		string errorMessage = "Error: No body name specified for marker " + getName() + " in model " +
			aEngine->getModel()->getName();
		throw Exception(errorMessage);
	}
}
//_____________________________________________________________________________
/**
 * Remove self from the list of displayable objects and free resources
 */
void SimmMarker::removeSelfFromDisplay()
{
		VisibleObject* ownerBodyDisplayer;
		if (_body && (ownerBodyDisplayer = _body->getDisplayer())){
			if (ownerBodyDisplayer->hasDependent(&_displayer)){
				ownerBodyDisplayer->removeDependent(&_displayer);
			}
		}
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
SimmMarker& SimmMarker::operator=(const SimmMarker &aMarker)
{
	// BASE CLASS
	AbstractMarker::operator=(aMarker);

	copyData(aMarker);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Update an existing marker with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 *
 * @param aMarker marker to update from
 */
void SimmMarker::updateFromMarker(const AbstractMarker &aMarker)
{
	if (!aMarker.getOffsetUseDefault())
	{
		const double* off = aMarker.getOffset();
		setOffset(off);
		_offsetProp.setUseDefault(false);
	}

	if (!aMarker.getFixedUseDefault())
	{
		_fixed = aMarker.getFixed();
		_fixedProp.setUseDefault(false);
	}

	if (!aMarker.getWeightUseDefault())
	{
		_weight = aMarker.getWeight();
		_weightProp.setUseDefault(false);
	}

	if (!aMarker.getBodyNameUseDefault())
	{	
		_bodyName = *aMarker.getBodyName();
		_bodyNameProp.setUseDefault(false);
	}
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the marker's XYZ offset from the body it's attached to.
 *
 * @param rOffset XYZ offset is returned here.
 */
void SimmMarker::getOffset(double *rOffset) const
{
	if (rOffset)
	{
		for (int i = 0; i < 3; i++)
			rOffset[i] = _offset[i];
	}
}

//_____________________________________________________________________________
/**
 * Set the marker's XYZ offset from the body it's attached to.
 *
 * @param aOffset XYZ offset to set to.
 * @return Whether or not the offset was set.
 */
bool SimmMarker::setOffset(Array<double>& aOffset)
{
	_offset[0] = aOffset[0];
	_offset[1] = aOffset[1];
	_offset[2] = aOffset[2];

	updateGeometry();
	return true;
}

//_____________________________________________________________________________
/**
 * Set the marker's XYZ offset from the body it's attached to.
 *
 * @param aPoint XYZ offset to set to.
 * @return Whether or not the offset was set.
 */
bool SimmMarker::setOffset(const double aPoint[3])
{
	for (int i = 0; i < 3; i++)
		_offset[i] = aPoint[i];

	updateGeometry();
	return true;
}

//_____________________________________________________________________________
/**
 * Set the marker's fixed status.
 *
 * @param aFixed boolean value to set to.
 * @return Whether or not the fixed status was set.
 */
bool SimmMarker::setFixed(bool aFixed)
{
	_fixed = aFixed;
	return true;
}

//_____________________________________________________________________________
/**
 * Set the marker's weight.
 *
 * @param aWeight weight to set to.
 * @return Whether or not the weight was set.
 */
bool SimmMarker::setWeight(double aWeight)
{
	if (aWeight >= 0.0)
	{
		_weight = aWeight;
		return true;
	}

	return false;
}

//_____________________________________________________________________________
/**
 * Set the 'body name' field, which is used when the marker is added to
 * an existing model.
 *
 * @param aName name of body
 * @return Whether or not the body name was set.
 */
bool SimmMarker::setBodyName(const string& aName)
{
	_bodyName = aName;

	return true;
}

//_____________________________________________________________________________
/**
 * Get the 'body name' field, which is used when the marker is added to
 * an existing model.
 *
 * @return Pointer to the body name.
 */
const string* SimmMarker::getBodyName() const
{
	if (_bodyNameProp.getUseDefault())
		return NULL;

	return &_bodyName;
}

//_____________________________________________________________________________
/**
 * Set the 'body name' field to use or not use the default value.
 *
 * @return Whether or not the flag was set properly.
 */
bool SimmMarker::setBodyNameUseDefault(bool aValue)
{
	_bodyNameProp.setUseDefault(aValue);

	return true;
}

//_____________________________________________________________________________
/**
 * Set the body that this marker is attached to.
 *
 * @param aBody Pointer to the body.
 */
void SimmMarker::setBody(AbstractBody* aBody)
{
	_body = aBody;
	// TODO: should body and bodyName be synced when either one is changed?
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Scale the marker.
 *
 * @param aScaleFactors XYZ scale factors.
 */
void SimmMarker::scale(const Array<double>& aScaleFactors)
{
	for (int i = 0; i < 3; i++)
		_offset[i] *= aScaleFactors[i];
}

//_____________________________________________________________________________
/**
 * Update the geometry to correspond to position changes
 */
void SimmMarker::updateGeometry()
{
	Transform position;
	position.translate(getOffset());
	getDisplayer()->setTransform(position);

}
void SimmMarker::peteTest() const
{
	cout << "   Marker: " << getName() << endl;
	cout << "      location: " << _offset << endl;
	cout << "      weight: " << _weight << endl;
	cout << "      fixed: " << ((_fixed) ? ("true") : ("false")) << endl;
}
