// SimmMeasurement.cpp
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
#include "SimmMeasurement.h"

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
SimmMeasurement::SimmMeasurement() :
	_markerPairSetProp(PropertyObj("", SimmMarkerPairSet())),
	_markerPairSet((SimmMarkerPairSet&)_markerPairSetProp.getValueObj()),
	_bodyScaleSetProp(PropertyObj("", BodyScaleSet())),
	_bodyScaleSet((BodyScaleSet&)_bodyScaleSetProp.getValueObj()),
	_apply(_applyProp.getValueBool())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmMeasurement::SimmMeasurement(DOMElement *aElement) :
   Object(aElement),
	_markerPairSetProp(PropertyObj("", SimmMarkerPairSet())),
	_markerPairSet((SimmMarkerPairSet&)_markerPairSetProp.getValueObj()),
	_bodyScaleSetProp(PropertyObj("", BodyScaleSet())),
	_bodyScaleSet((BodyScaleSet&)_bodyScaleSetProp.getValueObj()),
	_apply(_applyProp.getValueBool())
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmMeasurement::~SimmMeasurement()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMeasurement SimmMeasurement to be copied.
 */
SimmMeasurement::SimmMeasurement(const SimmMeasurement &aMeasurement) :
   Object(aMeasurement),
	_markerPairSetProp(PropertyObj("", SimmMarkerPairSet())),
	_markerPairSet((SimmMarkerPairSet&)_markerPairSetProp.getValueObj()),
	_bodyScaleSetProp(PropertyObj("", BodyScaleSet())),
	_bodyScaleSet((BodyScaleSet&)_bodyScaleSetProp.getValueObj()),
	_apply(_applyProp.getValueBool())
{
	setupProperties();
	copyData(aMeasurement);
}
//_____________________________________________________________________________
/**
 * Copy this measurement and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmMeasurement.
 */
Object* SimmMeasurement::copy() const
{
	SimmMeasurement *measurement = new SimmMeasurement(*this);
	return(measurement);
}
//_____________________________________________________________________________
/**
 * Copy this SimmMeasurement and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmMeasurement::SimmMeasurement(DOMElement*) in order to establish the
 * relationship of the SimmMeasurement object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmMeasurement object. Finally, the data members of the
 * copy are updated using SimmMeasurement::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmMeasurement.
 */
Object* SimmMeasurement::copy(DOMElement *aElement) const
{
	SimmMeasurement *measurement = new SimmMeasurement(aElement);
	*measurement = *this;
	measurement->updateFromXMLNode();
	return(measurement);
}

void SimmMeasurement::copyData(const SimmMeasurement &aMeasurement)
{
	_markerPairSet = aMeasurement._markerPairSet;
	_bodyScaleSet = aMeasurement._bodyScaleSet;
	_apply = aMeasurement._apply;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmMeasurement to their null values.
 */
void SimmMeasurement::setNull()
{
	setupProperties();
	setType("SimmMeasurement");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmMeasurement::setupProperties()
{
	_applyProp.setComment("Flag to turn on and off scaling for this measurement.");
	_applyProp.setName("apply");
	_applyProp.setValue(true);
	_propertySet.append(&_applyProp);

	_markerPairSetProp.setComment("Set of marker pairs used to determine the scale factors.");
	_markerPairSetProp.setName("SimmMarkerPairSet");
	_propertySet.append(&_markerPairSetProp);

	_bodyScaleSetProp.setComment("Set of bodies to be scaled by this measurement.");
	_bodyScaleSetProp.setName("BodyScaleSet");
	_propertySet.append(&_bodyScaleSetProp);

}

void SimmMeasurement::registerTypes()
{
	Object::RegisterType(SimmMarkerPair());
	Object::RegisterType(BodyScale());
}

SimmMeasurement& SimmMeasurement::operator=(const SimmMeasurement &aMeasurement)
{
	// BASE CLASS
	Object::operator=(aMeasurement);

	copyData(aMeasurement);

	return(*this);
}

/* Apply a scale factor to a scale set, according to the elements of
 * the SimmMeasurement's BodyScaleSet.
 */
void SimmMeasurement::applyScaleFactor(double aFactor, ScaleSet& aScaleSet)
{
	for (int i = 0; i < _bodyScaleSet.getSize(); i++)
	{
		const string& bodyName = _bodyScaleSet[i]->getName();
		for (int j = 0; j < aScaleSet.getSize(); j++)
		{
			if (aScaleSet[j]->getSegmentName() == bodyName)
			{
				const Array<std::string>& axisNames = _bodyScaleSet[i]->getAxisNames();
				Array<double> factors(1.0, 3);
				aScaleSet[j]->getScaleFactors(factors);

				for (int k = 0; k < axisNames.getSize(); k++)
				{
					if (axisNames[k] == "x" || axisNames[k] == "X")
						factors[0] = aFactor;
					else if (axisNames[k] == "y" || axisNames[k] == "Y")
						factors[1] = aFactor;
					else if (axisNames[k] == "z" || axisNames[k] == "Z")
						factors[2] = aFactor;
				}
				aScaleSet[j]->setScaleFactors(factors);
			}
		}
	}
}

void SimmMeasurement::peteTest() const
{
	int i;

	cout << "      Measurement: " << getName() << endl;
	cout << "         apply: " << _apply << endl;
	for (i = 0; i < _markerPairSet.getSize(); i++)
		_markerPairSet[i]->peteTest();
	for (i = 0; i < _bodyScaleSet.getSize(); i++)
		_bodyScaleSet[i]->peteTest();
}
