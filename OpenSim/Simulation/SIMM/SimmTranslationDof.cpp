// SimmTranslationDof.cpp
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
#include "SimmTranslationDof.h"

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
SimmTranslationDof::SimmTranslationDof()
{
	setNull();

	_axis[0] = _axis[1] = _axis[2] = 0.0;
	_axisIndex = xTranslation;
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmTranslationDof::SimmTranslationDof(DOMElement *aElement) :
   SimmDof(aElement)
{
	setNull();

	_axis[0] = _axis[1] = _axis[2] = 0.0;
	_axisIndex = xTranslation;

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmTranslationDof::~SimmTranslationDof()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof SimmTranslationDof to be copied.
 */
SimmTranslationDof::SimmTranslationDof(const SimmTranslationDof &aDof) :
   SimmDof(aDof)
{
	setupProperties();
	copyData(aDof);
}
//_____________________________________________________________________________
/**
 * Copy this dof and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmTranslationDof.
 */
Object* SimmTranslationDof::copy() const
{
	SimmTranslationDof *dof = new SimmTranslationDof(*this);
	return(dof);
}
//_____________________________________________________________________________
/**
 * Copy this SimmTranslationDof and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmTranslationDof::SimmTranslationDof(DOMElement*) in order to establish the
 * relationship of the SimmTranslationDof object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmTranslationDof object. Finally, the data members of the
 * copy are updated using SimmTranslationDof::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmTranslationDof.
 */
Object* SimmTranslationDof::copy(DOMElement *aElement) const
{
	SimmTranslationDof *dof = new SimmTranslationDof(aElement);
	*dof = *this;
	dof->updateFromXMLNode();
	return(dof);
}

void SimmTranslationDof::copyData(const SimmTranslationDof &aDof)
{
	aDof.getAxis(_axis);
	_axisIndex = aDof._axisIndex;
}

void SimmTranslationDof::updateFromXMLNode()
{
	Object::updateFromXMLNode();

	if (_name == TX_NAME)
	{
		_axis[0] = 1.0;
		_axisIndex = xTranslation;
	}
	else if (_name == TY_NAME)
	{
		_axis[1] = 1.0;
		_axisIndex = yTranslation;
	}
	else if (_name == TZ_NAME)
	{
		_axis[2] = 1.0;
		_axisIndex = zTranslation;
	}
}

double SimmTranslationDof::getValue()
{
	if (_coordinate)
		return _functions[0]->evaluate(0, _coordinate->getValue(), 0.0, 0.0);
	else
		return _functions[0]->evaluate(0, 0.0, 0.0, 0.0);
}

void SimmTranslationDof::getAxis(double axis[3]) const
{
	if (axis != NULL)
	{
		axis[0] = _axis[0];
		axis[1] = _axis[1];
		axis[2] = _axis[2];
	}
}

void SimmTranslationDof::getTranslation(double vec[4])
{
	double value = getValue();

	vec[0] = _axis[0] * value;
	vec[1] = _axis[1] * value;
	vec[2] = _axis[2] * value;
	vec[3] = 1.0;
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmTranslationDof to their null values.
 */
void SimmTranslationDof::setNull()
{
	setupProperties();
	setType("SimmTranslationDof");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmTranslationDof::setupProperties()
{
}

SimmTranslationDof& SimmTranslationDof::operator=(const SimmTranslationDof &aDof)
{
	// BASE CLASS
	SimmDof::operator=(aDof);

	copyData(aDof);

	return(*this);
}

void SimmTranslationDof::peteTest()
{
	cout << "TranslationDof: " << getName() << endl;
	double v[4];
	getTranslation(v);
	cout << "   trans: " << v[0] << ", " << v[1] << ", " << v[2] << endl;
	cout << "   axisIndex: " << _axisIndex << endl;
	cout << "   coordinate: " << _coordinateName << endl;
	if (_functions.getSize() > 0)
		cout << "   function: " << *(_functions[0]) << endl;
}

