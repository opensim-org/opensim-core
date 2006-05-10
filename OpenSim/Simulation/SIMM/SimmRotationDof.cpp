// SimmRotationDof.cpp
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
#include "SimmRotationDof.h"

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
SimmRotationDof::SimmRotationDof() :
   _axis(_axisProp.getValueDblArray())
{
	setNull();

}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmRotationDof::SimmRotationDof(DOMElement *aElement) :
   SimmDof(aElement),
   _axis(_axisProp.getValueDblArray())
{
	setNull();

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmRotationDof::~SimmRotationDof()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aDof SimmRotationDof to be copied.
 */
SimmRotationDof::SimmRotationDof(const SimmRotationDof &aDof) :
   SimmDof(aDof),
   _axis(_axisProp.getValueDblArray())
{
	setupProperties();
	copyData(aDof);
}
//_____________________________________________________________________________
/**
 * Copy this dof and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmRotationDof.
 */
Object* SimmRotationDof::copy() const
{
	SimmRotationDof *dof = new SimmRotationDof(*this);
	return(dof);
}
//_____________________________________________________________________________
/**
 * Copy this SimmRotationDof and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmRotationDof::SimmRotationDof(DOMElement*) in order to establish the
 * relationship of the SimmRotationDof object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmRotationDof object. Finally, the data members of the copy are
 * updated using SimmRotationDof::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmRotationDof.
 */
Object* SimmRotationDof::copy(DOMElement *aElement) const
{
	SimmRotationDof *dof = new SimmRotationDof(aElement);
	*dof = *this;
	dof->updateFromXMLNode();
	return(dof);
}

void SimmRotationDof::copyData(const SimmRotationDof &aDof)
{
	_axis = aDof._axis;
}

void SimmRotationDof::getAxis(double axis[3]) const
{
	axis[0] = _axis[0];
	axis[1] = _axis[1];
	axis[2] = _axis[2];
}

double SimmRotationDof::getValue()
{
	if (_coordinate)
		return _functions[0]->evaluate(0, _coordinate->getValue(), 0.0, 0.0);
	else
		return _functions[0]->evaluate(0, 0.0, 0.0, 0.0);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmRotationDof to their null values.
 */
void SimmRotationDof::setNull()
{
	setupProperties();
	setType("SimmRotationDof");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmRotationDof::setupProperties()
{
	const double defaultAxis[] = {1.0, 0.0, 0.0};
	_axisProp.setName("axis");
	_axisProp.setValue(3, defaultAxis);
	_propertySet.append(&_axisProp);
}

SimmRotationDof& SimmRotationDof::operator=(const SimmRotationDof &aDof)
{
	// BASE CLASS
	SimmDof::operator=(aDof);

	copyData(aDof);

	return(*this);
}

void SimmRotationDof::peteTest()
{
	cout << "RotationDof: " << getName() << endl;
	cout << "   value: " << getValue() << endl;
	cout << "   coordinate: " << _coordinateName << endl;
	if (_functions.getSize() > 0)
		cout << "   function: " << *(_functions[0]) << endl;
}
