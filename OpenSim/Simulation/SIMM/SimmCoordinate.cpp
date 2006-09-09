// SimmCoordinate.cpp
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
#include "SimmCoordinate.h"
#include "SimmJoint.h"
#include "SimmDof.h"
#include "SimmKinematicsEngine.h"
#include "simmIO.h"
#include "simmMacros.h"

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
SimmCoordinate::SimmCoordinate() :
   _defaultValue(_defaultValueProp.getValueDbl()),
   _valueStr(_valueStrProp.getValueStr()),
   _tolerance(_toleranceProp.getValueDbl()),
   _PDStiffness(_PDStiffnessProp.getValueDbl()),
   _IKweight(_IKweightProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_restraintFunction((ArrayPtrs<Function>&)_restraintFunctionProp.getValueObjArray()),
	_minRestraintFunction((ArrayPtrs<Function>&)_minRestraintFunctionProp.getValueObjArray()),
	_maxRestraintFunction((ArrayPtrs<Function>&)_maxRestraintFunctionProp.getValueObjArray()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_jointList(0),
	_pathList(0),
	_RTtype(SimmDof::Rotational),
	_value(0.0)
{
	setNull();

}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmCoordinate::SimmCoordinate(DOMElement *aElement) :
   Coordinate(aElement),
	_defaultValue(_defaultValueProp.getValueDbl()),
   _valueStr(_valueStrProp.getValueStr()),
   _tolerance(_toleranceProp.getValueDbl()),
   _PDStiffness(_PDStiffnessProp.getValueDbl()),
   _IKweight(_IKweightProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_restraintFunction((ArrayPtrs<Function>&)_restraintFunctionProp.getValueObjArray()),
	_minRestraintFunction((ArrayPtrs<Function>&)_minRestraintFunctionProp.getValueObjArray()),
	_maxRestraintFunction((ArrayPtrs<Function>&)_maxRestraintFunctionProp.getValueObjArray()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_jointList(0),
	_pathList(0),
	_RTtype(SimmDof::Rotational),
	_value(0.0)
{
	setNull();

	updateFromXMLNode();

	/* In case "value" was specified in the DOMelement
	 * (as a string), set the _value member accordingly.
	 */
	setValue(_valueStr);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmCoordinate::~SimmCoordinate()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aCoordinate SimmCoordinate to be copied.
 */
SimmCoordinate::SimmCoordinate(const SimmCoordinate &aCoordinate) :
   Coordinate(aCoordinate),
	_defaultValue(_defaultValueProp.getValueDbl()),
   _valueStr(_valueStrProp.getValueStr()),
   _tolerance(_toleranceProp.getValueDbl()),
   _PDStiffness(_PDStiffnessProp.getValueDbl()),
   _IKweight(_IKweightProp.getValueDbl()),
	_range(_rangeProp.getValueDblArray()),
	_keys(_keysProp.getValueStrArray()),
	_clamped(_clampedProp.getValueBool()),
	_locked(_lockedProp.getValueBool()),
	_restraintFunction((ArrayPtrs<Function>&)_restraintFunctionProp.getValueObjArray()),
	_minRestraintFunction((ArrayPtrs<Function>&)_minRestraintFunctionProp.getValueObjArray()),
	_maxRestraintFunction((ArrayPtrs<Function>&)_maxRestraintFunctionProp.getValueObjArray()),
	_restraintActive(_restraintActiveProp.getValueBool()),
	_jointList(0),
	_pathList(0),
	_RTtype(SimmDof::Rotational)
{
	setupProperties();
	copyData(aCoordinate);
}
//_____________________________________________________________________________
/**
 * Copy this coordinate and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmCoordinate.
 */
Object* SimmCoordinate::copy() const
{
	SimmCoordinate *gc = new SimmCoordinate(*this);
	return(gc);
}
//_____________________________________________________________________________
/**
 * Copy this SimmCoordinate and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmCoordinate::SimmCoordinate(DOMElement) in order to establish the
 * relationship of the SimmCoordinate object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmCoordinate object. Finally, the data members of the
 * copy are updated using SimmCoordinate::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmCoordinate.
 */
Object* SimmCoordinate::copy(DOMElement *aElement) const
{
	SimmCoordinate *gc = new SimmCoordinate(aElement);
	*gc = *this;
	gc->updateFromXMLNode();
	return(gc);
}

void SimmCoordinate::copyData(const SimmCoordinate &aCoordinate)
{
	_defaultValue = aCoordinate.getDefaultValue();
	_valueStr = aCoordinate._valueStr;
	_value = aCoordinate.getValue();
	_tolerance = aCoordinate.getTolerance();
	_PDStiffness = aCoordinate._PDStiffness;
	_IKweight = aCoordinate._IKweight;
	_range = aCoordinate._range;
	_keys = aCoordinate._keys;
	_clamped = aCoordinate._clamped;
	_locked = aCoordinate._locked;
	_restraintFunction = aCoordinate._restraintFunction;
	_minRestraintFunction = aCoordinate._minRestraintFunction;
	_maxRestraintFunction = aCoordinate._maxRestraintFunction;
	_restraintActive = aCoordinate._restraintActive;
	_jointList = aCoordinate._jointList;
	_pathList = aCoordinate._pathList;
	_RTtype = aCoordinate._RTtype;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmCoordinate to their null values.
 */
void SimmCoordinate::setNull(void)
{
	setupProperties();
	setType("SimmCoordinate");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmCoordinate::setupProperties(void)
{
	_defaultValueProp.setName("default_value");
	_defaultValueProp.setValue(0.0);
	_propertySet.append(&_defaultValueProp);

	_valueStrProp.setName("value");
	_propertySet.append(&_valueStrProp);

	_toleranceProp.setName("tolerance");
	_toleranceProp.setValue(0.0);
	_propertySet.append(&_toleranceProp);

	_PDStiffnessProp.setName("stiffness");
	_PDStiffnessProp.setValue(0.0);
	_propertySet.append(&_PDStiffnessProp);

	_IKweightProp.setName("weight");
	_IKweightProp.setValue(0.0);
	_propertySet.append(&_IKweightProp);

	const double defaultRange[] = {0.0, 90.0};
	_rangeProp.setName("range");
	_rangeProp.setValue(2, defaultRange);
	_propertySet.append(&_rangeProp);

	_keysProp.setName("keys");
	_propertySet.append(&_keysProp);

	_clampedProp.setName("clamped");
	_clampedProp.setValue(true);
	_propertySet.append(&_clampedProp);

	_lockedProp.setName("locked");
	_lockedProp.setValue(false);
	_propertySet.append(&_lockedProp);

	ArrayPtrs<Object> func;

	_restraintFunctionProp.setName("RestraintFunctions");
	_restraintFunctionProp.setValue(func);
	_propertySet.append(&_restraintFunctionProp);

	_minRestraintFunctionProp.setName("MinRestraintFunctions");
	_minRestraintFunctionProp.setValue(func);
	_propertySet.append(&_minRestraintFunctionProp);

	_maxRestraintFunctionProp.setName("MaxRestraintFunctions");
	_maxRestraintFunctionProp.setValue(func);
	_propertySet.append(&_maxRestraintFunctionProp);

	_restraintActiveProp.setName("restraint_active");
	_restraintActiveProp.setValue(true);
	_propertySet.append(&_restraintActiveProp);
}

SimmCoordinate& SimmCoordinate::operator=(const SimmCoordinate &aCoordinate)
{
	// BASE CLASS
	Coordinate::operator=(aCoordinate);

	copyData(aCoordinate);

	return(*this);
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmCoordinate::setup(SimmKinematicsEngine* aEngine)
{
	if (_defaultValue < _range[0])
		_defaultValue = _range[0];
	else if (_defaultValue > _range[1])
		_defaultValue = _range[1];

	/* If 'value' was not specified in the XML node, set the
	 * current value to the default value.
	 */
	if (_valueStrProp.getUseDefault())
		setValue(_defaultValue);

	determineType(aEngine);
}

/* This function tries to determine whether the coordinate is primarily rotational
 * or translational. This information is needed for two reasons:
 * 1. to know how big to set the step size for the arrow buttons on the slider for
 *    that gencoord in the Model Viewer (0.001 for translations, 1.0 for rotations).
 * 2. to know how to interpret the moment arm values w.r.t. that coordinate. If the
 *    coordinate maps directly to a rotational dof, then the coordinate's units need to
 *    be treated as degrees, and thus converted to radians when calculating moment arms.
 *    If the coordinate maps directly to a translational dof, then its units are meters
 *    and no conversion is needed. If the gencoord maps directly to a dof of each type,
 *    then it really is an abstract quantity, and interpreting the moment arm in
 *    real-world units is problematic. So just find the first dof that maps directly
 *    to it and go with that one.
 * To determine whether a coordinate is rotational or translational, find the first
 * dof that maps directly to the coordinate (has a function with two points and slope
 * equal to 1.0 or -1.0 and passes through zero). If there is no such function, then
 * just look at the dofs that the coordinate is used in. If they are all translational,
 * then the gencoord is translational. If one or more is rotational, then the coordinate
 * is rotational.
 */
void SimmCoordinate::determineType(SimmKinematicsEngine* aEngine)
{
   SimmDof* dof;

   if ((dof = aEngine->markUnconstrainedDof(this)))
   {
		_RTtype = dof->getDofType();
   }
   else
   {
		_RTtype = SimmDof::Translational;

      for (int i = 0; i < _jointList.getSize(); i++)
      {
			ArrayPtrs<SimmDof>& dofList = _jointList[i]->getDofs();

			for (int j = 0; j < dofList.getSize(); j++)
         {
				if (dofList[i]->getCoordinate() == this && dofList[i]->getDofType() == SimmDof::Rotational)
            {
					/* You've found one rotational DOF: set the coordinate's type
					 * to rotational and you're done.
					 */
               _RTtype = SimmDof::Rotational;
               return;
            }
         }
      }
   }
}

bool SimmCoordinate::setValue(double value)
{
	if (_locked)
	{
		cout << "___WARNING___: Coordinate " << getName() << " is locked. Unable to change its value." << endl;
		return false;
	}

	if (DABS(value - _value) > _tolerance &&
		 value >= _range[0] && value <= _range[1])
	{
		_value = value;

		int i;
		for (i = 0; i < _jointList.getSize(); i++)
			_jointList[i]->invalidate();

		int pListSize = _pathList.getSize();
		for (i = 0; i < pListSize; i++)
			_pathList[i]->invalidate();

		// Potential source of slowdown!
		/* Store the updated value as a string in the value property. */
		// TODO: use sprintf (recommended) instead of gcvt
#ifdef __linux__
		gcvt(value, 8, (char*)_valueStr.c_str());
#else
		_gcvt(value, 8, (char*)_valueStr.c_str());
#endif
	}

	return true;
}

bool SimmCoordinate::setValue(string& aValueStr)
{
	if (_locked && aValueStr != "fromFile" && aValueStr != "Unassigned")
	{
		cout << "___WARNING___: Coordinate " << getName() << " is locked. Unable to change its value." << endl;
		return false;
	}

	double value;
	string strCopy(aValueStr);

	if (readDoubleFromString(strCopy, &value))
	{
		return setValue(value);
	}
	else if (aValueStr != "fromFile" && aValueStr != "Unassigned")
	{
		cout << "___WARNING___: Unable to set coordinate " << getName() << " to value \"" << aValueStr << "\"" << endl;
		return false;
	}

	return true;
}

void SimmCoordinate::getKeys(string keys[]) const
{
	for (int i = 0; i < _keys.getSize(); i++)
		keys[i] = _keys[i];
}

Function* SimmCoordinate::getRestraintFunction(void) const
{
	if (_restraintFunction.getSize() < 1)
		return NULL;

	return _restraintFunction[0];
}

Function* SimmCoordinate::getMinRestraintFunction(void) const
{
	if (_minRestraintFunction.getSize() < 1)
		return NULL;

	return _minRestraintFunction[0];
}

Function* SimmCoordinate::getMaxRestraintFunction(void) const
{
	if (_maxRestraintFunction.getSize() < 1)
		return NULL;

	return _maxRestraintFunction[0];
}

/* Update an existing coordinate with parameter values from a
 * new one, but only for the parameters that were explicitly
 * specified in the XML node.
 */
void SimmCoordinate::updateFromCoordinate(const SimmCoordinate &aCoordinate)
{
	if (!aCoordinate._defaultValueProp.getUseDefault())
		_defaultValue = aCoordinate._defaultValue;

	if (!aCoordinate._valueStrProp.getUseDefault())
	{
		_valueStr = aCoordinate._valueStr;
		setValue(_valueStr);
	}

	if (!aCoordinate._toleranceProp.getUseDefault())
		_tolerance = aCoordinate._tolerance;

	if (!aCoordinate._PDStiffnessProp.getUseDefault())
		_PDStiffness = aCoordinate._PDStiffness;

	if (!aCoordinate._IKweightProp.getUseDefault())
		_IKweight = aCoordinate._IKweight;

	if (!aCoordinate._rangeProp.getUseDefault())
		_range = aCoordinate._range;

	if (!aCoordinate._keysProp.getUseDefault())
		_keys = aCoordinate._keys;

	if (!aCoordinate._clampedProp.getUseDefault())
		_clamped = aCoordinate._clamped;

	if (!aCoordinate._lockedProp.getUseDefault())
		_locked = aCoordinate._locked;

	if (!aCoordinate._restraintFunctionProp.getUseDefault())
		_restraintFunction = aCoordinate._restraintFunction;

	if (!aCoordinate._maxRestraintFunctionProp.getUseDefault())
		_maxRestraintFunction = aCoordinate._maxRestraintFunction;

	if (!aCoordinate._restraintActiveProp.getUseDefault())
		_restraintActive = aCoordinate._restraintActive;
}

void SimmCoordinate::writeSIMM(ofstream& out, int& aFunctionIndex) const
{
	int RFIndex = -1, minRFIndex = -1, maxRFIndex = -1;
	out << "begingencoord " << getName() << endl;
	out << "default_value " << _defaultValue << endl;
	out << "range " << _range[0] << " " << _range[1] << endl;
	out << "tolerance " << _tolerance << endl;
	/* SIMM41 out << "pd_stiffness " << _PDStiffness << endl; */
	if (_keys.getSize() > 0)
	{
		out << "keys ";
		for (int i = 0; i < MIN(2, _keys.getSize()); i++)
			out << _keys[i] << " ";
		out << endl;
	}
	out << "clamped " << ((_clamped) ? ("yes") : ("no")) << endl;
	out << "locked " << ((_locked) ? ("yes") : ("no")) << endl;
	out << "active " << ((_restraintActive) ? ("yes") : ("no")) << endl;
	if (_restraintFunction.getSize() > 0)
	{
		RFIndex = aFunctionIndex++;
		out << "restraint f" << RFIndex << endl;
	}
	if (_minRestraintFunction.getSize() > 0)
	{
		minRFIndex = aFunctionIndex++;
		out << "minrestraint f" << minRFIndex << endl;
	}
	if (_maxRestraintFunction.getSize() > 0)
	{
		maxRFIndex = aFunctionIndex++;
		out << "maxrestraint f" << maxRFIndex << endl;
	}
	out << "endgencoord" << endl << endl;

	if (RFIndex >= 0)
		_restraintFunction[0]->writeSIMM(out, RFIndex);
	if (minRFIndex >= 0)
		_minRestraintFunction[0]->writeSIMM(out, minRFIndex);
	if (maxRFIndex >= 0)
		_maxRestraintFunction[0]->writeSIMM(out, maxRFIndex);
}

void SimmCoordinate::peteTest(void) const
{
	cout << "Coordinate: " << getName() << endl;
	cout << "   default_value: " << _defaultValue << endl;
	cout << "   value: " << _value << endl;
	cout << "   tolerance: " << _tolerance << endl;
	cout << "   PDstiffness: " << _PDStiffness << endl;
	cout << "   IKweight: " << _IKweight << endl;
	cout << "   range: " << _range << endl;
	cout << "   keys: " << _keys << endl;
	cout << "   clamped: " << ((_clamped) ? ("true") : ("false")) << endl;
	cout << "   locked: " << ((_locked) ? ("true") : ("false")) << endl;
	if (_restraintFunction.getSize() > 0)
		cout << "   restraintFunction: " << *(_restraintFunction[0]) << endl;
	if (_minRestraintFunction.getSize() > 0)
		cout << "   minRestraintFunction: " << *(_minRestraintFunction[0]) << endl;
	if (_maxRestraintFunction.getSize() > 0)
		cout << "   maxRestraintFunction: " << *(_maxRestraintFunction[0]) << endl;
	cout << "   restraintActive: " << ((_restraintActive) ? ("true") : ("false")) << endl;
}

