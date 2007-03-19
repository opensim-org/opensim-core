// Units.cpp
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
#include "Units.h"
#include "rdMath.h"

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
Units::Units() :
	_type(simmUnknownUnits)
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aUnits Units to be copied.
 */
Units::Units(const Units& aUnits)
{
	_type = aUnits._type;
}

//_____________________________________________________________________________
/**
 * Constructor from a string.
 *
 * @param aString string containing the units text label
 */
Units::Units(string& aString) :
	_type(simmUnknownUnits)
{
   if (aString == "RADIANS" || aString == "RAD" || aString == "radians" || aString == "rad")
      _type = simmRadians;
   if (aString == "DEGREES" || aString == "DEG" || aString == "degrees" || aString == "deg")
      _type = simmDegrees;
   if (aString == "MM" || aString == "MILLIMETERS" || aString == "mm" || aString == "millimeters")
      _type = simmMillimeters;
   if (aString == "CM" || aString == "CENTIMETERS" || aString == "cm" || aString == "centimeters")
      _type = simmCentimeters;
   if (aString == "M" || aString == "METERS" || aString == "m" || aString == "meters")
      _type = simmMeters;
   if (aString == "SEC" || aString == "SECONDS" || aString == "sec" || aString == "seconds")
      _type = simmSeconds;
   if (aString == "MSEC" || aString == "MILLISECONDS" || aString == "msec" || aString == "milliseconds")
      _type = simmMilliseconds;
	if (aString == "N" || aString == "NEWTONS" || aString == "Newtons")
		_type = simmNewtons;
}

//_____________________________________________________________________________
/**
 * Constructor from a unit type.
 *
 * @param aType the unit type.
 */
Units::Units(UnitType aType)
{
	_type = aType;
}

//_____________________________________________________________________________
/**
 * Destructor
 */
Units::~Units()
{
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Convert a number from the units of this object to the units
 * of aType.
 *
 * @param aType the unit type to convert to
 * @param aValue the number to convert
 * @return The number converted to the new units
 */
double Units::convertTo(UnitType aType, double aValue) const
{
	return aValue * convertTo(aType);
}

//_____________________________________________________________________________
/**
 * Return a conversion factor for converting a number from the
 * units of this object to the units of aUnit.
 *
 * @param aUnit the units to convert to
 * @return The conversion factor
 */
double Units::convertTo(const Units& aUnit) const
{
	return convertTo(aUnit._type);
}

//_____________________________________________________________________________
/**
 * Return a conversion factor for converting a number from the
 * units of this object to the units of aType.
 *
 * @param aType the units to convert to
 * @return The conversion factor
 */
double Units::convertTo(UnitType aType) const
{
	if (_type == aType)
		return 1.0;

	if (_type == simmRadians)
	{
		if (aType == simmDegrees)
			return rdMath::RTD;
		else
			return rdMath::NAN;
	}
	else if (_type == simmDegrees)
	{
		if (aType == simmRadians)
			return rdMath::DTR;
		else
			return rdMath::NAN;
	}
	else if (_type == simmMillimeters)
	{
		if (aType == simmCentimeters)
			return 0.1;
		else if (aType == simmMeters)
			return 0.001;
		else
			return rdMath::NAN;
	}
	else if (_type == simmCentimeters)
	{
		if (aType == simmMillimeters)
			return 10.0;
		else if (aType == simmMeters)
			return 0.01;
		else
			return rdMath::NAN;
	}
	else if (_type == simmMeters)
	{
		if (aType == simmMillimeters)
			return 1000.0;
		else if (aType == simmCentimeters)
			return 100.0;
		else
			return rdMath::NAN;
	}
	else if (_type == simmSeconds)
	{
		if (aType == simmMilliseconds)
			return 1000.0;
		else
			return rdMath::NAN;
	}
	else if (_type == simmMilliseconds)
	{
		if (aType == simmSeconds)
			return 0.001;
		else
			return rdMath::NAN;
	}

	return rdMath::NAN;
}

//_____________________________________________________________________________
/**
 * Get the text label for the units.
 *
 * @return Pointer to the character string.
 */
const char* Units::getLabel() const
{
	switch(_type)
	{
	   case simmRadians:
		   return "radians";
	   case simmDegrees:
		   return "radians";
	   case simmMillimeters:
		   return "millimeters";
	   case simmCentimeters:
		   return "centimeters";
	   case simmMeters:
		   return "meters";
		case simmSeconds:
			return "seconds";
		case simmMilliseconds:
			return "milliseconds";
		case simmNewtons:
			return "N";
		case simmUnknownUnits:
		default:
			return "unknown";
	}
}

void Units::peteTest() const
{
	cout << "   Units: " << getLabel() << endl;
}

