// Units.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved.
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "Units.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

#define DEG_TO_RAD  0.017453292519943295769
#define RAD_TO_DEG 57.295779513082320876846

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Units::Units() :
	_type(UnknownUnits)
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
Units::Units(const string aString) :
	_type(UnknownUnits)
{
   if (aString == "RADIANS" || aString == "RAD" || aString == "radians" || aString == "rad")
      _type = Radians;
   if (aString == "DEGREES" || aString == "DEG" || aString == "degrees" || aString == "deg")
      _type = Degrees;
   if (aString == "MM" || aString == "MILLIMETERS" || aString == "mm" || aString == "millimeters")
      _type = Millimeters;
   if (aString == "CM" || aString == "CENTIMETERS" || aString == "cm" || aString == "centimeters")
      _type = Centimeters;
   if (aString == "M" || aString == "METERS" || aString == "m" || aString == "meters")
      _type = Meters;
   if (aString == "SEC" || aString == "SECONDS" || aString == "sec" || aString == "seconds")
      _type = Seconds;
   if (aString == "MSEC" || aString == "MILLISECONDS" || aString == "msec" || aString == "milliseconds")
      _type = Milliseconds;
	if (aString == "N" || aString == "NEWTONS" || aString == "Newtons")
		_type = Newtons;
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

	if (_type == Radians)
	{
		if (aType == Degrees)
			return SimTK_RADIAN_TO_DEGREE;
		else
			return SimTK::NaN;
	}
	else if (_type == Degrees)
	{
		if (aType == Radians)
			return SimTK_DEGREE_TO_RADIAN;
		else
			return SimTK::NaN;
	}
	else if (_type == Millimeters)
	{
		if (aType == Centimeters)
			return 0.1;
		else if (aType == Meters)
			return 0.001;
		else
			return SimTK::NaN;
	}
	else if (_type == Centimeters)
	{
		if (aType == Millimeters)
			return 10.0;
		else if (aType == Meters)
			return 0.01;
		else
			return SimTK::NaN;
	}
	else if (_type == Meters)
	{
		if (aType == Millimeters)
			return 1000.0;
		else if (aType == Centimeters)
			return 100.0;
		else
			return SimTK::NaN;
	}
	else if (_type == Seconds)
	{
		if (aType == Milliseconds)
			return 1000.0;
		else
			return SimTK::NaN;
	}
	else if (_type == Milliseconds)
	{
		if (aType == Seconds)
			return 0.001;
		else
			return SimTK::NaN;
	}

	return SimTK::NaN;
}

//_____________________________________________________________________________
/**
 * Get the text label for the units.
 *
 * @return String label.
 */
string Units::getLabel() const
{
	switch(_type)
	{
	   case Radians:
		   return "radians";
	   case Degrees:
		   return "degrees";
	   case Millimeters:
		   return "millimeters";
	   case Centimeters:
		   return "centimeters";
	   case Meters:
		   return "meters";
		case Seconds:
			return "seconds";
		case Milliseconds:
			return "milliseconds";
		case Newtons:
			return "N";
		case UnknownUnits:
		default:
			return "unknown";
	}
}
/**
 * Get the text label for the units.
 *
 * @return String label.
 */
string Units::getAbbreviation() const
{
	switch(_type)
	{
	   case Radians:
		   return "rad";
	   case Degrees:
		   return "deg";
	   case Millimeters:
		   return "mm";
	   case Centimeters:
		   return "cm";
	   case Meters:
		   return "m";
		case Seconds:
			return "s";
		case Milliseconds:
			return "ms";
		case Newtons:
			return "N";
		case UnknownUnits:
		default:
			return "unknown";
	}
}
