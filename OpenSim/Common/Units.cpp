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
#include "rdMath.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
// DTOR and RTOD are defined in rdMath.cpp, but they use PI, which is defined
// in that file as acos(-1.0). But maybe because of a prior definition it ends
// up truncated as 3.1415926538, which is not accurate enough for very small
// angles. So in this file DTOR and RTOD are replaced with these:
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
			return RAD_TO_DEG;
		else
			return rdMath::NAN;
	}
	else if (_type == simmDegrees)
	{
		if (aType == simmRadians)
			return DEG_TO_RAD;
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
 * @return String label.
 */
string Units::getLabel() const
{
	switch(_type)
	{
	   case simmRadians:
		   return "radians";
	   case simmDegrees:
		   return "degrees";
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
/**
 * Get the text label for the units.
 *
 * @return String label.
 */
string Units::getAbbreviation() const
{
	switch(_type)
	{
	   case simmRadians:
		   return "rad";
	   case simmDegrees:
		   return "deg";
	   case simmMillimeters:
		   return "mm";
	   case simmCentimeters:
		   return "cm";
	   case simmMeters:
		   return "m";
		case simmSeconds:
			return "s";
		case simmMilliseconds:
			return "ms";
		case simmNewtons:
			return "N";
		case simmUnknownUnits:
		default:
			return "unknown";
	}
}
