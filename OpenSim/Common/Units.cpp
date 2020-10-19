/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Units.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
#include "Units.h"
#include "SimTKcommon/Constants.h"
#include "SimTKcommon/Scalar.h"

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
