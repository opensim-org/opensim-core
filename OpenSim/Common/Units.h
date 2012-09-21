#ifndef __Units_h__
#define __Units_h__
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Units.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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


// INCLUDE
#include <iostream>
#include <string>
#include "osimCommonDLL.h"
#include "SimTKcommon.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing various units for measuring quantities.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API Units
{

public:
	enum UnitType
	{
		UnknownUnits = 0,
		Radians,
		Degrees,
		Millimeters,
		Centimeters,
		Meters,
		Seconds,
		Milliseconds,
		Newtons
	};

//=============================================================================
// DATA
//=============================================================================
private:
	UnitType _type;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Units();
	Units(const Units& aUnits);
	Units(const std::string aString);
	Units(UnitType aType);
	virtual ~Units();
	double convertTo(UnitType aType, double aValue) const;
	double convertTo(UnitType aType) const;
	double convertTo(const Units& aUnit) const;
	UnitType getType() const { return _type; }
	std::string getLabel() const;
	std::string getAbbreviation() const;

//=============================================================================
};	// END of class Units
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Units_h__


