#ifndef __Units_h__
#define __Units_h__

// Units.h
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


// INCLUDE
#include <iostream>
#include <string>
#include "osimCommonDLL.h"

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
		simmUnknownUnits = 0,
		simmRadians,
		simmDegrees,
		simmMillimeters,
		simmCentimeters,
		simmMeters,
		simmSeconds,
		simmMilliseconds,
		simmNewtons
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
	Units(std::string& aString);
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


