#ifndef __AbstractSpeed_h__
#define __AbstractSpeed_h__

// AbstractSpeed.h
// Author: Peter Loan, Frank C. Anderson
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>

namespace OpenSim {

class AbstractDynamicsEngine;
class AbstractCoordinate;

//=============================================================================
//=============================================================================
/**
 * An abstract class that specifies the interface for a generalied speed.
 * Speeds usually have a one-to-one correspondence with coordinates, but
 * they don't have to.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractSpeed : public Object
{

//=============================================================================
// DATA
//=============================================================================
protected:
	AbstractDynamicsEngine* _dynamicsEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractSpeed();
	AbstractSpeed(const AbstractSpeed &aSpeed);
	virtual ~AbstractSpeed();
	virtual Object* copy() const = 0;

	AbstractSpeed& operator=(const AbstractSpeed &aSpeed);
   void copyData(const AbstractSpeed &aSpeed);
   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual AbstractCoordinate* getCoordinate() const = 0;
	virtual bool setCoordinate(AbstractCoordinate *aCoordinate) = 0;
	virtual bool setCoordinateName(const std::string& aCoordName) = 0;

	virtual double getDefaultValue() const = 0;
	virtual bool setDefaultValue(double aDefaultValue) = 0;
	virtual bool getDefaultValueUseDefault() const = 0;
	virtual bool getValueUseDefault() const = 0;

	virtual bool setValue(double aValue) = 0;
	virtual double getValue() const = 0;
	virtual double getAcceleration() const = 0;

	virtual void peteTest() const { }

	static std::string getSpeedName(const std::string &aCoordinateName);
	static std::string getCoordinateName(const std::string &aSpeedName);

private:
	void setNull();
//=============================================================================
};	// END of class AbstractSpeed
//=============================================================================
//=============================================================================

//typedef OSIMSIMULATION_API Set<AbstractSpeed> MarkerSet;

} // end of namespace OpenSim

#endif // __AbstractSpeed_h__


