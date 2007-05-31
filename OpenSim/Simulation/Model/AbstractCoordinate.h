#ifndef __AbstractCoordinate_h__
#define __AbstractCoordinate_h__

// AbstractCoordinate.h
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include "AbstractDof.h"

namespace OpenSim {

class AbstractDynamicsEngine;
class AbstractJoint;
class SimmPath;
class Function;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractCoordinate : public Object  
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
	AbstractCoordinate();
	AbstractCoordinate(const AbstractCoordinate &aCoordinate);
	virtual ~AbstractCoordinate();
	virtual Object* copy() const = 0;

#ifndef SWIG
	AbstractCoordinate& operator=(const AbstractCoordinate &aCoordinate);
#endif
   void copyData(const AbstractCoordinate &aCoordinate);

	virtual AbstractDynamicsEngine* getDynamicsEngine() { return _dynamicsEngine; }
   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual void updateFromCoordinate(const AbstractCoordinate &aCoordinate) = 0;
	virtual double getValue() const  = 0;
	virtual bool setValue(double aValue) = 0;
	virtual bool getValueUseDefault() const = 0;
	virtual void getRange(double rRange[2]) const = 0;
	virtual bool setRange(double aRange[2]) = 0;
	virtual double getRangeMin() const = 0;
	virtual double getRangeMax() const = 0;
	virtual bool setRangeMin(double aMin) = 0;
	virtual bool setRangeMax(double aMax) = 0;
	virtual bool getRangeUseDefault() const = 0;
	virtual double getTolerance() const = 0;
	virtual bool setTolerance(double aTolerance) = 0;
	virtual bool getToleranceUseDefault() const = 0;
	virtual double getStiffness() const = 0;
	virtual bool setStiffness(double aStiffness) = 0;
	virtual bool getStiffnessUseDefault() const = 0;
	virtual double getDefaultValue() const = 0;
	virtual bool setDefaultValue(double aDefaultValue) = 0;
	virtual bool getDefaultValueUseDefault() const = 0;
	virtual bool getClamped() const = 0;
	virtual bool setClamped(bool aClamped) = 0;
	virtual bool getClampedUseDefault() const = 0;
	virtual bool getLocked() const = 0;
	virtual bool setLocked(bool aLocked) = 0;
	virtual bool getLockedUseDefault() const = 0;
   virtual void addJointToList(AbstractJoint* aJoint) { }
   virtual void addPathToList(SimmPath* aJoint) { }
	virtual bool isUsedInModel() const { return true; }
	virtual bool isRestraintActive() const { return false; }
	virtual Function* getRestraintFunction() const { return NULL; }
	virtual Function* getMinRestraintFunction() const { return NULL; }
	virtual Function* getMaxRestraintFunction() const { return NULL; }
	virtual AbstractDof::DofType getMotionType() const = 0;
	virtual void determineType() = 0;
	virtual bool getConstrained() const { return false; }

	virtual void peteTest() const { }
private:
	void setNull(void);

//=============================================================================
};	// END of class AbstractCoordinate
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractCoordinate_h__


