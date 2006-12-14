#ifndef __AbstractBody_h__
#define __AbstractBody_h__

// AbstractBody.h
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/Set.h>
#include "WrapObjectSet.h"

namespace OpenSim {

class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a body.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API AbstractBody : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Set containing the wrap objects attached to this body. */
	PropertyObj _wrapObjectSetProp;
	WrapObjectSet &_wrapObjectSet;

	AbstractDynamicsEngine* _dynamicsEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractBody();
	AbstractBody(DOMElement *aElement);
	AbstractBody(const AbstractBody &aBody);
	virtual ~AbstractBody();

	virtual Object* copy() const = 0;
	virtual Object* copy(DOMElement *aElement) const = 0;
#ifndef SWIG
	AbstractBody& operator=(const AbstractBody &aBody);
#endif
   void copyData(const AbstractBody &aBody);

	virtual AbstractDynamicsEngine* getDynamicsEngine() { return _dynamicsEngine; }
   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const = 0;
	virtual bool setMass(double aMass) = 0;
	virtual void getMassCenter(double rVec[3]) const = 0;
	virtual bool setMassCenter(double aVec[3]) = 0;
	virtual void getInertia(double rInertia[3][3]) const = 0;
	virtual bool setInertia(const Array<double>& aInertia) = 0;
	virtual void scale(Array<double>& aScaleFactors, bool aScaleMass = false) = 0;
	virtual void scaleInertialProperties(Array<double>& aScaleFactors) = 0;
	virtual VisibleObject* getDisplayer() const = 0;
	AbstractWrapObject* getWrapObject(const std::string& aName);
	WrapObjectSet* getWrapObjectSet() { return &_wrapObjectSet; }

	virtual void peteTest(void) const { }

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class AbstractBody
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractBody_h__


