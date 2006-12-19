#ifndef __SimmBody_h__
#define __SimmBody_h__

// SimmBody.h
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
#include <fstream>
#include <string>
#include <math.h>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include "AbstractBody.h"
#include "BoneSet.h"

namespace OpenSim {

class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM body segment.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API SimmBody : public AbstractBody  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _massProp;
	double &_mass;

	PropertyDblArray _massCenterProp;
	Array<double> &_massCenter;

	PropertyDblArray _inertiaProp;
	Array<double> &_inertia;

	// Support of display.
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmBody();
	SimmBody(DOMElement *aElement);
	SimmBody(const SimmBody &aBody);
	virtual ~SimmBody();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;

#ifndef SWIG
	SimmBody& operator=(const SimmBody &aBody);
#endif
   void copyData(const SimmBody &aBody);

   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const { return _mass; }
	virtual bool setMass(double aMass);
	virtual void getMassCenter(double rVec[3]) const;
	virtual bool setMassCenter(double aVec[3]);
	virtual void getInertia(double rInertia[3][3]) const;
	virtual bool setInertia(const Array<double>& aInertia);
	virtual void scale(Array<double>& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(Array<double>& aScaleFactors);
	virtual VisibleObject* getDisplayer() const { return &_displayer; }
	void getScaleFactors(Array<double>& aScaleFactors) const;

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmBody
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmBody_h__


