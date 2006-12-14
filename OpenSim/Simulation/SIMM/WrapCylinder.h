#ifndef __WrapCylinder_h__
#define __WrapCylinder_h__

// WrapCylinder.h
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include "AbstractWrapObject.h"

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a cylinder for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API WrapCylinder : public AbstractWrapObject
{

//=============================================================================
// DATA
//=============================================================================

	PropertyDbl _radiusProp;
	double& _radius;

	// Height is used only for diplaying the cylinder, not for wrapping.
	// It is specified here as a parameter to remain consistent with SIMM.
	PropertyDbl _heightProp;
	double& _height;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapCylinder();
	WrapCylinder(DOMElement* aElement);
	WrapCylinder(const WrapCylinder& aWrapCylinder);
	virtual ~WrapCylinder();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement* aElement) const;
#ifndef SWIG
	WrapCylinder& operator=(const WrapCylinder& aWrapCylinder);
#endif
   void copyData(const WrapCylinder& aWrapCylinder);

	virtual void scale(Array<double>& aScaleFactors) { }
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual VisibleObject* getDisplayer() { return NULL; }
	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();
//=============================================================================
};	// END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


