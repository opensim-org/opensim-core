#ifndef __WrapEllipsoid_h__
#define __WrapEllipsoid_h__

// WrapEllipsoid.h
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
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include "AbstractWrapObject.h"

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing an ellipsoid for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API WrapEllipsoid : public AbstractWrapObject
{

//=============================================================================
// DATA
//=============================================================================
public:

	enum WrapMethod
	{
		hybrid,
		midpoint,
		axial
	};

protected:

	PropertyDblArray _dimensionsProp;
	Array<double>& _dimensions;

	PropertyStr _methodNameProp;
	std::string& _methodName;
	WrapMethod _method;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapEllipsoid();
	WrapEllipsoid(DOMElement* aElement);
	WrapEllipsoid(const WrapEllipsoid& aWrapEllipsoid);
	virtual ~WrapEllipsoid();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement* aElement) const;
#ifndef SWIG
	WrapEllipsoid& operator=(const WrapEllipsoid& aWrapEllipsoid);
#endif
   void copyData(const WrapEllipsoid& aWrapEllipsoid);

	virtual void scale(Array<double>& aScaleFactors) { }
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual VisibleObject* getDisplayer() { return NULL; }
	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();
//=============================================================================
};	// END of class WrapEllipsoid
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapEllipsoid_h__


