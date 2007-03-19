#ifndef __WrapSphere_h__
#define __WrapSphere_h__

// WrapSphere.h
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "AbstractWrapObject.h"

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;
class MusclePoint;
class MuscleWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a sphere for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapSphere : public AbstractWrapObject
{

//=============================================================================
// DATA
//=============================================================================

	PropertyDbl _radiusProp;
	double& _radius;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapSphere();
	WrapSphere(const WrapSphere& aWrapSphere);
	virtual ~WrapSphere();
	virtual Object* copy() const;
#ifndef SWIG
	WrapSphere& operator=(const WrapSphere& aWrapSphere);
#endif
   void copyData(const WrapSphere& aWrapSphere);
	virtual const char* getWrapTypeName() const;
	virtual std::string getDimensionsString() const;

	virtual void scale(Array<double>& aScaleFactors) { }
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();

//=============================================================================
};	// END of class WrapSphere
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapSphere_h__


