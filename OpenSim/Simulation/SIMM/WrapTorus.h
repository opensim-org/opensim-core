#ifndef __WrapTorus_h__
#define __WrapTorus_h__

// WrapTorus.h
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
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/VisibleObject.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include "AbstractWrapObject.h"

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;
class SimmMusclePoint;
class MuscleWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a torus for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class RDSIMULATION_API WrapTorus : public AbstractWrapObject
{

	struct CircleCallback {
		double p1[3], p2[3], r;
	};

//=============================================================================
// DATA
//=============================================================================

	PropertyDbl _innerRadiusProp;
	double& _innerRadius;

	PropertyDbl _outerRadiusProp;
	double& _outerRadius;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapTorus();
	WrapTorus(const WrapTorus& aWrapTorus);
	virtual ~WrapTorus();
	virtual Object* copy() const;
#ifndef SWIG
	WrapTorus& operator=(const WrapTorus& aWrapTorus);
#endif
   void copyData(const WrapTorus& aWrapTorus);
	virtual const char* getWrapTypeName() const;
	virtual std::string getDimensionsString() const;

	virtual void scale(Array<double>& aScaleFactors) { }
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

	virtual VisibleObject* getDisplayer() { return NULL; }
	virtual void peteTest() const;

protected:
	void setupProperties();

private:
	void setNull();
	int findClosestPoint(double radius, double p1[], double p2[],
		double* xc, double* yc, double* zc,
		int wrap_sign, int wrap_axis) const;
	static void calcCircleResids(int numResid, int numQs, double q[],
		double resid[], int *flag2, void *ptr);

//=============================================================================
};	// END of class WrapTorus
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapTorus_h__


