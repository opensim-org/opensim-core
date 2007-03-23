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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include "AbstractWrapObject.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class VisibleObject;
class AbstractBody;
class AbstractDynamicsEngine;
class MuscleWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing an ellipsoid for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapEllipsoid : public AbstractWrapObject
{

//=============================================================================
// DATA
//=============================================================================
protected:

	PropertyDblArray _dimensionsProp;
	Array<double>& _dimensions;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapEllipsoid();
	WrapEllipsoid(const WrapEllipsoid& aWrapEllipsoid);
	virtual ~WrapEllipsoid();
	virtual Object* copy() const;
#ifndef SWIG
	WrapEllipsoid& operator=(const WrapEllipsoid& aWrapEllipsoid);
#endif
   void copyData(const WrapEllipsoid& aWrapEllipsoid);
	virtual const char* getWrapTypeName() const;
	virtual std::string getDimensionsString() const;

	virtual void scale(Array<double>& aScaleFactors) { }
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(Array<double>& aPoint1, Array<double>& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

	virtual void peteTest() const;

	OPENSIM_DECLARE_DERIVED(WrapEllipsoid, AbstractWrapObject);
protected:
	void setupProperties();

private:
	void setNull();
	int calcTangentPoint(double p1e, double r1[], double p1[], double m[],
		double a[], double vs[], double vs4) const;
	void CalcDistanceOnEllipsoid(double r1[], double r2[], double m[], double a[], 
		double vs[], double vs4, bool far_side_wrap,
		WrapResult& aWrapResult) const;
	double findClosestPoint(double a, double b, double c,
		double u, double v, double w,
		double* x, double* y, double* z,
		int specialCaseAxis = -1) const;
	double closestPointToEllipse(double a, double b, double u,
		double v, double* x, double* y) const;
//=============================================================================
};	// END of class WrapEllipsoid
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapEllipsoid_h__


