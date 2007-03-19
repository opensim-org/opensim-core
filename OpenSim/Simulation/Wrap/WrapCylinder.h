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
 * A class implementing a cylinder for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapCylinder : public AbstractWrapObject
{

//=============================================================================
// DATA
//=============================================================================

	PropertyDbl _radiusProp;
	double& _radius;

	PropertyDbl _lengthProp;
	double& _length;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapCylinder();
	WrapCylinder(const WrapCylinder& aWrapCylinder);
	virtual ~WrapCylinder();
	virtual Object* copy() const;
#ifndef SWIG
	WrapCylinder& operator=(const WrapCylinder& aWrapCylinder);
#endif
   void copyData(const WrapCylinder& aWrapCylinder);

	double getRadius() const { return _radius; }
	void setRadius(double aRadius) { _radius = aRadius; }
	double getLength() const { return _length; }
	void setLength(double aLength) { _length = aLength; }

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
	void _make_spiral_path(double aPoint1[3], double aPoint2[3],
		bool far_side_wrap, WrapResult& aWrapResult) const;
	void _calc_spiral_wrap_point(const double	r1a[3], const double axial_vec[3],
		double m[4][4], const double axis[3], double sense,
		double t, double theta, double wrap_pt[3]) const;
	bool _adjust_tangent_point(double pt1[3], double dn[3], double r1[3], double w1[3]) const;

//=============================================================================
};	// END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


