#ifndef __WrapCylinder_h__
#define __WrapCylinder_h__

// WrapCylinder.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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


