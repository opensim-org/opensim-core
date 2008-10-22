#ifndef __WrapEllipsoid_h__
#define __WrapEllipsoid_h__

// WrapEllipsoid.h
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

	virtual void scale(const SimTK::Vec3& aScaleFactors);
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

	OPENSIM_DECLARE_DERIVED(WrapEllipsoid, AbstractWrapObject);
protected:
	void setupProperties();

private:
	void setNull();
	int calcTangentPoint(double p1e, SimTK::Vec3& r1, SimTK::Vec3& p1, SimTK::Vec3& m,
												SimTK::Vec3& a, SimTK::Vec3& vs, double vs4) const;
	void CalcDistanceOnEllipsoid(SimTK::Vec3& r1, SimTK::Vec3& r2, SimTK::Vec3& m, SimTK::Vec3& a, 
														  SimTK::Vec3& vs, double vs4, bool far_side_wrap,
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


