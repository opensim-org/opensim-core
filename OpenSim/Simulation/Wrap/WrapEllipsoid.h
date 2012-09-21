#ifndef __WrapEllipsoid_h__
#define __WrapEllipsoid_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  WrapEllipsoid.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include "WrapObject.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class VisibleObject;
class Body;
class Model;
class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing an ellipsoid for muscle wrapping.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API WrapEllipsoid : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapEllipsoid, WrapObject);

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

#ifndef SWIG
	WrapEllipsoid& operator=(const WrapEllipsoid& aWrapEllipsoid);
#endif
   void copyData(const WrapEllipsoid& aWrapEllipsoid);
	virtual const char* getWrapTypeName() const;
	virtual std::string getDimensionsString() const;
        SimTK::Vec3 getRadii() const;

	virtual void scale(const SimTK::Vec3& aScaleFactors);
	virtual void connectToModelAndBody(Model& aModel, OpenSim::Body& aBody);
#ifndef SWIG
	virtual int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
		const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const;
#endif

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


