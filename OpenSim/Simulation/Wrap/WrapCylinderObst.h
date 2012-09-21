#ifndef __WrapCylinderObst_h__
#define __WrapCylinderObst_h__
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  WrapCylinderObst.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Brian Garner, Peter Loan                                        *
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
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>

namespace OpenSim {

class VisibleObject;
class Body;
class PathPoint;
class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a cylinder obstacle for muscle wrapping, based on
 * algorithm presented in Garner & Pandy (2000).
 *
 * @author Brian Garner, derivded from Peter Loan
 * @version 0.1
 */
class OSIMSIMULATION_API WrapCylinderObst : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapCylinderObst, WrapObject);

//=============================================================================
// DATA
//=============================================================================

	enum WrapDirectionEnum	// The prescribed direction of wrapping about the cylinders' z-axis
	{
		righthand,
		lefthand
	};

	PropertyDbl _radiusProp;
	double& _radius;

	// Facilitate prescription of wrapping direction around obstacle: "righthand" or "lefthand".
	// In traversing from the 1st point (P) to the 2nd (S), the path will wrap either
	//    righthanded or lefthanded about the obstacle's z-axis.
	PropertyStr _wrapDirectionNameProp;
	std::string& _wrapDirectionName;
	WrapDirectionEnum _wrapDirection;

	PropertyDbl _lengthProp;
	double& _length;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapCylinderObst();
	WrapCylinderObst(const WrapCylinderObst& aWrapCylinderObst);
	virtual ~WrapCylinderObst();

#ifndef SWIG
	WrapCylinderObst& operator=(const WrapCylinderObst& aWrapCylinderObst);
#endif
   void copyData(const WrapCylinderObst& aWrapCylinderObst);

	double getRadius() const { return _radius; }
	void setRadius(double aRadius) { _radius = aRadius; }
	double getLength() const { return _length; }
	void setLength(double aLength) { _length = aLength; }
	//WrapDirectionEnum getWrapDirection() const { return _wrapDirection; }
	int getWrapDirection() const { return (int)_wrapDirection; }

	virtual const char* getWrapTypeName() const;
	virtual std::string getDimensionsString() const;
	virtual void scale(const SimTK::Vec3& aScaleFactors) { }
	virtual void connectToModelAndBody(Model& aModel, OpenSim::Body& aBody);
#ifndef SWIG
	virtual int wrapLine(const SimTK::State& s, SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
		const PathWrap& aPathWrap, WrapResult& aWrapResult, bool& aFlag) const;
#endif
protected:
	void setupProperties();

private:
	void setNull();
	void initCircleWrapPts();

//=============================================================================
};	// END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


