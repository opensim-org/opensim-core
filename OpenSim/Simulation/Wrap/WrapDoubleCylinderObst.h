#ifndef __WrapDoubleCylinderObst_h__
#define __WrapDoubleCylinderObst_h__
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  WrapDoubleCylinderObst.h                     *
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
class Model;
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
class OSIMSIMULATION_API WrapDoubleCylinderObst : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapDoubleCylinderObst, WrapObject);

//=============================================================================
// DATA
//=============================================================================

	enum WrapDirectionEnum	// The prescribed direction of wrapping about the cylinders' z-axis
	{
		righthand,
		lefthand
	};

	PropertyDbl _radiusUcylProp;
	double& _radiusUcyl;

	PropertyDbl _radiusVcylProp;
	double& _radiusVcyl;

	// Facilitate prescription of wrapping direction around obstacle: "righthand" or "lefthand".
	// In traversing from the 1st point (P) to the 2nd (S), the path will wrap either
	//    righthanded or lefthanded about the obstacle's z-axis.
	PropertyStr _wrapUcylDirectionNameProp;
	std::string& _wrapUcylDirectionName;
	WrapDirectionEnum _wrapUcylDirection;

	PropertyStr _wrapVcylDirectionNameProp;
	std::string& _wrapVcylDirectionName;
	WrapDirectionEnum _wrapVcylDirection;

	// Name of body to which B cylinder is attached
	PropertyStr _wrapVcylHomeBodyNameProp;
	std::string& _wrapVcylHomeBodyName;
	OpenSim::Body* _wrapVcylHomeBody;
	OpenSim::Body* _wrapUcylHomeBody;

	PropertyDblArray _xyzBodyRotationVcylProp;
	Array<double>& _xyzBodyRotationVcyl;

	PropertyDblVec3 _translationVcylProp;
	SimTK::Vec3 & _translationVcyl;

	PropertyDbl _lengthProp;
	double& _length;
	
	// State of activity of each or both cylinders:  0=inactive, 1=U-Cylinder, 2=V-Cylinder, 3=Both Cylinders
	int _activeState;	
	Model* _model;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	WrapDoubleCylinderObst();
	WrapDoubleCylinderObst(const WrapDoubleCylinderObst& aWrapDoubleCylinderObst);
	virtual ~WrapDoubleCylinderObst();

#ifndef SWIG
	WrapDoubleCylinderObst& operator=(const WrapDoubleCylinderObst& aWrapDoubleCylinderObst);
#endif
   void copyData(const WrapDoubleCylinderObst& aWrapDoubleCylinderObst);

	double getRadius() const { return _radiusUcyl; }
	void setRadius(double aRadius) { _radiusUcyl = aRadius; }
	double getLength() const { return _length; }
	void setLength(double aLength) { _length = aLength; }
	//WrapDirectionEnum getWrapDirection() const { return _wrapUcylDirection; }
	int getWrapDirection() const { return (int)_wrapUcylDirection; }

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
	void getVcylToUcylRotationMatrix(const SimTK::State& s, double M[9]) const;


//=============================================================================
};	// END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


