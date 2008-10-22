#ifndef __WrapDoubleCylinderObst_h__
#define __WrapDoubleCylinderObst_h__

// WrapDoubleCylinderObst.h
// Author: Brian Garner, as derived from WrapCylinder.h by Peter Loan


// INCLUDE
#include <iostream>
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Simulation/Wrap/AbstractWrapObject.h>

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
 * A class implementing a cylinder obstacle for muscle wrapping, based on
 * algorithm presented in Garner & Pandy (2000).
 *
 * @author Brian Garner, derivded from Peter Loan
 * @version 0.1
 */
class OSIMSIMULATION_API WrapDoubleCylinderObst : public AbstractWrapObject
{

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
	// In traversing from the 1st attachment (P) to the 2nd (S), the muscle will wrap either
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
	AbstractBody* _wrapVcylHomeBody;
	AbstractBody* _wrapUcylHomeBody;

	PropertyDblArray _xyzBodyRotationVcylProp;
	Array<double>& _xyzBodyRotationVcyl;

	PropertyDblVec3 _translationVcylProp;
	SimTK::Vec3 & _translationVcyl;

	PropertyDbl _lengthProp;
	double& _length;
	
	// State of activity of each or both cylinders:  0=inactive, 1=U-Cylinder, 2=V-Cylinder, 3=Both Cylinders
	int _activeState;	
	AbstractDynamicsEngine* _theDynEngine;

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
	virtual Object* copy() const;
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
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

protected:
	void setupProperties();

private:
	void setNull();
	void getVcylToUcylRotationMatrix(double M[9]) const;


//=============================================================================
};	// END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


