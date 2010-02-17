#ifndef __WrapCylinderObst_h__
#define __WrapCylinderObst_h__

// WrapCylinderObst.h
// Author: Brian Garner, as derived from WrapCylinder.h by Peter Loan


// INCLUDE
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
class OSIMSIMULATION_API WrapCylinderObst : public WrapObject
{

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
	virtual Object* copy() const;
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
	virtual void setup(Model& aModel, OpenSim::Body& aBody);
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


