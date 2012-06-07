#ifndef __WrapSphereObst_h__
#define __WrapSphereObst_h__

// WrapSphereObst.h
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
class Model;
class PathPoint;
class PathWrap;
class WrapResult;

//=============================================================================
//=============================================================================
/**
 * A class implementing a sphere obstacle for muscle wrapping, based on the
 * algorithm presented in Garner & Pandy (2000).
 *
 * @author Brian Garner, derivded from Peter Loan
 * @version 0.1
 */
class OSIMSIMULATION_API WrapSphereObst : public WrapObject {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapSphereObst, WrapObject);

//=============================================================================
// DATA
//=============================================================================
private:
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
	WrapSphereObst();
	WrapSphereObst(const WrapSphereObst& aWrapSphereObst);
	virtual ~WrapSphereObst();

#ifndef SWIG
	WrapSphereObst& operator=(const WrapSphereObst& aWrapSphereObst);
#endif
   void copyData(const WrapSphereObst& aWrapSphereObst);

	double getRadius() const { return _radius; }
	void setRadius(double aRadius) { _radius = aRadius; }
	double getLength() const { return _length; }
	void setLength(double aLength) { _length = aLength; }

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

//=============================================================================
};	// END of class WrapCylinder
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapCylinder_h__


