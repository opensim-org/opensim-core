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
 * A class implementing a sphere obstacle for muscle wrapping, based on the
 * algorithm presented in Garner & Pandy (2000).
 *
 * @author Brian Garner, derivded from Peter Loan
 * @version 0.1
 */
class OSIMSIMULATION_API WrapSphereObst : public AbstractWrapObject
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
	WrapSphereObst();
	WrapSphereObst(const WrapSphereObst& aWrapSphereObst);
	virtual ~WrapSphereObst();
	virtual Object* copy() const;
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
	virtual void setup(AbstractDynamicsEngine* aEngine, AbstractBody* aBody);

	virtual int wrapLine(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2,
		const MuscleWrap& aMuscleWrap, WrapResult& aWrapResult, bool& aFlag) const;

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


