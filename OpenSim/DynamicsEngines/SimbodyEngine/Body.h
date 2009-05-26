#ifndef __Body_h__
#define __Body_h__
// Body.h
// Author: Frank C. Anderson
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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
#include <math.h>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <SimTKsimbody.h>

class SimTK::MassProperties;

namespace OpenSim {

class SimbodyEngine;
class Joint;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody body segment.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API Body : public AbstractBody  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Mass of the body. */
	PropertyDbl _massProp;
	double &_mass;

	/** Mass center of body. */
	PropertyDblVec3 _massCenterProp;
	SimTK::Vec3 &_massCenter;

	// Inertia tensor of the body about its center of mass.
	/** Moment of inertia Ixx computed with respect to the body's center of mass. */
	PropertyDbl _inertiaXXProp;
	double &_inertiaXX;
	/** Moment of inertia Iyy computed with respect to the body's center of mass. */
	PropertyDbl _inertiaYYProp;
	double &_inertiaYY;
	/** Moment of inertia Izz computed with respect to the body's center of mass. */
	PropertyDbl _inertiaZZProp;
	double &_inertiaZZ;
	/** Product of inertia Ixy computed with respect to the body's center of mass. */
	PropertyDbl _inertiaXYProp;
	double &_inertiaXY;
	/** Product of inertia Ixz computed with respect to the body's center of mass. */
	PropertyDbl _inertiaXZProp;
	double &_inertiaXZ;
	/** Product of inertia Iyz computed with respect to the body's center of mass. */
	PropertyDbl _inertiaYZProp;
	double &_inertiaYZ;

	/** Joint connecting this body with the parent body. */
	PropertyObjPtr<Joint> _jointProp;
	Joint* &_joint;

	/** For display of the body. */
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/** ID for the body in Simbody. */
	SimTK::MobilizedBodyIndex _index;

	/** Pointer to the SimbodyEngine that contains this body. */
	// SimbodyEngine* _engine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~Body();
	Body();
	Body(const Body &aBody);
	Body(const AbstractBody &aBody);
	virtual Object* copy() const;
	Body& operator=(const Body &aBody);
	void copyData(const Body &aBody);
	void copyData(const AbstractBody &aBody);
	void setup(AbstractDynamicsEngine* aEngine);
	OpenSim::SimbodyEngine* getEngine() const {
		return (OpenSim::SimbodyEngine*)(_dynamicsEngine);
	}
	virtual double getMass() const;
	virtual bool setMass(double aMass);
	virtual void getMassCenter(SimTK::Vec3& rVec) const;
	virtual bool setMassCenter(const SimTK::Vec3& aVec);
	virtual void getInertia(SimTK::Mat33& rInertia) const;
	virtual void getInertia(double rInertia[]) const;
	virtual bool setInertia(const SimTK::Mat33& aInertia);
	virtual Joint* getJoint() const { return _joint; }
	virtual void setJoint(const Joint *aJoint);
	virtual void scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass = true);
	virtual void scaleMass(double aScaleFactor);
	virtual VisibleObject* getDisplayer() const { return &_displayer; }
	virtual void setDisplayer(VisibleObject& aVisibleObject);
	void getScaleFactors(SimTK::Vec3& aScaleFactors) const;

	OPENSIM_DECLARE_DERIVED(Body, AbstractBody);

	/** Assemble body interial properties: mass, center of mass location, moment of inertia
	    about the origin of the body and return as a SimTK::MassProperties */
	SimTK::MassProperties getMassProperties();

	SimTK::MobilizedBodyIndex getIndex() {return _index;}

private:
	void setNull();
	void setupProperties();
	friend class SimbodyEngine;
	friend class Joint;
	friend class WeldConstraint;

//=============================================================================
};	// END of class Body
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Body_h__


