#ifndef __Body_h__
#define __Body_h__
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Body.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <SimTKsimbody.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

namespace OpenSim {

class Model;
class WrapObject;
class WrapObjectSet;
//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody rigid body.
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Body : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Body, ModelComponent);

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

	/** Set containing the wrap objects attached to this body. */
	PropertyObj _wrapObjectSetProp;
	WrapObjectSet &_wrapObjectSet;

	/** ID for the body in Simbody. */
	SimTK::MobilizedBodyIndex _index;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Body();
	Body(const std::string &aName,double aMass,const SimTK::Vec3& aMassCenter,const SimTK::Inertia& aInertia);
	Body(const Body &aBody);
	virtual ~Body();

#ifndef SWIG
	Body& operator=(const Body &aBody);
#endif
	void copyData(const Body &aBody);

	virtual void addDisplayGeometry(const std::string &aGeometryFileName);
	virtual double getMass() const;
	virtual bool setMass(double aMass);
	virtual void getMassCenter(SimTK::Vec3& rVec) const;
	virtual bool setMassCenter(const SimTK::Vec3& aVec);
	virtual void getInertia(SimTK::Mat33& rInertia) const;
	virtual bool setInertia(const SimTK::Inertia& aInertia);
    virtual bool hasJoint() const { return _joint != NULL; }
	virtual Joint& getJoint() const;
	virtual void setJoint(Joint& aJoint);
	virtual void scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass = true);
	virtual void scaleMass(double aScaleFactor);

	virtual const VisibleObject* getDisplayer() const { return &_displayer; }
	virtual VisibleObject*	updDisplayer() { return &_displayer; };

	virtual const SimTK::MobilizedBodyIndex getIndex() const {return _index;};
	void getScaleFactors(SimTK::Vec3& aScaleFactors) const;

	/** add a wrap object to the body. Note that the body takes ownership of the WrapObject */
	void addWrapObject(WrapObject* wrapObject);
	WrapObject* getWrapObject(const std::string& aName) const;
	const WrapObjectSet& getWrapObjectSet() const { return _wrapObjectSet; }
	

	/** Assemble body interial properties: mass, center of mass location, moment of inertia
	    about the origin of the body and return as a SimTK::MassProperties */
	SimTK::MassProperties getMassProperties();
	virtual int getNumStateVariables() const { return 0; };

protected:
    // Model component interface.
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;	

private:
	void setNull();
	void setupProperties();
	friend class Joint;

//=============================================================================
};	// END of class Body
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Body_h__


