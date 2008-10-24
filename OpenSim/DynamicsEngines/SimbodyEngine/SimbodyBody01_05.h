#ifndef __SimbodyBody01_05_h__
#define __SimbodyBody01_05_h__
// SimbodyBody01_05.h
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
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class SimbodyEngine01_05;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody body segment.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodyBody01_05 : public AbstractBody  
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

	/** Inertia tensor of the body about the center of mass when the local body
	reference frame is aligned with the global reference frame.  This is a
	9-element array in the following order:
	Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz. */
	PropertyDblArray _inertiaProp;
	Array<double> &_inertia;

	/** For display of the body. */
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/** ID for the body in Simbody. */
	SimTK::MobilizedBodyIndex _id;

	/** Pointer to the SimbodyEngine that contains this body. */
	SimbodyEngine01_05* _engine;

	const static std::string _NewType;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~SimbodyBody01_05();
	SimbodyBody01_05();
	SimbodyBody01_05(const SimbodyBody01_05 &aBody);
	SimbodyBody01_05(const AbstractBody &aBody);
	virtual Object* copy() const;
	SimbodyBody01_05& operator=(const SimbodyBody01_05 &aBody);
	void copyData(const SimbodyBody01_05 &aBody);
	void copyData(const AbstractBody &aBody);
	void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const;
	virtual bool setMass(double aMass);
	virtual void getMassCenter(SimTK::Vec3& rVec) const;
	virtual bool setMassCenter(const SimTK::Vec3& aVec);
	virtual void getInertia(SimTK::Mat33& rInertia) const;
	virtual void getInertia(double rInertia[3][3]) const;
	virtual bool setInertia(const SimTK::Mat33& aInertia);
	virtual bool setInertia(const double aInertia[3][3]);
	virtual void scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass = true);
	virtual void scaleMass(double aScaleFactor);
	virtual VisibleObject* getDisplayer() const { return &_displayer; }
	virtual void setDisplayer(VisibleObject& aVisibleObject);
	void getScaleFactors(SimTK::Vec3& aScaleFactors) const;

	virtual const std::string& getNewType() const { return _NewType; };
private:
	void setNull();
	void setupProperties();
	void updateSimbody();
	friend class SimbodyEngine01_05;
	friend class SimbodyEngine;

//=============================================================================
};	// END of class SimbodyBody01_05
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodyBody01_05_h__


