#ifndef __SdfastBody_h__
#define __SdfastBody_h__

// SdfastBody.h
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include <string>
#include "osimSdfastEngineDLL.h"
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>

namespace OpenSim {

class SdfastEngine;
class VisibleObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM body segment.
 *
 * @author Peter Loan, Frank C. Anderson
 * @version 1.0
 */
class OSIMSDFASTENGINE_API SdfastBody : public AbstractBody  
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
	SimTK::Vec3&	_massCenter;

	/** Inertia tensor of the body about the center of mass when the local body
	reference frame is aligned with the global reference frame.  This is a
	9-element array in the following order:
	Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz. */
	PropertyDblArray _inertiaProp;
	Array<double> &_inertia;

	/** For display of the body. */
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

	/** Index of this body in the SD/FAST code. */
	PropertyInt _indexProp;
	int &_index;

	/** Pointer to the SdfastEngine that contains this body. */
	SdfastEngine* _SdfastEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SdfastBody();
	SdfastBody(const SdfastBody &aBody);
	SdfastBody(const AbstractBody &aBody);
	virtual ~SdfastBody();
	virtual Object* copy() const;

	SdfastBody& operator=(const SdfastBody &aBody);
	void copyData(const SdfastBody &aBody);
	void copyData(const AbstractBody &aBody);

	void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const;
	virtual bool setMass(double aMass);
	virtual void getMassCenter(SimTK::Vec3& rVec) const;
	virtual bool setMassCenter(const SimTK::Vec3& aVec);
	virtual void getInertia(SimTK::Mat33 &rInertia) const;
	virtual bool setInertia(const SimTK::Mat33& aInertia);
	virtual void scale(const SimTK::Vec3& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(const SimTK::Vec3& aScaleFactors, bool aScaleMass = true);
	virtual void scaleMass(double aScaleFactor);
	virtual VisibleObject* getDisplayer() const { return &_displayer; }

	void setSdfastIndex(int aIndex) { _index = aIndex; }
	int getSdfastIndex() const { return _index; }
	void transformToSdfastFrame(const double aPos[3], double rPos[3]) const;
	void transformToSdfastFrame(const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const;
	void transformFromSdfastFrame(const double aPos[3], double rPos[3]) const;
	void transformFromSdfastFrame(const SimTK::Vec3& aPos, SimTK::Vec3& rPos) const;

private:
	void setNull();
	void setupProperties();
	void updateSdfast();
//=============================================================================
};	// END of class SdfastBody
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SdfastBody_h__


