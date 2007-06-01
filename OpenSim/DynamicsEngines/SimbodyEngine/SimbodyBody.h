#ifndef __SimbodyBody_h__
#define __SimbodyBody_h__

// SimbodyBody.h
// Author: Frank C. Anderson
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDE
#include <iostream>
#include <string>
#include <math.h>
#include "osimSimbodyEngineDLL.h"
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody body segment.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodyBody : public AbstractBody  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Mass of the body. */
	PropertyDbl _massProp;
	double &_mass;

	/** Mass center of body. */
	PropertyDblArray _massCenterProp;
	Array<double> &_massCenter;

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
	SimTK::BodyId _id;

	/** Pointer to the SimbodyEngine that contains this body. */
	SimbodyEngine* _engine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimbodyBody();
	SimbodyBody(const SimbodyBody &aBody);
	SimbodyBody(const AbstractBody &aBody);
	virtual ~SimbodyBody();
	virtual Object* copy() const;

	SimbodyBody& operator=(const SimbodyBody &aBody);
	void copyData(const SimbodyBody &aBody);
	void copyData(const AbstractBody &aBody);

	void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const;
	virtual bool setMass(double aMass);
	virtual void getMassCenter(double rVec[3]) const;
	virtual bool setMassCenter(double aVec[3]);
	virtual void getInertia(Array<double> &rInertia) const;
	virtual void getInertia(double rInertia[3][3]) const;
	virtual bool setInertia(const Array<double>& aInertia);
	virtual bool setInertia(const double aInertia[3][3]);
	virtual void scale(const Array<double>& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(const Array<double>& aScaleFactors, bool aScaleMass = true);
	virtual void scaleMass(double aScaleFactor);
	virtual VisibleObject* getDisplayer() const { return &_displayer; }

	void transformToSimbodyFrame(const double aPos[3], double rPos[3]) const;
	void transformToSimbodyFrame(const Array<double>& aPos, double rPos[3]) const;
	void transformFromSimbodyFrame(const double aPos[3], double rPos[3]) const;
	void transformFromSimbodyFrame(const Array<double>& aPos, double rPos[3]) const;

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();
	void updateSimbody();
//=============================================================================
};	// END of class SimbodyBody
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimbodyBody_h__


