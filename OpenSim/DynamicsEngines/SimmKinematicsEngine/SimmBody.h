#ifndef __SimmBody_h__
#define __SimmBody_h__

// SimmBody.h
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
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include "osimSimmKinematicsEngineDLL.h"
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractBody.h>
#include <OpenSim/Simulation/Model/BoneSet.h>

namespace OpenSim {

class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM body segment.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMKINEMATICSENGINE_API SimmBody : public AbstractBody  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _massProp;
	double &_mass;

	PropertyDblArray _massCenterProp;
	Array<double> &_massCenter;

	PropertyDblArray _inertiaProp;
	Array<double> &_inertia;

	// Support of display.
	PropertyObj _displayerProp;
	VisibleObject &_displayer;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmBody();
	SimmBody(const SimmBody &aBody);
	virtual ~SimmBody();
	virtual Object* copy() const;

#ifndef SWIG
	SimmBody& operator=(const SimmBody &aBody);
#endif
   void copyData(const SimmBody &aBody);

   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const { return _mass; }
	virtual bool setMass(double aMass);
	virtual void getMassCenter(double rVec[3]) const;
	virtual bool setMassCenter(const double aVec[3]);
	virtual void getInertia(double rInertia[3][3]) const;
	virtual bool setInertia(const Array<double>& aInertia);
	virtual void scale(const Array<double>& aScaleFactors, bool aScaleMass = false);
	virtual void scaleInertialProperties(const Array<double>& aScaleFactors, bool aScaleMass = true);
	virtual void scaleMass(double aScaleFactor);
	virtual VisibleObject* getDisplayer() const { return &_displayer; }
	void getScaleFactors(Array<double>& aScaleFactors) const;

private:
	void setNull();
	void setupProperties();
//=============================================================================
};	// END of class SimmBody
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmBody_h__


