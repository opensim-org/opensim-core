#ifndef __AbstractBody_h__
#define __AbstractBody_h__

// AbstractBody.h
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/VisibleObject.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

namespace OpenSim {

class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a body.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractBody : public Object  
{

//=============================================================================
// DATA
//=============================================================================
protected:

	/** Set containing the wrap objects attached to this body. */
	PropertyObj _wrapObjectSetProp;
	WrapObjectSet &_wrapObjectSet;

	AbstractDynamicsEngine* _dynamicsEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractBody();
	AbstractBody(const AbstractBody &aBody);
	virtual ~AbstractBody();

	virtual Object* copy() const = 0;
#ifndef SWIG
	AbstractBody& operator=(const AbstractBody &aBody);
#endif
   void copyData(const AbstractBody &aBody);

	virtual AbstractDynamicsEngine* getDynamicsEngine() { return _dynamicsEngine; }
   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual double getMass() const = 0;
	virtual bool setMass(double aMass) = 0;
	virtual void getMassCenter(double rVec[3]) const = 0;
	virtual bool setMassCenter(const double aVec[3]) = 0;
	virtual void getInertia(double rInertia[3][3]) const = 0;
	virtual bool setInertia(const Array<double>& aInertia) = 0;
	virtual void scale(const Array<double>& aScaleFactors, bool aScaleMass = false) = 0;
	virtual void scaleInertialProperties(const Array<double>& aScaleFactors, bool aScaleMass = true) = 0;
	virtual void scaleMass(double aScaleFactor) = 0;
	virtual VisibleObject* getDisplayer() const = 0;
	AbstractWrapObject* getWrapObject(const std::string& aName) const;
	WrapObjectSet& getWrapObjectSet() { return _wrapObjectSet; }

	OPENSIM_DECLARE_DERIVED(AbstractBody, Object);

private:
	void setNull();
	void setupProperties();

protected:
	static void scaleInertiaTensor(double aMass, const Array<double> &aScaleFactors, double rInertia[3][3]);
//=============================================================================
};	// END of class AbstractBody
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractBody_h__


