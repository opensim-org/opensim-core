#ifndef __AbstractCoordinate_h__
#define __AbstractCoordinate_h__

// AbstractCoordinate.h
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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include "AbstractTransformAxis.h"

namespace OpenSim {

class AbstractDynamicsEngine;
class AbstractJoint;
class SimmPath;
class Function;

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractCoordinate : public Object  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	AbstractDynamicsEngine* _dynamicsEngine;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractCoordinate();
	AbstractCoordinate(const AbstractCoordinate &aCoordinate);
	virtual ~AbstractCoordinate();
	virtual Object* copy() const = 0;

#ifndef SWIG
	AbstractCoordinate& operator=(const AbstractCoordinate &aCoordinate);
#endif
   void copyData(const AbstractCoordinate &aCoordinate);

	virtual AbstractDynamicsEngine* getDynamicsEngine() const { return _dynamicsEngine; }
	virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual void updateFromCoordinate(const AbstractCoordinate &aCoordinate) = 0;
	virtual double getValue() const  = 0;
	virtual bool setValue(double aValue) = 0;
	virtual bool setValue(double aValue, bool enforceConstraints) { return setValue(aValue); }
	virtual bool getValueUseDefault() const = 0;
	virtual void getRange(double rRange[2]) const = 0;
	virtual bool setRange(double aRange[2]) = 0;
	virtual double getRangeMin() const = 0;
	virtual double getRangeMax() const = 0;
	virtual bool setRangeMin(double aMin) = 0;
	virtual bool setRangeMax(double aMax) = 0;
	virtual bool getRangeUseDefault() const = 0;
	virtual double getTolerance() const = 0;
	virtual bool setTolerance(double aTolerance) = 0;
	virtual bool getToleranceUseDefault() const = 0;
	virtual double getStiffness() const = 0;
	virtual bool setStiffness(double aStiffness) = 0;
	virtual bool getStiffnessUseDefault() const = 0;
	virtual double getDefaultValue() const = 0;
	virtual bool setDefaultValue(double aDefaultValue) = 0;
	virtual bool getDefaultValueUseDefault() const = 0;
	virtual bool getClamped() const = 0;
	virtual bool setClamped(bool aClamped) = 0;
	virtual bool getClampedUseDefault() const = 0;
	virtual bool getLocked() const = 0;
	virtual bool setLocked(bool aLocked) = 0;
	virtual bool getLockedUseDefault() const = 0;
   virtual void addJointToList(AbstractJoint* aJoint) { }
   virtual void addPathToList(SimmPath* aJoint) { }
	virtual bool isUsedInModel() const { return true; }
	virtual bool isRestraintActive() const { return false; }
	virtual void setRestraintActive(bool aActive) { }
	virtual Function* getRestraintFunction() const { return NULL; }
	virtual Function* getMinRestraintFunction() const { return NULL; }
	virtual Function* getMaxRestraintFunction() const { return NULL; }
	virtual Function* getPrescribedFunction() const { return NULL; }
	virtual AbstractTransformAxis::MotionType getMotionType() const = 0;
	virtual void determineType() = 0;
	virtual bool isConstrained() const { return false; }
	virtual bool isPrescribed() const {return false;}
	virtual void getKeys(std::string rKeys[]) const = 0;
	virtual const Array<std::string>& getKeys() const = 0;
	virtual void setKeys(const Array<std::string>& aKeys) = 0;
	void setDynamicsEngine(AbstractDynamicsEngine* aEngine);

	OPENSIM_DECLARE_DERIVED(AbstractCoordinate, Object);

private:
	void setNull(void);

//=============================================================================
};	// END of class AbstractCoordinate
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractCoordinate_h__


