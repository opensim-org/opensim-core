#ifndef __SimmCoordinate_h__
#define __SimmCoordinate_h__

// SimmCoordinate.h
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
#include <math.h>
#include "osimSimmKinematicsEngineDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/AbstractTransformAxis.h>

namespace OpenSim {

class AbstractJoint;
class SimmPath;
class AbstractDynamicsEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a SIMM generalized coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMKINEMATICSENGINE_API SimmCoordinate : public AbstractCoordinate  
{
//=============================================================================
// DATA
//=============================================================================
protected:
	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	PropertyDbl _valueProp;
	double &_value;

	PropertyDbl _toleranceProp;
	double &_tolerance;

	PropertyDbl _stiffnessProp;
	double &_stiffness;

	PropertyDblArray _rangeProp;
	Array<double>& _range;

	PropertyStrArray _keysProp;
	Array<std::string>& _keys;

	PropertyBool _clampedProp;
	bool &_clamped;

	PropertyBool _lockedProp;
	bool &_locked;

	PropertyObjPtr<Function> _restraintFunctionProp;
	Function *&_restraintFunction;

	PropertyObjPtr<Function> _minRestraintFunctionProp;
	Function *&_minRestraintFunction;

	PropertyObjPtr<Function> _maxRestraintFunctionProp;
	Function *&_maxRestraintFunction;

	PropertyBool _restraintActiveProp;
	bool &_restraintActive;

	Array<AbstractJoint*> _jointList; // list of joints that use this coordinate
	Array<SimmPath*> _pathList; // list of paths that use this coordinate

	AbstractTransformAxis::MotionType _motionType; // rotational or translational (based on the DOFs it's used in)

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	SimmCoordinate();
	SimmCoordinate(const SimmCoordinate &aCoordinate);
	virtual ~SimmCoordinate();
	virtual Object* copy() const;

#ifndef SWIG
	SimmCoordinate& operator=(const SimmCoordinate &aCoordinate);
#endif
   void copyData(const SimmCoordinate &aCoordinate);

   virtual void setup(AbstractDynamicsEngine* aEngine);

	virtual void updateFromCoordinate(const AbstractCoordinate &aCoordinate);
	virtual double getValue() const { return _value; }
	virtual bool setValue(double aValue);
	virtual bool getValueUseDefault() const { return _valueProp.getUseDefault(); }
	virtual void getRange(double rRange[2]) const { rRange[0] = _range[0]; rRange[1] = _range[1]; }
	virtual bool setRange(double aRange[2]);
	virtual double getRangeMin() const { return _range[0]; }
	virtual double getRangeMax() const { return _range[1]; }
	virtual bool setRangeMin(double aMin);
	virtual bool setRangeMax(double aMax);
	virtual bool getRangeUseDefault() const { return _rangeProp.getUseDefault(); }
	virtual double getTolerance() const { return _tolerance; }
	virtual bool setTolerance(double aTolerance);
	virtual bool getToleranceUseDefault() const { return _toleranceProp.getUseDefault(); }
	virtual double getStiffness() const { return _stiffness; }
	virtual bool setStiffness(double aStiffness);
	virtual bool getStiffnessUseDefault() const { return _stiffnessProp.getUseDefault(); }
	virtual double getDefaultValue() const { return _defaultValue; }
	virtual bool setDefaultValue(double aDefaultValue);
	virtual bool getDefaultValueUseDefault() const { return _defaultValueProp.getUseDefault(); }
	virtual bool getClamped() const { return _clamped; }
	virtual bool setClamped(bool aClamped) { _clamped = aClamped; return true; }
	virtual bool getClampedUseDefault() const { return _clampedProp.getUseDefault(); }
	virtual bool getLocked() const { return _locked; }
	virtual bool setLocked(bool aLocked) { _locked = aLocked; return true; }
	virtual bool getLockedUseDefault() const { return _lockedProp.getUseDefault(); }
   virtual void addJointToList(AbstractJoint* aJoint) { _jointList.append(aJoint); }
   virtual void addPathToList(SimmPath* aPath) { _pathList.append(aPath); }
	virtual bool isUsedInModel() const { if (getJointList().getSize() > 0) return true; else return false; }
	virtual bool isRestraintActive() const { return _restraintActive; }
	virtual void setRestraintActive(bool aActive);
	virtual Function* getRestraintFunction() const;
	virtual Function* getMinRestraintFunction() const;
	virtual Function* getMaxRestraintFunction() const;
	virtual AbstractTransformAxis::MotionType getMotionType() const { return _motionType; }

	virtual void getKeys(std::string rKeys[]) const;
	virtual const Array<std::string>& getKeys() const { return _keys; }
   virtual void setKeys(const OpenSim::Array<std::string>& aKeys);
	const Array<AbstractJoint*>& getJointList() const { return _jointList; }
	void clearJointList() { _jointList.setSize(0); }
	const Array<SimmPath*>& getPathList() const { return _pathList; }
	void clearPathList() { _pathList.setSize(0); }
	void determineType();

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class SimmCoordinate
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmCoordinate_h__


