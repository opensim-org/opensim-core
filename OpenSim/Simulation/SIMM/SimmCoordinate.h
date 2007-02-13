#ifndef __SimmCoordinate_h__
#define __SimmCoordinate_h__

// SimmCoordinate.h
// Author: Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/Function.h>
#include "AbstractCoordinate.h"

namespace OpenSim {

class AbstractDof;
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
class RDSIMULATION_API SimmCoordinate : public AbstractCoordinate  
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

	PropertyDbl _weightProp;
	double &_weight;

	PropertyDblArray _rangeProp;
	Array<double>& _range;

	PropertyStrArray _keysProp;
	Array<std::string>& _keys;

	PropertyBool _clampedProp;
	bool &_clamped;

	PropertyBool _lockedProp;
	bool &_locked;

	PropertyObjArray _restraintFunctionProp;
	ArrayPtrs<Function> &_restraintFunction;

	PropertyObjArray _minRestraintFunctionProp;
	ArrayPtrs<Function> &_minRestraintFunction;

	PropertyObjArray _maxRestraintFunctionProp;
	ArrayPtrs<Function> &_maxRestraintFunction;

	PropertyBool _restraintActiveProp;
	bool &_restraintActive;

	Array<AbstractJoint*> _jointList; // list of joints that use this coordinate
	Array<SimmPath*> _pathList; // list of paths that use this coordinate

	AbstractDof::DofType _motionType; // rotational or translational (based on the DOFs it's used in)

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
	virtual double getWeight() const { return _weight; }
	virtual bool setWeight(double aWeight);
	virtual bool getWeightUseDefault() const { return _weightProp.getUseDefault(); }
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
	virtual Function* getRestraintFunction() const;
	virtual Function* getMinRestraintFunction() const;
	virtual Function* getMaxRestraintFunction() const;
	virtual AbstractDof::DofType getMotionType() const { return _motionType; }

	void getKeys(std::string rKeys[]) const;
	const Array<std::string>& getKeys() const { return _keys; }
	const Array<AbstractJoint*>& getJointList() const { return _jointList; }
	const Array<SimmPath*>& getPathList() const { return _pathList; }
	void determineType();

	virtual void peteTest() const;

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class SimmCoordinate
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __SimmCoordinate_h__


