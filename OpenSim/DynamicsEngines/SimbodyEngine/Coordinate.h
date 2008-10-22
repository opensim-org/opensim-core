#ifndef __Coordinate_h__
#define __Coordinate_h__

// Coordinate.h
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
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/AbstractTransformAxis.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class Joint;
class SimbodyEngine;
//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody generalized coordinate.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API Coordinate : public AbstractCoordinate  
{
//=============================================================================
// DATA
//=============================================================================

private:
	// All coordinates (Simbody mobility) have associated constraints that
	// perform joint locking, prescribed motion and range of motion.
	// Constraints are created upon setup: locked, precribedFunction
	// and range must be set.
	// NOTE: Changing the prescribed motion function requires topology to be realized
	//       so state is invalidated
	//       Enabling/disabling locking, prescribed motion or clamping is allowable 
	//       during a simulation.
	//       The last constraint to be set takes precedence.
	/** ID for the constraint in Simbody. */
	SimTK::ConstraintIndex _prescribedConstraintIndex;
	SimTK::ConstraintIndex _lockedConstraintIndex;
	SimTK::ConstraintIndex _clampedConstraintIndex;

protected:
	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	PropertyDbl _initialValueProp;
	double &_initialValue;

	PropertyDbl _toleranceProp;
	double &_tolerance;

	PropertyDbl _stiffnessProp;
	double &_stiffness;

	PropertyDblArray _rangeProp;
	Array<double>& _range;

	PropertyStrArray _keysProp;
	Array<std::string>& _keys;

	/** Flag indicating whether the coordinate is clamped or not.  Clamped means
	that the coordinate is not allowed to go outside its range. */
	PropertyBool _clampedProp;
	bool &_clamped;

	/** Flag indicating whether the coordinate is locked or not.  Locked means
	fixed at one value. */
	PropertyBool _lockedProp;
	bool &_locked;

	/** Flag indicating whether the coordinate is prescribed or not.  Prescribed means
	the value will vary with time according to the Prescribed Function, which must be set first. */
	PropertyBool _isPrescribedProp;
	bool &_isPrescribed;

	/** Flag specifying what joint of coordinate this coordinate is (constrained, unconstrained). */
	PropertyInt _QTypeProp;
	int &_QType;

	/** Specify the desired prescribed motion as a function of time. */
	PropertyObjPtr<OpenSim::Function> _prescribedFunctionProp;
	OpenSim::Function *&_prescribedFunction;

	PropertyObjPtr<OpenSim::Function> _restraintFunctionProp;
	OpenSim::Function *&_restraintFunction;

	PropertyObjPtr<OpenSim::Function> _minRestraintFunctionProp;
	OpenSim::Function *&_minRestraintFunction;

	PropertyObjPtr<OpenSim::Function> _maxRestraintFunctionProp;
	OpenSim::Function *&_maxRestraintFunction;

	PropertyBool _restraintActiveProp;
	bool &_restraintActive;

	/** ID of the body which this coordinate serves.  */
	SimTK::MobilizedBodyIndex _bodyIndex;

	/** Mobility index for this coordinate. */
	int _mobilityIndex;

	/** Motion type (rotational or translational). */
	AbstractTransformAxis::MotionType _motionType;

	/** Simbody dynamics engine that contains this coordinate. */
	//SimbodyEngine *_engine;

	/** Simbody joint that owns this coordinate. */
	Joint *_joint;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Coordinate();
	Coordinate(const Coordinate &aCoordinate);
	Coordinate(const AbstractCoordinate &aCoordinate);
	virtual ~Coordinate();
	virtual Object* copy() const;

	Coordinate& operator=(const Coordinate &aCoordinate);
	void copyData(const Coordinate &aCoordinate);
	void copyData(const AbstractCoordinate &aCoordinate);

	void setup(AbstractDynamicsEngine* aEngine);
	SimbodyEngine* getEngine() const {
		return (SimbodyEngine*)(_dynamicsEngine);
	}

	void initializeState(SimTK::State& completeState);

	virtual void setJoint(Joint *aOwningJoint);
	virtual Joint* getJoint() const;
	virtual void updateFromCoordinate(const AbstractCoordinate &aCoordinate);
	
	virtual double getValue() const;
	virtual bool setValue(double aValue);
	virtual bool setValue(double aValue, bool aRealize=false);

	virtual bool setValue(double aValue, SimTK::State& theState, bool aRealize=true);
	virtual bool getValueUseDefault() const { return true; }
	
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
	
	// Legacy stuff from SIMM models
	virtual double getStiffness() const { return _stiffness; }
	virtual bool setStiffness(double aStiffness);
	virtual bool getStiffnessUseDefault() const { return _stiffnessProp.getUseDefault(); }
	
	virtual double getDefaultValue() const { return _defaultValue; }
	virtual bool setDefaultValue(double aDefaultValue);
	virtual double getInitialValue() const { return _initialValue; }
	virtual void setInitialValue(double aInitialValue);
	virtual bool getDefaultValueUseDefault() const { return _defaultValueProp.getUseDefault(); }
	
	// Clamping coordinate between a range of values
	virtual bool getClamped() const { return _clamped; }
	virtual bool setClamped(bool aClamped);
	virtual bool setClamped(bool aClamped, SimTK::State& theState);
	virtual bool getClampedUseDefault() const { return _clampedProp.getUseDefault(); }
	
	//Locking for the coordinate to the current value
	virtual bool getLocked() const;
	virtual bool setLocked(bool aLocked);
	virtual bool setLocked(bool aLocked, SimTK::State& theState);
	virtual bool getLockedUseDefault() const { return _lockedProp.getUseDefault(); }

	//Prescribed motion for Coordinate
	virtual bool isPrescribed() const {return _isPrescribed;}
	void setIsPrescribed(bool isPrescribed);
	void setIsPrescribed(bool isPrescribed, SimTK::State& theState);
	void setPrescribedFunction(const Function *function);
	virtual OpenSim::Function* getPrescribedFunction() const;

	virtual void getKeys(std::string rKeys[]) const;
	virtual const Array<std::string>& getKeys() const { return _keys; }
	virtual bool isUsedInModel() const { return true; }
	bool isRestraintActive() const { return _restraintActive; }
	OpenSim::Function* getRestraintFunction() const;
	OpenSim::Function* getMinRestraintFunction() const;
	OpenSim::Function* getMaxRestraintFunction() const;

	virtual AbstractTransformAxis::MotionType getMotionType() const { return _motionType; }
	virtual void setMotionType(AbstractTransformAxis::MotionType aMotionType) {_motionType = aMotionType; } 
	// Return true if coordinate is dependent on other coordinates 
	virtual bool isConstrained() const; 

	OPENSIM_DECLARE_DERIVED(Coordinate, AbstractCoordinate); 


private:
	void setNull();
	void setupProperties();
	void determineType();
	void createConstraintsForLockClampPrescribed();
	friend class SimbodyEngine;
	friend class Constraint; 
	friend class CoordinateCouplerConstraint; 
	friend class Joint; 

//=============================================================================
};	// END of class Coordinate
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Coordinate_h__


