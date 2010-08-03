#ifndef __Coordinate_h__
#define __Coordinate_h__

// Coordinate.h
// Author: Frank C. Anderson, Ajay Seth, Jeffrey A. Reinbolt
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <SimTKsimbody.h>

namespace OpenSim {

class Function;
class Joint;
class Model;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Simbody generalized coordinate.
 *
 * @author Frank C. Anderson, Ajay Seth, Jeffrey A. Reinbolt
 * @version 1.0
 */
class OSIMSIMULATION_API Coordinate : public ModelComponent  
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Motion type that is described by the coordinate */
    enum MotionType
    {
        Rotational,
		Translational,
		Coupled
    };

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

	mutable bool _lockedWarningGiven;
protected:

	PropertyStr _motionTypeNameProp;
	std::string &_motionTypeName;

	PropertyDbl _defaultValueProp;
	double &_defaultValue;

	PropertyDbl _defaultSpeedValueProp;
	double &_defaultSpeedValue;

	PropertyDbl _initialValueProp;
	double &_initialValue;

	PropertyDblArray _rangeProp;
	Array<double>& _range;


	/** Flag indicating whether the coordinate is clamped or not.  Clamped means
	that the coordinate is not allowed to go outside its range. */
	PropertyBool _clampedProp;
	bool &_clamped;

	/** Flag indicating whether the coordinate is locked or not.  Locked means
	fixed at one value. */
	PropertyBool _lockedProp;
	bool &_locked;
	SimTK::Function *_lockFunction;

	/** Flag indicating whether the coordinate is prescribed or not.  Prescribed means
	the value will vary with time according to the Prescribed Function, which must be set first. */
	PropertyBool _isPrescribedProp;
	bool &_isPrescribed;

	/** Specify the desired prescribed motion as a function of time. */
	PropertyObjPtr<OpenSim::Function> _prescribedFunctionProp;
	OpenSim::Function *&_prescribedFunction;

	/** ID of the body which this coordinate serves.  */
	SimTK::MobilizedBodyIndex _bodyIndex;

	/** Mobility index for this coordinate. */
	int _mobilityIndex;

	/** Motion type (translational, rotational or combination). */
	MotionType _motionType;

	/** Simbody joint that owns this coordinate. */
	Joint const *_joint;

	int _qIndex;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Coordinate();
	Coordinate(const std::string &aName, MotionType aMotionType, double aRangeMin, double aRangeMax);	
	Coordinate(const Coordinate &aCoordinate);
	virtual ~Coordinate();
	virtual Object* copy() const;
#ifndef SWIG
	Coordinate& operator=(const Coordinate &aCoordinate);
#endif
	void copyData(const Coordinate &aCoordinate);

	virtual void setup(Model& aModel);

	virtual void setJoint(const Joint& aOwningJoint);
	virtual const Joint& getJoint() const;

	Model& getModel() const { return *_model; }
	virtual void updateFromCoordinate(const Coordinate &aCoordinate);
	virtual double getValue(const SimTK::State& s) const;
    virtual bool setValue(SimTK::State& s, double aValue, bool aEnforceContraints=true) const;

	virtual double getSpeedValue(const SimTK::State& s) const;
    virtual bool setSpeedValue(SimTK::State& s, double aValue) const;
	const std::string  getSpeedName() const;

	virtual double getAccelerationValue(const SimTK::State& s) const;

	virtual bool getValueUseDefault() const { return true; }
	
	virtual void getRange(double rRange[2]) const { rRange[0] = _range[0]; rRange[1] = _range[1]; }
	virtual bool setRange(double aRange[2]);
	virtual double getRangeMin() const { return _range[0]; }
	virtual double getRangeMax() const { return _range[1]; }
	virtual bool setRangeMin(double aMin);
	virtual bool setRangeMax(double aMax);
	virtual bool getRangeUseDefault() const { return _rangeProp.getUseDefault(); }
	
	virtual double getDefaultValue() const { return _defaultValue; }
	virtual bool setDefaultValue(double aDefaultValue);

	double getDefaultSpeedValue() const { return _defaultSpeedValue; }
	void setDefaultSpeedValue(double aDefaultSpeedValue) { _defaultSpeedValue = aDefaultSpeedValue; };

	virtual double getInitialValue() const { return _initialValue; }
	virtual void setInitialValue(double aInitialValue);
	virtual bool getDefaultValueUseDefault() const { return _defaultValueProp.getUseDefault(); }
	
	// Clamping coordinate between a range of values
	virtual bool getClamped(const SimTK::State& s) const;
	virtual bool setClamped(SimTK::State& s, bool aClamped ) const;
	virtual bool getDefaultClamped() const { return _clamped; }
    virtual void setDefaultClamped(bool aClamped ) { _clamped = aClamped; }
	virtual bool getClampedUseDefault() const { return _clampedProp.getUseDefault(); }
	
	//Locking for the coordinate to the current value
	virtual bool getLocked(const SimTK::State& s) const;
	virtual bool setLocked(SimTK::State& s, bool aLocked) const;
    virtual bool getDefaultLocked() const { return _locked; }
    virtual void setDefaultLocked(bool aLocked) { _locked = aLocked; }
	virtual bool getLockedUseDefault() const { return _lockedProp.getUseDefault(); }

	//Prescribed motion for Coordinate
	virtual bool isPrescribed(const SimTK::State& s) const;
	void setIsPrescribed(SimTK::State& s, bool isPrescribed ) const;
	virtual bool getDefaultIsPrescribed() const {return _isPrescribed;}
    void setDefaultIsPrescribed(bool isPrescribed ) {_isPrescribed = isPrescribed;}
	void setPrescribedFunction(const Function& function);
	virtual OpenSim::Function* getPrescribedFunction() const;

	virtual MotionType getMotionType() const { return _motionType; }
	virtual void setMotionType(MotionType aMotionType);
	
	// Return true if coordinate is dependent on other coordinates via a coupler constraint
	virtual bool isDependent(const SimTK::State& s) const;

	/** Return true if coordinate is locked, prescribed, or dependent on other coordinates */
	virtual bool isConstrained(const SimTK::State& s) const; 

	int getMobilityIndex() const { return _mobilityIndex; };
	SimTK::MobilizedBodyIndex getBodyIndex() const { return _bodyIndex; };

	virtual std::string getStateVariableName(int index) const;
	virtual int getStateVariableYIndex(int index) const;

	OPENSIM_DECLARE_DERIVED(Coordinate, Object); 

protected:
	/* Only model should be invoking these */
    virtual void createSystem(SimTK::MultibodySystem& system) const;
    virtual void initState(SimTK::State& s) const;
    virtual void setDefaultsFromState(const SimTK::State& state);
	virtual int getNumStateVariables() const { return 2; };	// For value and Speed

private:
	void setNull();
	void setupProperties();
	void determineType();

	friend class Constraint; 
	friend class CoordinateCouplerConstraint; 
	friend class Joint; 
    friend class Model;

//=============================================================================
};	// END of class Coordinate
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Coordinate_h__


