#ifndef __Actuator_h__
#define __Actuator_h__

// Actuator.h
// Author: Ajay Seth
/*
 * Copyright (c)  2010, Stanford University. All rights reserved. 
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
#include "Force.h"


#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Model;
class VisibleObject;
class Controller;
class StateFunction;
class Coordinate;

//=============================================================================
//=============================================================================
/**
 * Base class for an actuator (e.g., a torque motor, muscle, ...) that requires
 * external input (controls) to generate force.
 *
 * @author Ajay Seth
 * @version 2.0
 */
template<int M> class OSIMSIMULATION_API Actuator_ : public Force
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Name suffixes. */
	Array<std::string> _controlSuffixes;
  
    SimTK::SubsystemIndex _subsystemIndex;
    int _numStateVariables;
  
    // indexes in SimTK::State cache
	SimTK::CacheEntryIndex _controlIndex;
    SimTK::CacheEntryIndex _forceIndex;
    SimTK::CacheEntryIndex _speedIndex;

	SimTK::DiscreteVariableIndex _overrideForceIndex;
 
	StateFunction* _overrideForceFunction;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Actuator_();
	Actuator_(const Actuator_ &aActuator);
	virtual ~Actuator_();
	virtual Object* copy() const = 0;
	virtual void copyPropertyValues(Actuator_& aActuator) { }


private:
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Actuator_& operator=(const Actuator_ &aActuator);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:
	// ModelComponent Interface
	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void initState(SimTK::State& state) const;
	virtual void setDefaultsFromState(const SimTK::State& state);

	// Update the geometry attached to the actuator. Use inertial frame.
	virtual void updateGeometry();

public:

	// CONTROLS
	virtual const SimTK::Vector& getControls( const SimTK::State& s ) const;
	virtual double getControl( const SimTK::State& s ) const;
	virtual void setControls(const SimTK::State &s, const SimTK::Vector& controls) const;
	virtual void setControl(const SimTK::State &s, double control) const;
	virtual int getNumControls() const {return M;}  

	virtual void setForce(const SimTK::State& s, double aForce) const; 
    virtual double getForce( const SimTK::State& s) const;
    virtual void setSpeed( const SimTK::State& s, double aspeed) const;
    virtual double getSpeed( const SimTK::State& s) const;
	virtual double getPower(const SimTK::State& s) const { return getForce(s)*getSpeed(s); }
	virtual double getStress(const SimTK::State& s) const;
	virtual double getOptimalForce() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s) const = 0;
	virtual void computeEquilibrium(SimTK::State& s) const { }


	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet) { }
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet) { }
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet) { }

	// Visible Object Support
	virtual void updateDisplayer(const SimTK::State& s) { }
	virtual void replacePropertyFunction(Function* aOldFunction, Function* aNewFunction);


    ///--------------------------------------------------------------------------
    /// Overriding forces
    ///--------------------------------------------------------------------------
    /// The force normally produced by an Actuator can be overriden and 
    /// When the Actuator's force is overriden, the Actuator will by defualt
    /// produce a constant force which can be set with setOverrideForce(). 
    /// If the override force is not a constant, setOverrideForceFunction() 
    /// can be used supply a function which computes the Actuator's force. 
    ///
    
    /**
    * enable/disable an Actuator's override force
    *  
    * @param s    current state
    * @param flag true = override Actuator's output force
    *             false = use Actuator's computed forc (normal operation)
    */
    void overrideForce(SimTK::State& s, bool flag ) const;

    /**
    *  return Actuator's override status
    */
    bool isForceOverriden(const SimTK::State& s ) const;

    /**
    * set the force value  used when  the override is true 
    *  
    * 
    * @param s      current state
    * @param value  value of override force   
    */
    void setOverrideForce(SimTK::State& s, double value ) const;

    /**
    * return override force 
    */
    double getOverrideForce(const SimTK::State& s ) const;

    /**
    * set the function used to compute the override  force 
    * 
    * @param StateFunction    pointer to object used to compute the force
    */
    void setOverrideForceFunction( StateFunction* );

    /**
    * return a read only pointer to the function used to compute the override force 
    */
    const StateFunction* getOverrideForceFunction( ) const;

    /**
    * return a writable pointer to the function used to compute the override force 
    */
    StateFunction* updOverrideForceFunction( );


    /**
    * set override force function back to default (constant) 
    */
    void resetOverrideForceFunction();

protected:
    double computeOverrideForce(const SimTK::State& s ) const;

//=============================================================================
};	// END of class Actuator_<M>
//=============================================================================

/**
 * Derived class for an actuator (e.g., a torque motor, muscle, ...) that 
 * requires excactly one external input (control) to generate force.
 *
 * @author Ajay Seth
 * @version 2.0
 */
class OSIMSIMULATION_API Actuator : public Actuator_<1>
{
protected:
	/** Bounds on control of this actuator. */
	PropertyDbl _propMinControl;
	PropertyDbl _propMaxControl;
	// REFERENCES
	double& _minControl;
	double& _maxControl;

//=============================================================================
// METHODS
//=============================================================================
public:
	//-------------------------------------------------------------------------
	// CONSTRUCTION
	//-------------------------------------------------------------------------
	Actuator();
	Actuator(const Actuator &aActuator);
	virtual ~Actuator();

	/** Assignment operator */
	Actuator& operator=(const Actuator &aActuator);

	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode();

public:
	// manage bounds on Control
	void setMinControl(const double& aMinControl) {
		_minControl=aMinControl;
	}

	double getMinControl() const {
		return _minControl;
	}

	void setMaxControl(const double& aMaxControl) {
		_maxControl=aMaxControl;
	}

	double getMaxControl() const {
		return _maxControl;
	}

private:
	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class Actuator
//=============================================================================

} // end of namespace OpenSim

#endif // __Actuator_h__


