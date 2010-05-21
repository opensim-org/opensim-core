#ifndef __Actuator_h__
#define __Actuator_h__

// Actuator.h
// Author: Frank C. Anderson, Peter Loan
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
#include "Force.h"
#include "SimTKsimbody.h"

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

//=============================================================================
//=============================================================================
/**
 * An abstract class for representing an actuator (e.g., a torque motor,
 * muscle, ...).
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API Actuator : public Force
{

//=============================================================================
// DATA
//=============================================================================
public:
	static const double LARGE;

	/** Model which the actuator actuates. */
	//Model *_model;

protected:
    /** Flag indicating whether the actuator applies a force or a torque. */
    bool _appliesForce;

	/** Name suffixes. */
	Array<std::string> _controlSuffixes;
	Array<std::string> _stateVariableSuffixes;
  
    SimTK::SubsystemIndex _subsystemIndex;
    int _numStateVariables;

    // pointer to the controller that computes controls for this acuator and 
    // the index the controller uses to identify the actuator
    const Controller* _controller;
    int _controlIndex;
    
    // flag used during intialization to determine if an actator has been assinged a controller yet
    bool _isControlled;

    // indexes in SimTK::State cache
    SimTK::CacheEntryIndex _forceIndex;
    SimTK::CacheEntryIndex _speedIndex;

    // indexes into SimTK::State 
    mutable SimTK::ZIndex _zIndex; // index of (z's) for this actuator
    SimTK::CacheEntryIndex _stateVariableDerivIndex; // index of state derivaties (zdots) for this actuator

     SimTK::DiscreteVariableIndex _isOverridenIndex;
     SimTK::DiscreteVariableIndex _overrideForceIndex;
 
     StateFunction* _overrideForceFunction;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Actuator();
	Actuator(const Actuator &aActuator);
	virtual ~Actuator();
	virtual Object* copy() const = 0;
	virtual void copyPropertyValues(Actuator& aActuator) { }
	static void deleteActuator(Actuator* aActuator) { if (aActuator) delete aActuator; }

   virtual void initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model);


private:
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Actuator& operator=(const Actuator &aActuator);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:
    void setNumStateVariables( int aNumStateVariables);
	void bindStateVariable( int aIndex,const std::string &aSuffix);


	virtual void setup(Model& aModel);

public:

	// MODEL
	Model& getModel() const { return *_model; }
	// CONTROLS
	virtual int getControlIndex() const;
	virtual void setControlIndex(int index);
	virtual void setController(const Controller*);
    virtual const Controller& getController() const;
	virtual double getControl( const SimTK::State& s ) const;
	// STATES
	virtual std::string getStateVariableName( int aIndex) const;
	virtual int getStateVariableIndex(const std::string &aName) const;
	virtual void setStateVariable( SimTK::State& s, int aIndex,double aValue) const ;
	virtual void setStateVariables( SimTK::State& s, const double aY[]) const ;
	virtual double getStateVariable( const SimTK::State& s, int aIndex) const;
	virtual void getStateVariables( const SimTK::State& s, double rY[]) const;
    virtual int getNumStateVariables() const;
	virtual void setStateVariableDeriv( const SimTK::State& s, int aIndex,double aValue) const ;
	virtual void setStateVariableDerivs( const SimTK::State& s, const double aY[]) const ;
	virtual double getStateVariableDeriv( const SimTK::State& s, int aIndex) const;
	virtual void getStateVariableDerivs( const SimTK::State& s, double rY[]) const;
	// Visible Object Support
	//virtual VisibleObject* getDisplayer() const { return NULL; }
	virtual void updateDisplayer(const SimTK::State& s) { }
	virtual void replacePropertyFunction(Function* aOldFunction, Function* aNewFunction);
	OPENSIM_DECLARE_DERIVED(Actuator, Object);

protected:
	// FORCE
	void setAppliesForce(bool aTrueFalse) { _appliesForce = aTrueFalse; }
	// Update the geometry attached to the actuator. Use inertial frame.
	virtual void updateGeometry();

public:
    bool getAppliesForce() const { return _appliesForce; }
	virtual void setForce(const SimTK::State& s, double aForce) const; 
    virtual double getForce( const SimTK::State& s) const;
    virtual double getAppliedForce( const SimTK::State& s) const;
    virtual void setSpeed( const SimTK::State& s, double aspeed) const;
    virtual double getSpeed( const SimTK::State& s) const;
	virtual double getPower(const SimTK::State& s) const { return getForce(s)*getSpeed(s); }
	virtual double getStress(const SimTK::State& s) const;
	virtual double getOptimalForce() const;
    virtual void setIsControlled(bool flag) { _isControlled = flag; }
    virtual bool isControlled() const { return _isControlled; }

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const SimTK::State& s, int index ) { }
	virtual double computeActuation( const SimTK::State& s) const = 0;
	virtual void computeStateDerivatives(const SimTK::State& s ) { }
	virtual void computeEquilibrium(SimTK::State& s) const { }

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const { return true; }

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet) { }
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet) { }
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet) { }
    int getNumControls() { return 1; }  // all actuators can have one control

    //--------------------------------------------------------------------------
    // Overriding forces
    //--------------------------------------------------------------------------
    void overrideForce(SimTK::State& s, bool flag ) const;
    bool isForceOverriden(const SimTK::State& s ) const;
    void setOverrideForce(SimTK::State& s, double value ) const;
    double getOverrideForce(const SimTK::State& s ) const;
    double computeOverrideForce(const SimTK::State& s ) const;
    void setOverrideForceFunction( StateFunction* );
    const StateFunction* getOverrideForceFunction( ) const;
    StateFunction* updOverrideForceFunction( );
    void resetOverrideForceFunction();


//=============================================================================
};	// END of class Actuator
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Actuator_h__


