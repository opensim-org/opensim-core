#ifndef OPENSIM_ACTIVATION_FIBER_LENGTH_MUSCLE_H_
#define OPENSIM_ACTIVATION_FIBER_LENGTH_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  ActivationFiberLengthMuscle.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDE
#include "Muscle.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

//==============================================================================
//                    ACTIVATION FIBER LENGTH MUSCLE
//==============================================================================
/**
 * A base class representing a two-state muscle-tendon actuator. 
 * It adds activation and fiber-length states and dynamics to the 
 * Muscle class, but does not implement all of the necessary methods,
 * so it is abstract as well. The path information for a muscle is contained
 * in the Muscle class, and the force-generating behavior should be defined in
 * the derived classes.
 *
 * @author Ajay Seth
 *
 * (Based on earlier work by Peter Loan and Frank C. Anderson.)
 */
class OSIMSIMULATION_API ActivationFiberLengthMuscle : public Muscle {
OpenSim_DECLARE_ABSTRACT_OBJECT(ActivationFiberLengthMuscle, Muscle);

public:
//===============================================================================
// PROPERTIES
//===============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(default_activation, double,
		"Assumed activation level if none is assigned.");
	OpenSim_DECLARE_PROPERTY(default_fiber_length, double,
		"Assumed fiber length, unless otherwise assigned.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	ActivationFiberLengthMuscle();

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    //--------------------------------------------------------------------------
    // ActivationFiberLengthMuscle Parameters
    //--------------------------------------------------------------------------
    // Defaults
    double getDefaultActivation() const;
    void setDefaultActivation(double activation);
    double getDefaultFiberLength() const;
    void setDefaultFiberLength(double length);

	//--------------------------------------------------------------------------
    // State Variables
    //--------------------------------------------------------------------------
	void setActivation(SimTK::State& s, double activation) const;
	void setFiberLength(SimTK::State& s, double fiberLength) const;

	virtual Array<std::string> getStateVariableNames() const OVERRIDE_11;
	virtual SimTK::SystemYIndex getStateVariableSystemIndex(const std::string &stateVariableName) const OVERRIDE_11;

	//--------------------------------------------------------------------------
    // State Variable Derivative
    //--------------------------------------------------------------------------
	double getActivationRate(const SimTK::State& s) const;
	



	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
protected:
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

	//--------------------------------------------------------------------------
	// FORCE APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForce) const;

	/** Calculate activation rate */
	virtual double calcActivationRate(const SimTK::State& s) const = 0;

	/* compute initial fiber length (velocity) such that muscle fiber and tendon are 
	    in static equilibrium and update the state */
    //virtual void computeInitialFiberEquilibrium(SimTK::State& s) const;
    
	/** Model Component Interface */
	virtual void addToSystem(SimTK::MultibodySystem& system) const;
	virtual void initStateFromProperties(SimTK::State& s) const;
    virtual void setPropertiesFromState(const SimTK::State& state);
    virtual void connectToModel(Model& aModel) OVERRIDE_11;
	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

	/** State derivative access helper methods */
	void setStateVariableDeriv(const SimTK::State& s, const std::string &aStateName, double aValue) const;
	double getStateVariableDeriv(const SimTK::State& s, const std::string &aStateName) const;

	static const std::string STATE_ACTIVATION_NAME;
	static const std::string STATE_FIBER_LENGTH_NAME;   

private:
	void constructProperties();

//==============================================================================
};	// END of class ActivationFiberLengthMuscle
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ACTIVATION_FIBER_LENGTH_MUSCLE_H_


