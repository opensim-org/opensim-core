#ifndef OPENSIM_ACTIVATION_FIBER_LENGTH_MUSCLE_DEPRECATED_H_
#define OPENSIM_ACTIVATION_FIBER_LENGTH_MUSCLE_DEPRECATED_H_
/* -------------------------------------------------------------------------- *
 *             OpenSim:  ActivationFiberLengthMuscle_Deprecated.h             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Peter Loan, Frank C. Anderson                        *
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
//               ACTIVATION FIBER LENGTH MUSCLE (DEPRECATED)
//==============================================================================
/**
 * A base class representing a muscle-tendon actuator. It adds states to the 
 * Muscle class, but does not implement all of the necessary methods,
 * so it is abstract too. The path information for a muscle is contained
 * in this class, and the force-generating behavior should be defined in
 * the derived classes.
 *
 * @author Peter Loan, Frank C. Anderson, Ajay Seth
 */
class OSIMSIMULATION_API ActivationFiberLengthMuscle_Deprecated 
:   public Muscle {
OpenSim_DECLARE_ABSTRACT_OBJECT(ActivationFiberLengthMuscle_Deprecated, Muscle);

public:
//================================================================================
// PROPERTIES
//================================================================================
    /** @name Property declarations
    There are no serializable properties associated with this class. **/
    /**@{**/
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ActivationFiberLengthMuscle_Deprecated();

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	virtual void equilibrate(SimTK::State& state) const;

    //--------------------------------------------------------------------------
    // GET
    //--------------------------------------------------------------------------
    // Defaults
    virtual double getDefaultActivation() const;
    virtual void setDefaultActivation(double activation);
    virtual double getDefaultFiberLength() const;
    virtual void setDefaultFiberLength(double length);

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double getFiberLength(const SimTK::State& s) const;
    virtual void setFiberLength(SimTK::State& s, double fiberLength) const;
	virtual double getFiberLengthDeriv(const SimTK::State& s) const;
	virtual void setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const;
	virtual double getNormalizedFiberLength(const SimTK::State& s) const;
	virtual double getFiberLengthAlongTendon(const SimTK::State& s) const;
	virtual double getTendonLength(const SimTK::State& s) const;
	virtual double getFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForce(const SimTK::State& s) const;
	virtual double getPassiveFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getPassiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getPassiveForce( const SimTK::State& s) const;
	virtual void setPassiveForce(const SimTK::State& s, double aForce) const;
	virtual double getTendonForce(const SimTK::State& s) const;
	virtual void setTendonForce(const SimTK::State& s, double aForce) const;
	virtual double getActivation(const SimTK::State& s) const;
    virtual void setActivation(SimTK::State& s, double activation) const;
	virtual double getActivationDeriv(const SimTK::State& s) const;
	virtual void setActivationDeriv(const SimTK::State& s, double activationDeriv) const;
    virtual double getExcitation( const SimTK::State& s) const;
	virtual double getStress(const SimTK::State& s) const;


	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeInitialFiberEquilibrium(SimTK::State& s ) const;
	virtual double computeActuation( const SimTK::State& s ) const = 0;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const = 0;
	//virtual double computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation) const;
  	
    virtual double calcInextensibleTendonActiveFiberForce(SimTK::State& s, 
                                                      double aActivation) const;

    virtual double evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity) const;
 	virtual double calcPennation( double aFiberLength, double aOptimalFiberLength, double aInitialPennationAngle) const;
 
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
public:

	virtual Array<std::string> getStateVariableNames() const;
	virtual SimTK::SystemYIndex getStateVariableSystemIndex(const std::string &stateVariableName) const;


protected:
	virtual void addToSystem(SimTK::MultibodySystem& system) const;
	virtual void initStateFromProperties(SimTK::State& s) const;
    virtual void setPropertiesFromState(const SimTK::State& state);

	virtual void setStateVariableDeriv(const SimTK::State& s, const std::string &aStateName, double aValue) const;
	virtual double getStateVariableDeriv(const SimTK::State& s, const std::string &aStateName) const;

	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

	// Muscle interface
	virtual void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const;
	virtual void calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const;
	virtual void calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const;
	/** calculate muscle's fiber and tendon potential energy */
	void calcMusclePotentialEnergyInfo(const SimTK::State& s, 
		MusclePotentialEnergyInfo& mpei) const;
	
	virtual double calcActiveForce(const SimTK::State& s, double aNormFiberLength) const
	{
		throw Exception("ERROR- "+getConcreteClassName()+"::calcActiveForce() NOT IMPLEMENTED.");
	}
	virtual double calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const
	{
		throw Exception("ERROR- "+getConcreteClassName()+"::calcPassiveForce() NOT IMPLEMENTED.");
	}

	static const int STATE_ACTIVATION;
	static const int STATE_FIBER_LENGTH;

	static const std::string STATE_ACTIVATION_NAME;
	static const std::string STATE_FIBER_LENGTH_NAME;


//==============================================================================
// DATA
//==============================================================================
protected:
	// Defaults for state variables.
    double _defaultActivation;
    double _defaultFiberLength;


private:
	void setNull();
	void constructProperties();
//==============================================================================
};	// END of class ActivationFiberLengthMuscle_Deprecated
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ACTIVATION_FIBER_LENGTH_MUSCLE_DEPRECATED_H_


