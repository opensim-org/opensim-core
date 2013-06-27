#ifndef OPENSIM_RIGID_TENDON_MUSCLE_H_
#define OPENSIM_RIGID_TENDON_MUSCLE_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  RigidTendonMuscle.h                        *
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
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/PropertyObjPtr.h>

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

//==============================================================================
//                          RIGID TENDON MUSCLE
//==============================================================================
/**
 * A class implementing a RigidTendonMuscle actuator with no states.
 * The path information for a RigidTendonMuscle is contained
 * in the base class, and the force-generating behavior should is defined in
 * this class. The force (muscle tension) assumes rigid tendon so that 
 * fiber-length and velocity are kinematics dependent and the force-length
 * force-velocity relationships are evaluated directly.
 * The control of this model is its activation. Force production is instantaneous  
 * with no excitation-to-activation dynamics and excitation=activation.
 *
 * @author Ajay Seth
 */
class OSIMACTUATORS_API RigidTendonMuscle : public Muscle {
OpenSim_DECLARE_CONCRETE_OBJECT(RigidTendonMuscle, Muscle);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(active_force_length_curve, Function,
		"Function representing active force-length behavior of muscle fibers");
	OpenSim_DECLARE_PROPERTY(passive_force_length_curve, Function,
		"Function representing passive force-length behavior of muscle fibers");
	OpenSim_DECLARE_PROPERTY(force_velocity_curve, Function,
		"Function representing force-velocity behavior of muscle fibers");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	RigidTendonMuscle();
	RigidTendonMuscle(const std::string&    name,
                      double                maxIsometricForce,
                      double                optimalFiberLength,
                      double                tendonSlackLength,
                      double                pennationAngle);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	/** activation level for this muscle */
	void setActivation(SimTK::State& s, double activation) const {setExcitation(s, activation); }

protected:

	/** calculate muscle's length related values such fiber and tendon lengths,
		normalized lengths, pennation angle, etc... */
	void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const;

	/** calculate muscle's velocity related values such fiber and tendon velocities,
		normalized velocities, pennation angular velocity, etc... */
	void  calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const;

	/** calculate muscle's active and passive force-length, force-velocity, 
	    tendon force, relationships and their related values */
	void  calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const;

	/** calculate muscle's fiber and tendon potential energy */
	void calcMusclePotentialEnergyInfo(const SimTK::State& s,
		MusclePotentialEnergyInfo& mpei) const;

	/** compute initial fiber length (velocity) such that muscle fiber and tendon are 
	    in static equilibrium and update the state */
	void computeInitialFiberEquilibrium(SimTK::State& s) const {}

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	double computeActuation( const SimTK::State& s ) const;
	double computeIsometricForce(SimTK::State& s, double activation) const;
	void equilibrate(SimTK::State& s) const {}
    

private:
	void setNull();
	void constructProperties();

protected:

//==============================================================================
};	// END of class RigidTendonMuscle
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_RIGID_TENDON_MUSCLE_H_


