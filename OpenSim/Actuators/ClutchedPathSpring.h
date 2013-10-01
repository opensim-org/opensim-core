#ifndef OPENSIM_CLUTCHED_PATH_SPRING_H_
#define OPENSIM_CLUTCHED_PATH_SPRING_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ClutchedPathSpring.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
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
#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/PathActuator.h>

//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * The ClutchedPathSpring is an actuator that has passive path spring behavior
 * only when the clutch is engaged. The clutch is engaged by a control signal 
 * of 1 and is off for a control signal of 0. Off means the spring is not 
 * engaged and the path is free to change length with the motion of the bodies 
 * it is connected to. The tension produced by the spring is proportional
 * to the stretch (z) from the instant that the clutch is engaged.
 *
 * The spring tension = x*(K*z)*(1+D*Ldot), where
 *		- x is the control signal to the actuator
 *		- z is the stretch in the spring
 *		- Ldot is the lengthening speed of the actuator
 *		- K is the spring's linear stiffness (N/m)
 *		- D is the spring's dissipation factor
 *
 * The ClutchedPathSpring maintains the "stretch", z, as an internal state with
 * the following dynamics:
 *
 * <table border="0">
 *      <tr><td>dz/dt = </td><td>Ldot, when x>0 (clutch is engaged)</td></tr>
 *		<tr><td></td><td> -(1/Tau)*z, (decay to zero), otherwise</td></tr>
 * </table>
 *
 * Note that the control signal, x, is clamped to (0,1).
 *
 * @author Ajay Seth
 */
class OSIMACTUATORS_API ClutchedPathSpring : public PathActuator {
OpenSim_DECLARE_CONCRETE_OBJECT(ClutchedPathSpring, PathActuator);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations
    These are the serializable properties of the %ClutchedPathSpring class.  */
    /**@{**/
	OpenSim_DECLARE_PROPERTY(stiffness, double,
		"The linear stiffness (N/m) of the ClutchedPathSpring");
	OpenSim_DECLARE_PROPERTY(dissipation, double,
		"The dissipation factor (s/m) of the ClutchedPathSpring");
	OpenSim_DECLARE_PROPERTY(relaxation_time_constant, double,
		"The time constant (s) for the spring to relax (go slack) after the clutch "
		"is disengaged (i.e. control == 0).  If the clutch is re-engaged within the " 
		"relaxation period there will be residual tension in the spring.");
	OpenSim_DECLARE_PROPERTY(initial_stretch, double,
		"The initial stretch (m) of the spring element. Note if the clucth is "
		"not engaged, the actuator will 'slip' until there is no stretch according "
		"to the relaxation_time_constant.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
	ClutchedPathSpring();
	
	/** Convenience constructor with ClutchedPathSpring parameters
     * @param name          the name of a %ClutchedPathSpring instance
	 * @param stiffness		the spring stiffness (K) in N/m
	 * @param dissipation	the dissipation factor (D) in s/m 
	 * @param relaxationTau	the spring relaxation time constant (Tau) in s 
	 * @param stretch0      the initial stretch of the spring in m */
	ClutchedPathSpring(const std::string& name, double stiffness,
		 double dissipation, double relaxationTau, double stretch0=0.0);

    // default destructor, copy constructor, copy assignment

	//--------------------------------------------------------------------------
	//  <B> Get and set ClutchedPathSpring properties </B>
	//--------------------------------------------------------------------------
	/** Spring stiffness in N/m when clutch is engaged. */
	double getStiffness() const   {   return get_stiffness(); }
	void setStiffness(double stiffness);
	/** Spring dissipation factor in s/m when clutch is engaged. */
	double getDissipation() const {   return get_dissipation(); }
	void setDissipation(double dissipation);
	/** Initial spring stretch in m. */
	double getInitialStretch() {return get_initial_stretch(); }
	void setInitialStretch(double stretch0);

	//--------------------------------------------------------------------------
	//  <B> State dependent values </B>
	//--------------------------------------------------------------------------
    /** The stretch in the PathSpring. The value of the stretch 
        can only be obtained after the system has been realized to
        Stage::Position*/
	double getStretch(const SimTK::State& s) const;
    /** The tension generated by the PathSpring. The value of the tension 
        can only be obtained after the system has been realized to
        Stage::Dynamics
        */
	double getTension(const SimTK::State& s) const;

protected:
	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	double computeActuation( const SimTK::State& s) const OVERRIDE_11;

    /** Override PathActuator method to calculate a color for use when
    the %ClutchedPathSpring's path is displayed in the visualizer. 
    @param[in] state    
        A SimTK::State already realized through Stage::Velocity. Do not 
        attempt to realize it any further.
    @returns 
        The desired color for the path as an RGB vector with each
        component ranging from 0 to 1, or NaN to indicate that the color
        should not be changed. **/
    SimTK::Vec3 computePathColor(const SimTK::State& state) const OVERRIDE_11;

    /** Implement ModelComponent interface. */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	void initStateFromProperties(SimTK::State& state) const OVERRIDE_11;
	void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;
	Array<std::string> getStateVariableNames() const OVERRIDE_11;
	SimTK::SystemYIndex getStateVariableSystemIndex(
							const std::string& stateVariableName) const OVERRIDE_11;

	SimTK::Vector computeStateVariableDerivatives(
							const SimTK::State& s) const OVERRIDE_11;


private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class ClutchedPathSpring

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_CLUTCHED_PATH_SPRING_H_


