#ifndef OPENSIM_BODY_ACTUATOR_H_
#define OPENSIM_BODY_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
*                         OpenSim:  BodyActuator.h                          *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2014 Stanford University and the Authors                     *
* Author(s): Soha Pouya, Michael Sherman                                     *
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
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim {

	class Body;
	class Model;

	//==============================================================================
	//                              BODY ACTUATOR
	//==============================================================================
	/**
	* Apply a spatial force (that is, force and torque) on the origin of the given
	* body. That is, the force is applied at the origin; torques don't have
	* associated points. This actuator has no states; the control is simply the
	* force/torque to be applied to the model and is passed through directly.
	* The associated Controller is expected to generate forces/torques in the body
	* frame.
	*
	* @author Soha Pouya, Michael Sherman
	*/
	class OSIMACTUATORS_API BodyActuator : public Actuator {
		OpenSim_DECLARE_CONCRETE_OBJECT(BodyActuator, Actuator);
	public:
		//==============================================================================
		// PROPERTIES
		//==============================================================================
		/** @name Property declarations
		There are no serializable properties associated with this class (except
		inherited ones). **/
		/**@{**/
		OpenSim_DECLARE_OPTIONAL_PROPERTY(body, std::string,
			"Name of Body to which this actuator is applied.");
		/**@}**/

		//==============================================================================
		// PUBLIC METHODS
		//==============================================================================
		/** Default constructor or construct with body name given. An empty
		name ("") is treated as though it were unspecified. **/
		explicit BodyActuator(const std::string& bodyName = "");

		// Uses default (compiler-generated) destructor, copy constructor, copy 
		// assignment operator.

	private:
		void constructProperties();
		//--------------------------------------------------------------------------
		// Implement Component interface
		//--------------------------------------------------------------------------
		void constructStructuralConnectors() override;

		//--------------------------------------------------------------------------
		// Implement Force interface
		//--------------------------------------------------------------------------
		void computeForce(const SimTK::State& state,
			SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
			SimTK::Vector& mobilityForces) const override;

		//--------------------------------------------------------------------------
		// Implement Actuator interface.
		//--------------------------------------------------------------------------
		int numControls() const override { return 6; }

		double getPower(const SimTK::State& s) const { return 0; }
		void overrideForce(SimTK::State& s, bool flag) const {}

		//--------------------------------------------------------------------------
		// Implement ModelComponent interface
		//--------------------------------------------------------------------------
		// Setup method to initialize Body reference
		void connectToModel(Model& model) override;

		//=============================================================================
	};	// END of class BodyActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_BODY_ACTUATOR_H_


