#ifndef OPENSIM_RIGIDFRAME_H_
#define OPENSIM_RIGIDFRAME_H_
/* -------------------------------------------------------------------------- *
*                              OpenSim:  RigidFrame.h                             *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2013 Stanford University and the Authors                *
* Author(s): Matt DeMers, Ajay Seth, Ayman Habib                             *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/Frame.h>



namespace OpenSim {

	class Model;
    class Body;
	//=============================================================================
	//=============================================================================
	/**
	* A RigidFrame is a modeling abstraction that defines a Righthanded CoordinateSystem
	* attached rigidly to another Frame
	*
	* @author Matt DeMers
	*/
	class OSIMSIMULATION_API RigidFrame : public Frame {
		OpenSim_DECLARE_ABSTRACT_OBJECT(RigidFrame, Frame);
	public:
		//==============================================================================
		// PROPERTIES
		//==============================================================================
		/** @name Property declarations
		These are the serializable properties associated with a RigidFrame. **/
		/**@{**/
		/**@}**/
	protected:



		//=============================================================================
		// PUBLIC METHODS
		//=============================================================================

	public:
		//--------------------------------------------------------------------------
		// CONSTRUCTION
		//--------------------------------------------------------------------------
		/** default contructor*/
        RigidFrame();

        virtual const OpenSim::Body& getBody() const = 0;

	private:
		void setNull();

	protected:

		//=============================================================================
	};	// END of class RigidFrame
	//=============================================================================
	//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_RIGIDFRAME_H_


