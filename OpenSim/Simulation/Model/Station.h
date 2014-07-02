#ifndef __Station_h__
#define __Station_h__
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Station.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                         *
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
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Frame.h"

namespace OpenSim {

class Body;
//=============================================================================
//=============================================================================
/**
 * A class implementing a Fixed Station on a Body.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMSIMULATION_API Station : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Station, ModelComponent);
public:
	//==============================================================================
	// PROPERTIES
	//==============================================================================
	/** @name Property declarations
	These are the serializable properties associated with a Station. **/
	/**@{**/
	OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
		"The location (Vec3) of the station in some refernce frame. Frame will be specified as Connector.");
	/**@}**/

//=============================================================================
// Model Component Interface
//=============================================================================
	void connectToModel(Model& model);
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Station();
	virtual ~Station();
	const OpenSim::Frame& getReferenceFrame() const;
	void setReferenceFrame( const OpenSim::Frame& aFrame );
	//Station reexpressInFrame(const SimTK::State& s, OpenSim::Frame& aFrame) const;
private:
	void setNull();
	void constructProperties() OVERRIDE_11;
	void constructStructuralConnectors() OVERRIDE_11;

//=============================================================================
};	// END of class Station
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Station_h__


