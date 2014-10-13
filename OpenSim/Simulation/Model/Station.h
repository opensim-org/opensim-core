#ifndef OPENSIM_STATION_H_
#define OPENSIM_STATION_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Station.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include "OpenSim/Simulation/Model/RigidFrame.h"

namespace OpenSim {

class Body;

//=============================================================================
//=============================================================================
/**
 * A class implementing a Station. A Station is a point fixed to and defined with 
 * respect to a reference frame. The reference frame can be a Body (fixed 
 * to origin of a Body), a FixedFrame (affixed to another RigidFrame) or any 
 * other RigidFrame. The main functionality provided by Station is to find its 
 * location in any Frame.
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
		"The location (Vec3) of the station in a reference frame. "
		"Frame is specified as Connector.");
	/**@}**/

public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Station();
	virtual ~Station();
	/** getter of Reference Frame off which the Station is defined */
	const OpenSim::RigidFrame& getReferenceFrame() const;
	/** setter of Reference Frame off which the Station is defined */
	void setReferenceFrame(const OpenSim::RigidFrame& aFrame);
	/** Find this Station's location in any Frame */
	SimTK::Vec3 findLocationInFrame(const SimTK::State& s, OpenSim::Frame& aFrame) const;
private:
	void setNull();
	void constructProperties() override;
	void constructConnectors() override;

//=============================================================================
};	// END of class Station
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_STATION_H_


