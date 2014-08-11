#ifndef OPENSIM_BODYFRAME_H_
#define OPENSIM_BODYFRAME_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  BodyFrame.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Frame.h>


namespace OpenSim {

class Model;
class Body;

//=============================================================================
//=============================================================================
/**
 * A frame attached to the geometric origin of a Body. The Body is specified through 
 * the Connector mechanism.
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API BodyFrame : public Frame {
	OpenSim_DECLARE_CONCRETE_OBJECT(BodyFrame, Frame);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Body. **/
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
	BodyFrame();

	/** Convenience constructor */	
	BodyFrame(const Body& body);
	

	// use compiler generated destructor, copy constructor and assignment operator

	/** Access Properties of the Body */

	/** Spatial Operations for Frames*/
	const SimTK::Transform& getTransform() const override;
	const SimTK::Transform calcGroundTransform(const SimTK::State &state) const override;
	void setBody(const Body& body);
	const Body& getBody();

protected:
    // Model component interface.
	void constructStructuralConnectors() override;

	


private:


	void setNull();


//=============================================================================
};	// END of class Body
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_BODYFRAME_H_


