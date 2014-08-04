#ifndef OPENSIM_FIXEDFRAME_H_
#define OPENSIM_FIXEDFRAME_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  FixedFrame.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib                                          *
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
 *  FixedFrame a Frame based off another Frame that could be either a FixedFrame or BodyFrame
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API FixedFrame : public Frame {
	OpenSim_DECLARE_CONCRETE_OBJECT(FixedFrame, Frame);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Body. **/
    /**@{**/
	
	OpenSim_DECLARE_PROPERTY(translation, SimTK::Vec3, 
		"Translation from the parent frame's origin to this frame's origin," 
		"expressed in the parent frame.");
	OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
		"orientation of this frame in it's parent frame, expressed as a "
		"body-fixed x-y-z rotation sequence.");
	
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
	FixedFrame();

	/** Convenience constructor */	
	FixedFrame(const Frame& parent_frame);
	

	// use compiler generated destructor, copy constructor and assignment operator

	/** Access Properties of the Body */

	/** Spatial Operations for Frames*/
	virtual const SimTK::Transform& getTransform() const;
	void setTransform(const SimTK::Transform& xform);
	virtual const SimTK::Transform calcTransformToGround(const SimTK::State &state) const;
	virtual const SimTK::Transform calcTransformFromGround(const SimTK::State &state) const;
	/** Get and set the parent reference frame*/
	void setParentFrame(const Frame& frame);
	const Frame& getParentFrame() const;

protected:
    // Model component interface.
	void constructStructuralConnectors() override;

	


private:

	void setNull();
	void constructProperties() override;
	mutable SimTK::Transform transform; // made mutable since it's used only for caching, public const methods can still modify it.


//=============================================================================
};	// END of class FixedFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FIXEDFRAME_H_


