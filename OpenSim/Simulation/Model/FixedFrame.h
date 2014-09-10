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
 * Author(s): Matt DeMers Ajay Seth, Ayman Habib                              *
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
#include <OpenSim/Simulation/Model/RigidFrame.h>


namespace OpenSim {

class Model;
class Body;

//=============================================================================
//=============================================================================
/**
 * A FixedFrame is a Frame based off another Frame (parent frame) that could be either 
 * another FixedFrame or a Body (which is a Frame).
 * 
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API FixedFrame : public RigidFrame {
	OpenSim_DECLARE_CONCRETE_OBJECT(FixedFrame, RigidFrame);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a FixedFrame. **/
    /**@{**/
	
	OpenSim_DECLARE_PROPERTY(translation, SimTK::Vec3, 
		"Translation from the parent frame's origin to this frame's origin, " 
		"expressed in the parent frame.");
	OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
		"orientation of this frame in its parent frame, expressed as a "
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

	/** Convenience constructors */	
	FixedFrame(const RigidFrame& parent_frame);
	FixedFrame(const RigidFrame& parent_frame, const SimTK::Transform& xform);
	
	/** Set the parent reference frame*/
	void setParentFrame(const RigidFrame& frame);
	/** Get the parent reference frame*/
	const RigidFrame& getParentFrame() const;
	/**
	* Get the transform that describes the translation and rotation of this
	* frame (F frame) relative to its parent frame (P frame).  This method
	* returns the transform converting quantities expressed in F frame to
	* quantities expressed in the P frame. This is mathematically stated as,
	* vec_P = P_X_F*vec_F ,
	* where P_X_F is the transform returned by getTransform.
	*
	* @param state       The state applied to the model when determining the
	*                    transform.
	* @return transform  The transform between this frame and its parent frame
	*/
	const SimTK::Transform& getTransform() const;
	/** Set the transform the translates and rotates this frame (F frame) from 
	* its parent frame (P frame). You should provide the transform P_x_F
	* such that vec_P = P_X_F*vec_F
	*
	* @param transform  The transform between this frame and its parent frame
	*/
	void setTransform(const SimTK::Transform& xform);
	
    /** Return reference to the Body that Frame is affixed to, either directly or through 
    * intermediate Frames
    */
    const OpenSim::Body& getBody() const;
	
protected:
    // Model component interface.
	void constructStructuralConnectors() override;
	// Frame interface
	SimTK::Transform calcGroundTransform(const SimTK::State& state) const override;
	


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


