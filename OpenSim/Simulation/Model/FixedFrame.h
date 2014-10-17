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
 * A FixedFrame is a RigidFrame whose position and orientation is given with respect
 * to another RigidFrame. Additionally, the transform to the parent RigidFrame
 * is constant.
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
		"Position of this frame's origin from the parent frame's origin, " 
		"expressed in the parent frame.");
	OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
		"Orientation of this frame in its parent frame, expressed as a "
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
    /** By default, the frame is not connected to any parent frame,
     * and its transform is an identity transform.
     */
	FixedFrame();

    virtual ~FixedFrame() {};
    /** A convenience constructor that initializes the connections and
     * properties of this object.
     *
     * @param[in] parent_frame the parent reference frame.
     */
	FixedFrame(const RigidFrame& parent_frame);

    /** A convenience constructor that initializes the connections and
     * properties of this object.
     *
     * @param[in] parent_frame the parent reference frame.
     * @param[in] transform The transform between this frame and its parent
     *                      frame.
     */
    FixedFrame(const RigidFrame& parent_frame, const SimTK::Transform&
            transform);
	
	/** Set the parent reference frame*/
	void setParentFrame(const RigidFrame& frame);
	/** Get the parent reference frame*/
	const RigidFrame& getParentFrame() const;
	/**
	* Get the transform that describes the translation and rotation of this
	* frame (F frame) relative to its parent frame (P frame).  This method
	* returns the transform converting quantities expressed in F frame to
	* quantities expressed in the P frame. This is mathematically stated as,
	* vec_P = X_PF*vec_F ,
	* where X_PF is the transform returned by getTransform.
    *
    * This transform is computed using the translation and orientation
    * properties of this object.
	*
	* @return transform  The transform between this frame and its parent frame.
	*/
	const SimTK::Transform& getTransform() const;
	/** Set the transform the translates and rotates this frame (F frame) from 
	* its parent frame (P frame). You should provide the transform X_PF
	* such that vec_P = X_PF*vec_F.
    *
    * This transform is stored via the translation and orientation
    * properties of this object.
	*
	* @param transform  The transform between this frame and its parent frame.
	*/
	void setTransform(const SimTK::Transform& transform);
	
protected:
    
	
    /** @name Model Component Interface
    These methods adhere to the Model Component Interface**/
    /**@{**/
    
	void addToSystem(SimTK::MultibodySystem& system) const override;
	void constructConnectors() override;
    /**@}**/

    /** @name Utilities
    Internal helpers used by FixedFrames for initializing themselves.**/
    /**@{**/
    
	/** this helper function is called during buildSytem, after all connnection are resolved. **/
	void initFixedFrameCache() const;
    /** call this helper function whenever there is a structural change
    **  meaning when the transform or parent frame is updated. */
    void invalidate() const;
    /** Check the validity of the tree of RigidFrames connecting this to a base segment. */
    bool isPathToBaseValid() const;
	mutable bool _isCacheInitialized;
    /**@}**/
private:

	void setNull();
	void constructProperties() override;

    // made mutable since it's used only for caching, public const methods can
    // still modify it.
    
    // a private member to cache the tranform on my parent frame
	mutable SimTK::Transform _transform;

//=============================================================================
};	// END of class FixedFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FIXEDFRAME_H_


