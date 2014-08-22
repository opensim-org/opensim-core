#ifndef OPENSIM_FRAME_H_
#define OPENSIM_FRAME_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Frame.h                             *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>



namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A Frame is a modeling abstraction that defines a Righthanded CoordinateSystem
 * to be used for attaching and/or expressing model quantities/objects in. 
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API Frame : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Frame, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a Frame. **/
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
	Frame();

	/** @name Spatial Operations for Frames
	These methods allow access to the frame's transfrom and add some convenience
	To do math with the transform.*/
	/**@{**/

	/**
	* Get the transform that describes the translation and rotation of this
	* frame (F frame) relative to the ground frame (G frame).  This method 
	* returns the transform converting quantities expressed in F frame to 
	* quantities expressed in the G frame. This is mathematically stated as,
	* vec_G = G_X_F*vec_F ,
	* where G_X_F is the transform returned by getGroundTransform.
	* 
	* @param state       The state applied to the model when determining the 
	*                    transform.
	* @return transform  The transform between this frame and the ground frame
	*/
	SimTK::Transform getGroundTransform(const SimTK::State& state) const {
		return calcGroundTransform(state);
	}

	/**
	* Get the transform that describes the translation and rotation of this
	* frame (F frame) relative to another frame (A frame).  This method returns
	* the transform converting quantities expressed in F frame to quantities
	* expressed in the A frame. This is mathematically stated as,
	* vec_A = A_X_F*vec_F ,
	* where A_X_F is the transform returned by getGroundTransform.
	* 
	* @param state       The state applied to the model when determining the 
	*                    transform.
	* @param otherFrame  a second frame
	* @return transform  The transform between this frame and otherFrame
	*/
	
	SimTK::Transform calcTransformToOtherFrame(const SimTK::State& state, const Frame& otherFrame) const;
	/**
	* Take a vector expressed in this frame (F frame) as the same vector
	* expressed in another frame (A frame).  This re-expression accounts
	* for the difference in orientation between the frames. In mathematical
	* form, this method returns vec_A, where vec_A = A_R_F*vec.  THIS METHOD
	* DOES NOT PERFORM A HOMOGENOUS TRANSFORM, thus is does not add 
	* translations to the vector.
	*
	* @param state       The state applied to the model when determining the
	*                    transform.
	* @param vec		 The vector to be re-expressed.
	* @param otherFrame  The frame in which the vector will be re-expressed
	* @return newVec     The expression of the vector in otherFrame.
	* */
	SimTK::Vec3 expressVectorInAnotherFrame(const SimTK::State& state, const SimTK::Vec3& vec, const Frame& otherFrame) const;
	/**
	* Take a point located and expressed in this frame (F frame) and determine
	* its location expressed in another frame (A frame) using the homogeneous 
	* transformation. This transformation accounts for the difference in 
	* orientation and translation between the frames. In mathematical form, 
	* this method returns point_A, where point_A = A_T_F*point.
	*
	* @param state       The state applied to the model when determining the
	*                    transform.
	* @param vec		 The point to be re-expressed.
	* @param otherFrame  The frame in which the point will be re-expressed
	* @return newVec     The expression of the point measured in otherFrame.
	*/
	SimTK::Vec3 expressPointInAnotherFrame(const SimTK::State& state, const SimTK::Vec3& point, const Frame& otherFrame) const;
	/**@}**/
	
private:
	void setNull();

protected:
	/** @name Utility methods
	These methods just help with frame computations*/
	/**@{**/
	virtual SimTK::Transform calcGroundTransform(const SimTK::State& state) const = 0;
	/**@}**/

//=============================================================================
};	// END of class Frame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FRAME_H_


