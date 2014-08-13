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

	/** Spatial Operations for Frames*/
	virtual const SimTK::Transform& getTransform() const = 0;
    virtual const SimTK::Transform calcGroundTransform(const SimTK::State& state) const = 0;
	const SimTK::Transform calcTransformToOtherFrame(const SimTK::State& state, const Frame& frame) const;
	const SimTK::Vec3 expressVectorInAnotherFrame(const SimTK::State& state, const SimTK::Vec3& vec, const Frame& frame) const;
	const SimTK::Vec3 expressPointInAnotherFrame(const SimTK::State& state, const SimTK::Vec3& point, const Frame& frame) const;

	
private:
	void setNull();

protected:
	static const SimTK::Transform identityTransform;

//=============================================================================
};	// END of class Body
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FRAME_H_


