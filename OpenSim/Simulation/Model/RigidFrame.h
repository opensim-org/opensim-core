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
* A RigidFrame is fixed to a Body. That is, its transform to this body is
* constant. A RigidFrame can be a Body itself. The body to which this frame is
* attached can be obtained via getBody(). Thus, it is guaranteed that any
* objects fixed to a rigid frame are also fixed to some Body in the system.
*
* @author Matt DeMers
*/
class OSIMSIMULATION_API RigidFrame : public Frame {
	OpenSim_DECLARE_ABSTRACT_OBJECT(RigidFrame, Frame);
public:
	//==========================================================================
	// PROPERTIES
	//==========================================================================
	/** @name Property declarations
	These are the serializable properties associated with a RigidFrame. **/
	/**@{**/
	/**@}**/
protected:



	//==========================================================================
	// PUBLIC METHODS
	//==========================================================================

public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
    RigidFrame();

    virtual ~RigidFrame() {};
	/** Test if this RigidFrame is anchored to a Body.
	 */
	bool isAnchoredToBody() const;
    /** 
	 * If RigidFrame is anchored to a Body, either directly or through,
	 * intermediate Frames, return a reference to that Body. Check isAnchoredToBody()
	 * before calling. Throws an exception if this RigidFrame is not connected to a body.
     */
	const OpenSim::Body& getAnchorBody() const;

	/**
	 * All RigidFrames are ultimately anchored to a SimTK::Mobilized body.  
	 * Return the MobilizedBodyIndex of the MobilizedBody to which this RigidFrame
	 * is anchored.
	 */
	const SimTK::MobilizedBodyIndex getMobilizedBodyIndex() const { return _index; }

private:
	void setNull();
	
protected:
	/* ID for the underlying mobilized body in Simbody system.
	Only Joint can set, since it defines the mobilized body type and
	the connection to the parent body in the multibody tree. */
	mutable SimTK::MobilizedBodyIndex _index;
	/* Smart pointer to the Body to which this RigidFrame is connected.
	Will be null if not connected to a body*/
	mutable SimTK::ReferencePtr<OpenSim::Body> _body;
	//==========================================================================
};	// END of class RigidFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_RIGIDFRAME_H_


