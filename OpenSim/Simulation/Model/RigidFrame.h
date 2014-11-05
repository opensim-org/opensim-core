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
* A RigidFrame is fixed to body in the articulating, multibody system. That is,
* its transform to this body is constant. A RigidFrame can be a Body itself. 
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

	/**
     * All RigidFrames are ultimately rooted to a SimTK::MobilizedBody.  Return
     * the MobilizedBodyIndex of the MobilizedBody to which this RigidFrame is
     * rooted. This index is only available after Model::initSystem() is
     * invoked.
     *
     * @return index The MobilizedBodyIndex corresponding to this RigidFrame's
     * MobilizedBody
     *
     * @see getMobilizedBody, updMobilizedBody
	 */
	SimTK::MobilizedBodyIndex getMobilizedBodyIndex() const { return _index; }

    /**
     * The SimTK::MobilizedBody to which this RigidFrame is rooted. This
     * MobilizedBody is only available after Model::initSystem() has been
     * invoked.
     *
     * @see getMobilizedBodyIndex
     */
    const SimTK::MobilizedBody& getMobilizedBody() const;

    /**
     * The SimTK::MobilizedBody to which this RigidFrame is rooted. This
     * MobilizedBody is only available after Model::initSystem() has been
     * invoked.
     *
     * @see getMobilizedBodyIndex
     */
    SimTK::MobilizedBody& updMobilizedBody();

    /**
     * Get the fixed transform describing this RigidFrame's transform in its root MobilizedBody.
     *
     * @return Transform  The transform between this frame and its root RigidFrame.
    */
    SimTK::Transform getTransformInMobilizedBody() const { return _mbTransform; }
private:
	void setNull();
	
protected:
	/* ID for the underlying mobilized body in Simbody system.
	Only Joint can set, since it defines the mobilized body type and
	the connection to the parent body in the multibody tree. */
	mutable SimTK::MobilizedBodyIndex _index;
    /* RigidFrames, by definition have a fixed transform in their root MobilizedBody.
    This stores the fixed transform of this RigidFrame in it's root MobilizedBody*/
    mutable SimTK::Transform _mbTransform;
    /** @name Frame Interface
    These methods adhere to the Frame Interface**/
    /**@{**/
    SimTK::Transform calcGroundTransform(const SimTK::State& state) const override;
    /**@}**/
	//==========================================================================
};	// END of class RigidFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_RIGIDFRAME_H_


