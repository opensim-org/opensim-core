#ifndef OPENSIM_PHYSICAL_FRAME_H_
#define OPENSIM_PHYSICAL_FRAME_H_
/* -------------------------------------------------------------------------- *
*                              OpenSim:  PhysicalFrame.h                             *
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
#include <OpenSim/Simulation/Model/Frame.h>
namespace OpenSim {

class Model;
class Body;
//=============================================================================
//=============================================================================
/**
* A PhysicalFrame is a frame that represents a phyisical location at which 
* Joints and Constraints can be connected and Forces applied. A Body is an
* example of PhysicalFrame and so is Ground.
*
* @author Matt DeMers
*/

class OSIMSIMULATION_API PhysicalFrame : public Frame {
    OpenSim_DECLARE_CONCRETE_OBJECT(PhysicalFrame, Frame);

    //==========================================================================
    // PUBLIC METHODS
    //==========================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    PhysicalFrame();

    virtual ~PhysicalFrame() {};

    /**
     * All PhysicalFrames are backed by a SimTK::MobilizedBody, which is the
     * fundamental rigid element of a Simbody MultibodySystem. This method 
     * returns the MobilizedBodyIndex of the MobilizedBody for this PhysicalFrame.
     * This index is only available after Model::initSystem() has been invoked.
     *
     * The MobilizedBodyIndex is necessary to access individual PhysicalFrame
     * forces from the underlying MultibodySystem's body forces since the Vector
     * of net body forces (torque and force on each body) is indexed by its 
     * MobilizedBodyIndex.
     *
     * @return index The MobilizedBodyIndex corresponding to this PhysicalFrame's
     *               underlying MobilizedBody
     *
     * @see getMobilizedBody, updMobilizedBody
     */
    const SimTK::MobilizedBodyIndex& getMobilizedBodyIndex() const {
        return _mbIndex;
    }

    /**
     * Access the SimTK::MobilizedBody that backs this PhysicalFrame. The
     * MobilizedBody is only available after Model::initSystem() has been
     * invoked.
     *
     * @see getMobilizedBodyIndex
     */
    const SimTK::MobilizedBody& getMobilizedBody() const;

    /**
     * Access a writeable SimTK::MobilizedBody that backs this PhysicalFrame.
     * The MobilizedBody is only available after Model::initSystem() has been
     * invoked.
     *
     * @see getMobilizedBodyIndex
     */
    SimTK::MobilizedBody& updMobilizedBody();

protected:
    /** @name PhysicalFrame devloper interface
    These methods are for PhysicalFrame builders. **/
    /**@{**/
    /** Specify the MobilizedBody that implements this PhysicalFrame 
        in the underlying MultibodySystem. */
    void setMobilizedBodyIndex(const SimTK::MobilizedBodyIndex& mbix) const {
        _mbIndex = mbix; 
    }

    /** Extend how concrete Frame determines its base Frame. */
    const Frame& extendFindBaseFrame() const override {
        return *this;
    }

    SimTK::Transform extendFindTransformInBaseFrame() const override;

    /**@}**/

    /** Implement the Frame interface and return the transform X_GF for this
    PhysicalFrame, F, in ground, G.*/
    const SimTK::Transform&
        calcGroundTransform(const SimTK::State& state) const override;

    /** Extend Component */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;


private:
    /* ID for the underlying mobilized body in Simbody system.
    Only Joint can set, since it defines the mobilized body type and
    the connection to the parent body in the multibody tree. */
    mutable SimTK::MobilizedBodyIndex _mbIndex;

    //==========================================================================
};  // END of class PhysicalFrame

//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PHYSICAL_FRAME_H_


