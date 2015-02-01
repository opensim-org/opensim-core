#ifndef OPENSIM_PHYSICAL_FRAME_H_
#define OPENSIM_PHYSICAL_FRAME_H_
/* -------------------------------------------------------------------------- *
*                              OpenSim:  PhysicalFrame.h                        
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US   
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                      
*                                                                       
* Copyright (c) 2005-2015 Stanford University and the Authors           
* Author(s): Matt DeMers, Ajay Seth, Ayman Habib                        
*                                                                       
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.    
*                                                                       
* Unless required by applicable law or agreed to in writing, software   
* distributed under the License is distributed on an "AS IS" BASIS,     
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
* See the License for the specific language governing permissions and   
* limitations under the License.                                        
* -------------------------------------------------------------------------- */

// INCLUDE
#include <OpenSim/Simulation/Model/Frame.h>
namespace OpenSim {

class Model;
class Body;
//=============================================================================
//=============================================================================
/**
* A PhysicalFrame is a Frame that locates a physical element of the multi-
* body system that underlies a Model. A PhysicalFrame supports physical 
* connections (e.g. Joints, Constraints) and is the Frame type upon which 
* forces can be applied. A concrete example of a PhysicalFrame is a Body.
* Attributes of a Body (its center-of-mass, geometry, ...) are located in the
* Body frame. Bodies are connected by Joints and Constraints and Forces are 
* readily applied to them. A location that represents an offset from the Body
* frame, can also a PhysicalFrame (e.g. a PhysicalOffsetFrame).
*
* @see PhysicalOffsetFrame
*
* @author Matt DeMers
* @author Ajay Seth
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

    virtual ~PhysicalFrame() {}

    /** @name Advanced: PhysicalFrame's SimTK::MobilizedBody Access
    Although these methods are public, they are intended for advanced users and
    developers to access System resources associated with the MobilizedBody that
    underlies the PhysicalFrame. For example, Solvers operate on the System and
    not on the OpenSim modeling abstractions, such as Frames. 
    All PhysicalFrames are backed by a SimTK::MobilizedBody, which is the
    fundamental rigid element of a Simbody SimTK::MultibodySystem. */

    ///@{
    /**
    This method returns the MobilizedBodyIndex of the MobilizedBody for this
    PhysicalFrame. This index is only available after Model::initSystem() has
    been invoked.

    The MobilizedBodyIndex is necessary to access the underlying MobilizedBody
    in the System. It allows access to physical quantities (e.g. forces)
    associated with invidual PhysicalFrames. For examples, the underlying
    MultibodySystem's net body forces are represented as a Vector of spatial
    forces (torque and force on each body) and it is indexed by the 
    MobilizedBodyIndex.

    @return index The MobilizedBodyIndex corresponding to this PhysicalFrame's
               underlying MobilizedBody

    @see getMobilizedBody, updMobilizedBody
    */
    const SimTK::MobilizedBodyIndex& getMobilizedBodyIndex() const {
        return _mbIndex;
    }

    /**
    Access a readable SimTK::MobilizedBody that backs this PhysicalFrame.
    The MobilizedBody is only available after Model::initSystem() has been
    invoked.
    @see getMobilizedBodyIndex
    */
    const SimTK::MobilizedBody& getMobilizedBody() const;

    /**
    Access a writeable SimTK::MobilizedBody that backs this PhysicalFrame.
    The MobilizedBody is only available after Model::initSystem() has been
    invoked.
    @see getMobilizedBodyIndex
    */
    SimTK::MobilizedBody& updMobilizedBody();

    // End of underlying MobilizedBody accessors.
    ///@} 

protected:
    /** @name Advanced: PhysicalFrame Devloper Interface
    These methods are intended for PhysicalFrame builders. */
    ///@{
    /**
    Specify the MobilizedBody that implements this PhysicalFrame in the
    underlying MultibodySystem. The PhysicalFrame's MobilizedBodyIndex must be
    set by the end of PhysicalFrame::addToSystem()
        */
    void setMobilizedBodyIndex(const SimTK::MobilizedBodyIndex& mbix) const {
        _mbIndex = mbix; 
    }

    /** Extend how PhysicalFrame determines its base Frame. */
    const Frame& extendFindBaseFrame() const override {
        return *this;
    }

    /** Extend how PhysicalFrame determines its Transform in the base Frame. */
    SimTK::Transform extendFindTransformInBaseFrame() const override;

    ///@}

    /** The transform X_GF for this PhysicalFrame, F, in ground, G. */
    SimTK::Transform
        calcGroundTransform(const SimTK::State& state) const override;

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


