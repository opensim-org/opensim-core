#ifndef OPENSIM_OFFSET_FRAME_H_
#define OPENSIM_OFFSET_FRAME_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  OffsetFrame.h                           
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    
 * through the Warrior Web program.                                           
 *                                                                            
 * Copyright (c) 2005-2013 Stanford University and the Authors                
 * Author(s): Matt DeMers Ajay Seth, Ayman Habib                              
 *                                                                            
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * An OffsetFrame is a Frame whose transform (translation and orientation)
 * with respect to another (parent) Frame is constant in time. It acts as an
 * extension of the parent Frame type so that an OffsetFrame<Body>, for
 * example, can be treated identically to a Body. This enables Frames to be
 * filtered by their type (e.g. PhyscialFrame or not), regardless of whether or
 * not the Frame is also an OffsetFrame. (A class whose super class is a
 * template parameter is called a mixin class.)
 *
 * OffsetFrames also have the property that if they form a chain or a tree,
 * each OffsetFrame shares the same Base which is the parent of the first/root
 * OffsetFrame in the tree. This allows Solvers and algorithms to work directly
 * with the Base which can be more efficient.
 *
 * The only OffsetFrame we currently expect users to use is
 * OpenSim::PhysicalOffsetFrame.
 *
 * @tparam C The type of the parent frame, as well as the super class. Must be
 * of type Frame.
 *
 * @author Matt DeMers
 * @author Ajay Seth
 */
template <class C = Frame>
class OSIMSIMULATION_API OffsetFrame : public C {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(OffsetFrame, C, C);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations 
    These are the serializable properties associated with a OffsetFrame. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(translation, SimTK::Vec3, 
    "Translational offset of this frame's origin from the parent frame's origin, " 
    "expressed in the parent frame.");
    OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
    "Orientation offset of this frame in its parent frame, expressed as a "
    "frame-fixed x-y-z rotation sequence.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /**
    By default, the frame is not connected to any parent frame, and its
    transform is an identity transform.
    */
    OffsetFrame();

    /**
    A convenience constructor that initializes the connections and
    properties of this object.
     
    @param[in] parent   The parent reference frame.
    @param[in] offset The offset transform between this frame and its parent
    */
    OffsetFrame(const C& parent, const SimTK::Transform& offset);

    // use compiler generated destructor, copy constructor and assignment operator

    /** Sets the parent reference frame*/
    void setParentFrame(const C& parent);
    /** Get the parent reference frame*/
    const C& getParentFrame() const;
    /**
    Get the transform that describes the translational and rotational offset
    of this frame (F frame) relative to its parent frame (B frame).  This method
    returns the transform converting quantities expressed in F frame to
    quantities expressed in the B frame. This is mathematically stated as,
    vec_P = X_BF*vec_F ,
    where X_BF is the transform returned by getTransform.
    
    This transform is computed using the translation and orientation
    properties of this object.
    
    @return transform  The transform between this frame and its parent frame.
    */
    const SimTK::Transform& getOffsetTransform() const;

    /**
    Sets the transform the translates and rotates this frame (F frame) from 
    its parent frame (P frame). You should provide the transform X_PF
    such that vec_P = X_PF*vec_F.
    
    This transform is stored via the translation and orientation
    properties of this object.
    
    @param transform  The transform between this frame and its parent frame.
    */
    void setOffsetTransform(const SimTK::Transform& offsetTransform);

protected:
    /** The transform X_GF for this OffsetFrame, F, in ground, G.*/
    const SimTK::Transform&
        calcGroundTransform(const SimTK::State& state) const override;

    /** Extend how OffsetFrame determines its base Frame. */
    const Frame& extendFindBaseFrame() const final;
    SimTK::Transform extendFindTransformInBaseFrame() const final;

    /** @name Model Component Interface
    These methods adhere to the Model Component Interface**/
    /**@{**/
    void constructConnectors() override;
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /**@}**/

private:

    void setNull();
    void constructProperties() override;

    
    // the tranform on my parent frame
    SimTK::Transform _offsetTransform;

//=============================================================================
}; // END of class OffsetFrame
//=============================================================================
//=============================================================================

/** A PhysicalFrame whose location and orientation is specified as a constant
 * offset from another PhysicalFrame. One potential use case for a
 * PhysicalOffsetFrame is to specify the location of a Joint; e.g. the location
 * and orientation of the ankle joint frame as specified in the knee Body
 * frame. This class has the methods of both the OffsetFrame template class
 * and the PhysicalFrame class.
 */
typedef OffsetFrame<PhysicalFrame> PhysicalOffsetFrame;

} // end of namespace OpenSim

#endif // OPENSIM_OFFSET_FRAME_H_


