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
 * Copyright (c) 2005-2015 Stanford University and the Authors                
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
 * extension of the parent Frame type so that an OffsetFrame<PhysicalFrame>,
 * for example, can be treated as a PhysicalFrame. This enables Frames to be
 * filtered by their type (e.g. Physical or not), regardless of whether or
 * not the Frame is also an OffsetFrame. (A class whose super class is a
 * template parameter is called a mixin class.)
 *
 * OffsetFrames also have the property that if they form a chain or a tree,
 * each OffsetFrame shares the same Base which is the parent of the first/root
 * OffsetFrame in the tree. This allows Solvers and algorithms to work directly
 * with the Base which can be more efficient.
 *
 * OffsetFrame is an abstract class. Derive concrete subclasses in order to
 * accommodate new Frame types that require their offsets to retain the same
 * type as the parent. For example:
 * @code class PhysicalOffsetFrame : public OffsetFrame<PhysicalFrame> @endcode
 *
 * @see PhysicalOffsetFrame.
 *
 * @tparam C The type of the parent frame, as well as the super class. Must be
 * of type Frame.
 *
 * @author Matt DeMers
 * @author Ajay Seth
 */
template <class C = Frame>
class OffsetFrame : public C {
    OpenSim_DECLARE_ABSTRACT_OBJECT_T(OffsetFrame, C, C);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(translation, SimTK::Vec3, 
    "Translational offset of this frame's origin from the parent frame's origin, " 
    "expressed in the parent frame.");
    OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
    "Orientation offset of this frame in its parent frame, expressed as a "
    "frame-fixed x-y-z rotation sequence.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** By default, the frame is not connected to any parent frame, and its
    transform is an identity transform.
    */
    OffsetFrame();

    /** A convenience constructor that initializes the parent connection and
    offset property of this OffsetFrame.
     
    @param[in] parent   The parent reference frame.
    @param[in] offset   The offset transform between this frame and its parent
    */
    OffsetFrame(const C& parent, const SimTK::Transform& offset);

    /**  A convenience constructor that initializes the name of the OffsetFrame,
         the parent connection and its offset property.
    @param[in] name     The name of this OffsetFrame.
    @param[in] parent   The parent reference frame.
    @param[in] offset   The offset transform between this frame and its parent
    */
    OffsetFrame(const std::string& name, 
                const C& parent, const SimTK::Transform& offset);

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
    
    @return offset  The transform between this frame and its parent frame.
    */
    const SimTK::Transform& getOffsetTransform() const;

    /**
    Sets the transform the translates and rotates this frame (F frame) from 
    its parent frame (P frame). You should provide the transform X_PF
    such that vec_P = X_PF*vec_F.
    
    This transform is stored via the translation and orientation
    properties of this object.
    
    @param offset   The transform between this frame and its parent frame.
    */
    void setOffsetTransform(const SimTK::Transform& offset);


    /** Scale the offset given scale factors for spatial (XYZ) dimensions */
    void scale(const SimTK::Vec3& scaleFactors);

protected:
    /** The transform X_GF for this OffsetFrame, F, in ground, G.*/
    SimTK::Transform
        calcGroundTransform(const SimTK::State& state) const override;

    /** Extend how OffsetFrame determines its base Frame. */
    const Frame& extendFindBaseFrame() const override final;
    /** Extend how OffsetFrame determines its transform in its base Frame. */
    SimTK::Transform extendFindTransformInBaseFrame() const override final;

    /** @name Component Interface
    These methods adhere to the Component Interface**/
    /**@{**/
    void constructConnectors() override;
    void extendFinalizeFromProperties() override;
    /**@}**/

private:

    void setNull();
    void constructProperties() override;

    
    // the transform on my parent frame
    SimTK::Transform _offsetTransform;
//=============================================================================
}; // END of class OffsetFrame
//=============================================================================

//=============================================================================
// Implementation of OffsetFrame<C> template methods
//=============================================================================
// Default constructor
template <class C>
OffsetFrame<C>::OffsetFrame() : C()
{
    setNull();
    this->constructInfrastructure();
}

// Convenience constructors
template <class C>
OffsetFrame<C>::OffsetFrame(const C& parent,
    const SimTK::Transform& offset) : C()
{
    setNull();
    this->constructInfrastructure();
    setParentFrame(parent);
    setOffsetTransform(offset);
}

template <class C>
OffsetFrame<C>::OffsetFrame(const std::string& name, 
        const C& parent, const SimTK::Transform& offset)
    : OffsetFrame(parent, offset)
{
    this->setName(name);
}

// Set a null frame as Identity rotation, 0 translation
template <class C>
void OffsetFrame<C>::setNull()
{
    _offsetTransform.setToNaN();
    this->setAuthors("Matt DeMers, Ajay Seth");
}
template <class C>
void OffsetFrame<C>::constructProperties()
{
    SimTK::Vec3 zero(0.0, 0.0, 0.0);
    constructProperty_translation(zero);
    constructProperty_orientation(zero);
    // transform at default
}

template <class C>
void OffsetFrame<C>::constructConnectors()
{
    this->template constructConnector<C>("parent");
}

//=============================================================================
// FRAME COMPUTATIONS
//=============================================================================
// Implementation of Frame interface by OffsetFrame.
template <class C>
SimTK::Transform OffsetFrame<C>::
calcGroundTransform(const SimTK::State& s) const
{
    return getParentFrame().getGroundTransform(s)*getOffsetTransform();
}

//=============================================================================
// GET AND SET
//=============================================================================
template <class C>
void OffsetFrame<C>::setParentFrame(const C& parent)
{
    this->template updConnector<C>("parent").connect(parent);
}

template <class C>
const C& OffsetFrame<C>::getParentFrame() const
{
    return this->template getConnector<C>("parent").getConnectee();
}

template <class C>
const SimTK::Transform& OffsetFrame<C>::getOffsetTransform() const
{
    return _offsetTransform;
}

template <class C>
void OffsetFrame<C>::setOffsetTransform(const SimTK::Transform& xform)
{
    _offsetTransform = xform;
    // Make sure properties are updated in case we either call getters or
    // serialize after this call
    set_translation(xform.p());
    set_orientation(xform.R().convertRotationToBodyFixedXYZ());
}

template<class C>
inline void OffsetFrame<C>::scale(const SimTK::Vec3 & scaleFactors)
{
    upd_translation() = get_translation().elementwiseMultiply(scaleFactors);
}

template<class C>
const Frame& OffsetFrame<C>::extendFindBaseFrame() const
{
    // Offset defers finding the base frame to its parent
    // since it is never a base frame itself.
    return getParentFrame().findBaseFrame();
}

template<class C>
SimTK::Transform OffsetFrame<C>::extendFindTransformInBaseFrame() const
{
    // transform is always an offset on the parent's transform
    return getParentFrame().findTransformInBaseFrame() * getOffsetTransform();
}

template<class C>
void OffsetFrame<C>::extendFinalizeFromProperties()
{
    Super::extendFinalizeFromProperties();
    _offsetTransform.updP() = get_translation();
    _offsetTransform.updR().setRotationToBodyFixedXYZ(get_orientation());
}


} // end of namespace OpenSim

#endif // OPENSIM_OFFSET_FRAME_H_


