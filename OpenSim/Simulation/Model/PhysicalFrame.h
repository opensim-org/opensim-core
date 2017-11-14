#ifndef OPENSIM_PHYSICAL_FRAME_H_
#define OPENSIM_PHYSICAL_FRAME_H_
/* --------------------------------------------------------------------------*
*                              OpenSim:  PhysicalFrame.h                     *   
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "simbody/internal/Body.h"

// TODO remove these when corresponding properties are removed
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

namespace SimTK {
class MobilizedBody;
}

namespace OpenSim {


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
* frame, can also be a PhysicalFrame (e.g. a PhysicalOffsetFrame).
*
* @see PhysicalOffsetFrame
*
* @author Matt DeMers
* @author Ajay Seth
*/

class OSIMSIMULATION_API PhysicalFrame : public Frame {
    OpenSim_DECLARE_ABSTRACT_OBJECT(PhysicalFrame, Frame);

public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /* TODO: Both VisibleObject and the WrapObjectSet should NOT be properties
    of the PhysicalFrame. This is an intermediate solution as we integrate Frames 
    use into the OpenSim API. These properties should be their own components with
    Sockets to the PhysicalFrames they attach to. - aseth

    Note: VisibleObject was removed from this class by @aymanhab via PR #417.
                                                            -@chrisdembia
    */
    OpenSim_DECLARE_UNNAMED_PROPERTY(WrapObjectSet,
        "Set of wrap objects fixed to this body that GeometryPaths can wrap over."
        "This property used to be a member of Body but was moved up with "
        "the introduction of Frames.");

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
    associated with individual PhysicalFrames. For examples, the underlying
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
    Access a writable SimTK::MobilizedBody that backs this PhysicalFrame.
    The MobilizedBody is only available after Model::initSystem() has been
    invoked.
    @see getMobilizedBodyIndex
    */
    SimTK::MobilizedBody& updMobilizedBody();

    // End of underlying MobilizedBody accessors.
    ///@}

    /** @name DEPRECATED API */

    ///@{
    /** Deprecated methods for intermediate integration of Frames */
    /** Get the named wrap object, if it exists.
    *
    * @param aName Name of the wrap object.
    * @return const Pointer to the wrap object.
    */
    const WrapObject* getWrapObject(const std::string& aName) const;
    const WrapObjectSet& getWrapObjectSet() const { return get_WrapObjectSet(); }

    /** Add a wrap object to the Body. Note that the Body takes ownership of
    * the WrapObject.
    */
    void addWrapObject(WrapObject* wrapObject);
    ///@} 

protected:
    /** @name Advanced: PhysicalFrame Developer Interface
    These methods are intended for PhysicalFrame builders. */
    ///@{
    /**
    Specify the MobilizedBody that implements this PhysicalFrame in the
    underlying MultibodySystem. The PhysicalFrame's MobilizedBodyIndex must be
    set by the end of PhysicalFrame::addToSystem()
        */
    void setMobilizedBodyIndex(const SimTK::MobilizedBodyIndex& mbix) const;

    /** Extend how PhysicalFrame determines its base Frame. */
    const Frame& extendFindBaseFrame() const override {
        return *this;
    }

    /** Extend how PhysicalFrame determines its Transform in the base Frame. */
    SimTK::Transform extendFindTransformInBaseFrame() const override;
    ///@}

    /** @name Component Extension methods.
    PhysicalFrame extension of Component interface. */
    /**@{**/
    /// Associate a FrameGeometry (visualization) with the this PhysicalFrame
    void extendFinalizeFromProperties() override;
    /// Connect bound WrapObjects
    void extendConnectToModel(Model& model) override;
    /**@}**/

    /** Override to account for version updates in the XML format. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
        int versionNumber = -1) override;

private:
    /** The transform X_GF for this PhysicalFrame, F, in ground, G. */
    SimTK::Transform
        calcTransformInGround(const SimTK::State& state) const override;

    /** The spatial velocity {omega; v} for this PhysicalFrame in ground. */
    SimTK::SpatialVec
        calcVelocityInGround(const SimTK::State& state) const override;
    /** The spatial acceleration {alpha; a} for this PhysicalFrame in ground */
    SimTK::SpatialVec
        calcAccelerationInGround(const SimTK::State& state) const override;

    /* Component construction interface */
    void constructProperties();

    /* Utility to convert Geometry version 3.2 to recent 4.0 format */
    static void convertDisplayGeometryToGeometryXML(SimTK::Xml::Element& aNode,
        const SimTK::Vec3& outerScaleFactors,
        const SimTK::Vec6& outerTransform,
        SimTK::Xml::Element& geomSetElement);

    /* Utility for updating XML: add a PhysicalOffsetFrame to the XML element
    pointed to by ownerIter, and gives the frame the provided name, transform
    (from the frame's parent frame), and the path (connectee name) to the
    parent frame.*/
    static void createFrameForXform(const SimTK::Xml::element_iterator&,
        const std::string& frameName,
        const SimTK::Vec6& localXform, const std::string& parentConnecteeName);


    /* ID for the underlying mobilized body in Simbody system.
    Only Joint can set, since it defines the mobilized body type and
    the connection to the parent body in the multibody tree. */
    SimTK::ResetOnCopy<SimTK::MobilizedBodyIndex> _mbIndex;

    virtual const SimTK::Body& extractInternalRigidBody() const {
        return _internalRigidBody;
    }

    SimTK::Body::Massless _internalRigidBody;

    // Model is a friend because it creates the underlying mobilized body(ies)
    // that implement a Joint and is the only component that can assign
    // the MobilizedBodyIndex for this Body so it can communicate with its 
    // counter-part in the underlying system
    friend class Joint;

    //=========================================================================
};  // END of class PhysicalFrame

//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PHYSICAL_FRAME_H_


