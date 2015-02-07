#ifndef OPENSIM_JOINT_H_
#define OPENSIM_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Joint.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Peter Loan, Ajay Seth                        *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Common/Function.h>

namespace OpenSim {

class Model;
class Body;
class PhyscialFrame;
class ScaleSet;

/**
An OpenSim Joint is an OpenSim::ModelComponent which connects two Bodies
together and speficies their relative permissible motion as described in
internal coordinates. The base Joint specifies the two frames (on each body),
which the joint spans. The relative motion (including the # of coordinates)
are defined by concrete Joints, which specify the permissible kinematics of
a child joint frame (on a child body) with respect to a parent joint frame
(on a parent body). The designation of parent and child are used only to
identify the directionality of the joint and in which frame the joint
coordinates are expressed.

For example, A PinJoint between a parent, P, and a child body, B,
frames have a coordinate value of zero when the two frames are aligned
and positive coordinate values are the angle between the frames' X-axes given
a positive Z-rotation of the child frame about the coincident Z-axis in
the parent frame.

Concrete Joints can specify relative translations and even coupled
rotations and translations (see EllipsoidJoint and CustomJoint). For more
details on how the underlying formulation supports coupled curvilinear
joints, see "Minimal formulation of joint motion for biomechanisms", 2010
A Seth, M Sherman, P Eastman, S Delp; Nonlinear dynamics 62 (1), 291-303

<b>C++ example</b>
\code{.cpp}
    \\ Define a Pin joint between ground and platform
    PinJoint* platformToGround = new PinJoint("PlatformToGround", ground, locationInParent, orientationInParent, *platform, locationInChild, orientationInChild, false);
\endcode

<b>Python example</b>
\code{.py}
    ##Define a ball joint between blockA and blockB
    abJoint  = osim.BallJoint('JointName',blockA,locInParent,oriInParent,blockB,locInChild,oriInChild, 0)
\endcode

@author Ajay Seth
*/
class OSIMSIMULATION_API Joint : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Joint, ModelComponent);

public:

//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with a Joint. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(location_in_parent, SimTK::Vec3,
        "Location of the joint in the parent body specified in the parent "
        "reference frame. Default is (0,0,0).");

    OpenSim_DECLARE_PROPERTY(orientation_in_parent, SimTK::Vec3,
        "Orientation of the joint in the parent body specified in the parent "
        "reference frame. Euler XYZ body-fixed rotation angles (in radians) "
        "are used to express the orientation. Default is (0,0,0).");

    OpenSim_DECLARE_PROPERTY(location_in_child, SimTK::Vec3,
        "Location of the joint in the child body specified in the child "
        "reference frame. Default is (0,0,0).");

    OpenSim_DECLARE_PROPERTY(orientation_in_child, SimTK::Vec3,
        "Orientation of the joint in the child body specified in the child body "
        "reference frame. Euler XYZ body-fixed rotation angles (in radians) "
        "are used to express the orientation. Default is (0,0,0)" );

    OpenSim_DECLARE_UNNAMED_PROPERTY(CoordinateSet,
        "Set holding the generalized coordinates (q's) that parmeterize this joint." );

    OpenSim_DECLARE_PROPERTY(reverse, bool,
        "Advanced option. Specify the direction of the joint in the multibody tree: "
        "parent->child (forward, reverse == false) or child->parent (reverse == true) "
        "NOTE: the Joint transform and its coordinates maintain a parent->child "
        "sense, even if the Joint is reversed.");
    /**@}**/

//=============================================================================
// METHODS
//=============================================================================

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** DEFAULT CONSTRUCTION */
    Joint();

    /** Convenience Constructor */
    /** Create a Joint where the parent and body are specified as well as the
        joint frames in the child and parent bodies in terms of their location
        and orientation in their respective bodies. Also an advanced option to
        specify a tree structure to be constructed in the reverse direction,
        that is child to parent, but the coordinates remain as if defined parent
        to child. This can be useful for defining models from the ground up, yet
        maintaining the convention of the knee, for example, of the relative
        motion of the tibia (child) w.r.t. the femur (parent).

        @param[in] name     the name associated with this joint (should be
                            unique from other joints in the same model)
        @param[in] parent   the parent PhysicalFrame that joint connects to
        @param[in] locationInParent    Vec3 of the location of the joint in the
                                       parent body frame.
        @param[in] orientationInParent Vec3 of the XYZ body-fixed Euler angles
                                       of the joint frame orientation in the
                                       parent body frame.
        @param[in] child    the child PhysicalFrame that joint connects to
        @param[in] locationInChild     Vec3 of the location of the joint in the
                                       child body frame.
        @param[in] orientationInChild  Vec3 of the XYZ body-fixed Euler angles
                                       of the joint frame orientation in the
                                       child body frame.
        @param[in] reverse  Advanced optional flag (bool) specifying the 
                            direction of the Joint in the multibody tree. 
                            Default is false (that is, forward).
        */
    Joint(const std::string &name,
          const PhysicalFrame& parent,
          const SimTK::Vec3& locationInParent,
          const SimTK::Vec3& orientationInParent,
          const PhysicalFrame& child,
          const SimTK::Vec3& locationInChild,
          const SimTK::Vec3& orientationInChild,
          bool reverse = false);

    virtual ~Joint();

    // GET & SET

    void setChildFrameName(const std::string& name);
    const std::string& getChildFrameName() const;

    /**
     * Set the child body that this joint connects.
     *
     * @param child PhysicalFrame reference.
     */
    void setChildFrame(const PhysicalFrame& child);

    /**
     * Get the child joint frame.
     *
     * @return const PhysicalFrame reference.
     */
    const PhysicalFrame& getChildFrame() const;

    void setLocationInChild(const SimTK::Vec3& aLocation);
    const SimTK::Vec3& getLocationInChild() const;
    void setOrientationInChild(const SimTK::Vec3& aOrientation);
    const SimTK::Vec3& getOrientationInChild() const;

    // Relating to the parent joint frame
    void setParentFrameName(const std::string& aName);
    const std::string& getParentFrameName() const;

    /**
    * Set the parent frame that this joint connects.
    *
    * @param parent PhysicalFrame.
    */
    void setParentFrame(const PhysicalFrame& parent);
    /**
     * Get the parent frame to which this joint attaches.
     *
     * @return const ref to parent PhysicalFrame.
     */
    const OpenSim::PhysicalFrame& getParentFrame() const;

    void setLocationInParent(const SimTK::Vec3& aLocation);
    const SimTK::Vec3& getLocationInParent() const;
    void setOrientationInParent(const SimTK::Vec3& aOrientation);
    const SimTK::Vec3& getOrientationInParent() const;

    /** Get the Joint frames expressed as body transforms. Only available after
        extendConnectToModel() has been called on the Joint. */
    const SimTK::Transform& getParentTransform() const
        { return _jointFrameInParent; }
    const SimTK::Transform& getChildTransform() const
        { return _jointFrameInChild; }

    // Coordinate Set
    const CoordinateSet& getCoordinateSet() const {return get_CoordinateSet();}

    bool getReverse() const { return get_reverse(); }

    //Model building
    virtual int numCoordinates() const = 0;


    // Utility
    bool isCoordinateUsed(const Coordinate& aCoordinate) const;

    // Computation
    /** Given some system mobility (generalized) forces, calculate the 
    equivalent spatial body force for this Joint. Keep in mind that there are 
    typically nm < 6 mobilities per joint with an infinte set of solutions that 
    can map nm gen forces to 6 spatial force components (3 for torque + 3 for 
    force). The solution returned provides the "most" effective force and torque
    in the joint frame. This means the smallest magnituded force and/or torque 
    that will result in the same generalized force. If a generalized force is 
    defined along/about a joint axis, then this should be evident in the 
    reported results as a force or torque on the same axis.  NOTE: Joints 
    comprised of multiple mobilizers and/or constraints, should override this 
    method and account for multiple internal components.

    @param s containing the generalized coordinate and speed values 
    @param mobilityForces for the system as computed by inverse dynamics, 
                          for example 
    @return spatial force, FB_G, acting on the body connected by this joint at 
    its location B, expressed in ground.  */
    virtual SimTK::SpatialVec 
    calcEquivalentSpatialForce(const SimTK::State &s, 
                               const SimTK::Vector &mobilityForces) const;


    /** Joints in general do not contribute power since the reaction space 
        forces are orthogonal to the mobility space. However, when joint motion 
        is prescribed, the internal forces that move the joint will do work. In 
        that case, the power is non-zero and the supplied SimTK::State
        must already have been realized to %Acceleration stage so that 
        constraint forces are available. */
    virtual double calcPower(const SimTK::State &s) const;

    // SCALE
    /**
    * Scale a joint based on XYZ scale factors for the bodies.
    * Generic behavior is to scale the locations on parent and on the body
    * according to scale factors of the bodies upon which they are located.
    *
    * Joint subclasses should invoke this method before scaling joint specific
    * properties
    *
    * @param aScaleSet Set of XYZ scale factors for the bodies.
    */
    virtual void scale(const ScaleSet& aScaleSet);


protected:
    // build Joint transforms from properties
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    /** Construct coordinates according to the mobilities of the Joint */
    void constructCoordinates();

    // Methods that allow access for Joint subclasses to data members of
    // Body and Coordinate , which Joint befriends
    const SimTK::MobilizedBodyIndex getMobilizedBodyIndex(const Body& body) const;

    void setChildMobilizedBodyIndex(SimTK::MobilizedBodyIndex index) const;
    void setCoordinateMobilizedBodyIndex(Coordinate *aCoord, SimTK::MobilizedBodyIndex index) const {aCoord->_bodyIndex = index;}
    void setCoordinateMobilizerQIndex(Coordinate *aCoord, int index) const
        { aCoord->_mobilizerQIndex = SimTK::MobilizerQIndex(index);}
    void setCoordinateModel(Coordinate *aCoord, Model *aModel) const {aCoord->_model = aModel;}

    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber);

    /** Calculate the equivalent spatial force, FB_G, acting on a mobilized body specified by index
       acting at its mobilizer frame B, expressed in ground.  */
    SimTK::SpatialVec calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s, const SimTK::MobilizedBodyIndex mbx, const SimTK::Vector &mobilityForces) const;

    /** Return the equivalent (internal) SimTK::Rigid::Body for the parent/child
        OpenSim::Body. Not valid until after extendAddToSystem on the Body has been called.*/
    const SimTK::Body::Rigid& getParentInternalRigidBody() const;
    const SimTK::Body::Rigid& getChildInternalRigidBody() const;


    /** Utility method for creating the underlying MobilizedBody of the desired
        type of the concrete Joint. It is templatized by the MobilizedBody type.
        Concrete class cannot override this method but can customize extendAddToSystem()
        instead of using this service. It assumes that the MobilizedBody is
        associated with the child body, unless the Joint is specified as
        reversed in which case the parent is the Body that is "mobilized".
        For more granularity as to which bodies are being interconnected
        internally, and to specify their joint (mobilizer) transforms use:
        @see createMobilizedBody(MobilizedBody& inboard,
                        const SimTK::Transform& inboardTransform,
                            ...).  following this method. */
    template <typename T>
    T createMobilizedBody(SimTK::MultibodySystem& mbs) const
    {
        SimTK::MobilizedBody inb;
        const SimTK::Body* outb = &getChildInternalRigidBody();
        const SimTK::Transform* inbX = &getParentTransform();
        const SimTK::Transform* outbX = &getChildTransform();
        const Body* associatedBody = nullptr;
        // if the joint is reversed then flip the underlying tree representation
        // of inboard and outboard bodies, although the joint direction will be
        // preserved, the inboard must exist first.
        if (get_reverse()){
            inb = mbs.updMatterSubsystem().updMobilizedBody(
                                               getMobilizedBodyIndex(getChildFrame()) );
            const SimTK::Transform* swap = inbX;
            inbX = outbX;
            outbX = swap;

            outb = &getParentInternalRigidBody();
            associatedBody = (_slaveBodyForParent) ? _slaveBodyForParent :
                                                     &getParentFrame();
        }
        else{
            inb = mbs.updMatterSubsystem().updMobilizedBody(
                                               getMobilizedBodyIndex(getParentFrame()) );

            associatedBody = (_slaveBodyForChild) ? _slaveBodyForChild :
                &getChildFrame();
        }

        int startingCoordinateIndex = 0;
        T simtkBody = createMobilizedBody<T>(inb, *inbX,
                                             *outb, *outbX,
                                             startingCoordinateIndex,
                                             associatedBody);

        return simtkBody;
    }

    /** Utility method for creating an underlying MobilizedBody of the desired
    type. Method is templatized by the MobilizedBody. Unlike the previous method,
    the inboard and outboard of the mobilized body are not assumed to be parent
    and child of the joint. This enables Joint component makers to introduce
    intermediate MobilizedBodies for the purpose of creating more complex Joints.
    If more than one MobilizedBody is being created, it is up to the caller to
    supply the corresponding coordinateIndex for the purpose of automatically
    assigning indices necessary for the Coordinates of this Joint to access
    the coordinates and speed values from the state of the MultibodySystem.
    As a convenience the startingCoorinateIndex is updated so
    that sequential calls will increment correctly based on the number of
    mobilties the concrete MobilizedBody enables.

    @param[in] inboard           an existing SimTK::MobilizedBody in the
                                 multibody tree
    @param[in] inboardTransform  the transform locating the joint (mobilizer)
                                 frame on the inboard body
    @param[in] outboard          a SimTK::Rigid::Body to be added to the
                                 multibody tree
    @param[in] outboardTransform  the transform locating the joint (mobilizer)
                                  frame on the outboard body
    @param[in,out] startingCoordinateIndex
                                 the starting index of mobilities
                                 enabled by the created MobilizedBody and used
                                 to assign mobility indices to the Joint's
                                 coordinates. It is incremented by the number of
                                 mobilities of the MobilizedBody created
    @param[in] associatedBody    (optional) the Body associated with the
                                 MobilizeBody. The MobilizedBody index is
                                 assigned to the associated Body.
    */
    template <typename T>
    T createMobilizedBody(SimTK::MobilizedBody& inboard,
                          const SimTK::Transform& inboardTransform,
                          const SimTK::Body& outboard,
                          const SimTK::Transform& outboardTransform,
                          int& startingCoordinateIndex,
                          const OpenSim::Body* associatedBody=nullptr) const {
        // CREATE MOBILIZED BODY
        SimTK::MobilizedBody::Direction dir =
            SimTK::MobilizedBody::Direction(get_reverse());

        T simtkBody(inboard, inboardTransform, outboard, outboardTransform, dir);

        const CoordinateSet& coords = get_CoordinateSet();
        int nc = numCoordinates();

        SimTK_ASSERT1(nc == coords.getSize(), "%s list of coordinates does not match number of mobilities.",
                      getConcreteClassName().c_str());

        startingCoordinateIndex = assignSystemIndicesToBodyAndCoordinates(simtkBody,
            associatedBody,
            getNumMobilities<T>(simtkBody),
            startingCoordinateIndex);

        return simtkBody;
    }

    /** Utility method to assign body and coordinate indices from the underlying
        MultibodySystem from a newly created MobilizedBody. Assign its mobilities
        to OpenSim::Coordinates that belong to this Joint and the body index
        to the Body being mobilized by this Joint. If the Body is NULL then we
        assume that we are constructing intermediate MobilizedBodies and indices
        are assigned to corresponding coordinates, but not to a Body. If a Body
        is provided we assume that Joint creator has provided the correct child
        (or parent, if reversed) body that has been mobilized. This method throws
        an exception if the mobilized Body is neither the Joint's parent or child.*/
    int assignSystemIndicesToBodyAndCoordinates(
        const SimTK::MobilizedBody& mobod,
        const OpenSim::Body* mobilized,
        const int& numMobilities,
        const int& startingCoordinateIndex) const;

private:
    void setNull();

    /** Construct the infrastructure of the Joint component.
        Begin with its properties. */
    void constructProperties() override;

    /** Next define its structural dependencies on other components.
        These will be the parent and child bodies of the Joint.*/
    void constructConnectors() override;

    /** Utility method for accessing the number of mobilities provided by
        an underlying MobilizedBody */
    template <typename T>
    int getNumMobilities(const T& mobod) const
    {
        return mobod.getDefaultQ().size();
    }

    // Only Model's connectToModel can access private
    // members of the Joint to set Joint connected to a slave body
    // of a master body.
    friend Model; // void Model::extendConnectToModel(Model &model);

    void setSlaveBodyForParent(Body& slaveForParent){
        _slaveBodyForParent = slaveForParent;
    }

    void setSlaveBodyForChild(Body& slaveForChild){
        _slaveBodyForChild = slaveForChild;
    }

    //=========================================================================
    // DATA
    //=========================================================================
    // Hold complete transforms for the joint frame's in connected bodies
    SimTK::Transform _jointFrameInChild;
    SimTK::Transform _jointFrameInParent;

    SimTK::ReferencePtr<Body> _slaveBodyForParent;
    SimTK::ReferencePtr<Body> _slaveBodyForChild;

    friend class JointSet;

//=============================================================================
};  // END of class Joint
//=============================================================================
//=============================================================================

// Specializations
template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Pin& mobod) const
{
    return 1;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Slider& mobod) const
{
    return 1;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Weld& mobod) const
{
    return 0;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Universal& mobod) const
{
    return 2;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Ball& mobod) const
{
    return 3;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Ellipsoid& mobod) const
{
    return 3;
}

template <>
inline int Joint::getNumMobilities(const SimTK::MobilizedBody::Free& mobod) const
{
    return 6;
}


} // end of namespace OpenSim

#endif // OPENSIM_JOINT_H_
