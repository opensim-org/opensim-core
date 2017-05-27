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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <simbody/internal/MobilizedBody.h>

namespace OpenSim {

class Model;
class ScaleSet;

/**
An OpenSim Joint is an OpenSim::ModelComponent which connects two PhysicalFrames
together and specifies their relative permissible motion as described in
internal coordinates. The base Joint specifies two frames (e.g. one per body),
which the joint spans. The relative motion (including the # of coordinates)
are defined by concrete Joints, which specify the permissible kinematics of
a child joint frame (on a child body) with respect to a parent joint frame
(on a parent body). The designation of parent and child are used only to
identify the directionality of the joint and in which frame the joint
coordinates are expressed.

For example, A PinJoint between a parent frame, P, and a child frame, B,
has a coordinate value of zero when the two frames are aligned and
positive coordinate values are the angle between the frames' X-axes given
a positive Z-rotation of the child frame about the coincident Z-axis in
the parent frame.

Note: the parent and child frames must be added to the model by the time
      you call initSystem() on the model.

Concrete Joints can specify relative translations and even coupled
rotations and translations (see EllipsoidJoint and CustomJoint). For more
details on how the underlying formulation supports coupled curvilinear
joints, see "Minimal formulation of joint motion for biomechanisms", 2010
A Seth, M Sherman, P Eastman, S Delp; Nonlinear dynamics 62 (1), 291-303

<b>C++ example</b>
\code{.cpp}
// Define a pin joint that attaches pendulum (an OpenSim::Body) to ground.
PinJoint* myPin = new PinJoint("pendulumToGround", myModel.getGround(),
                               pendulum);
\endcode

<b>Python example</b>
\code{.py}
    # Define a ball joint between blockA and blockB.
    abJoint = osim.BallJoint('JointName', blockA, blockB)
\endcode

If you want to connect to an existing PhysicalFrame (e.g., a Body or Ground)
but not to its origin, you can create and connect to a PhysicalOffsetFrame; the
following convenience constructor does this for you:

<b>C++ example</b>
\code{.cpp}
// Define a pin joint that attaches the end of pendulum to the ground origin.
PinJoint* myPin = new PinJoint("pendulumToGround",
                               myModel.getGround(),   //parent PhysicalFrame
                               Vec3(0),               //location in parent
                               Vec3(0),               //orientation in parent
                               pendulum,              //child PhysicalFrame
                               Vec3(0,-length/2.,0),  //location in child
                               Vec3(0));              //orientation in child
\endcode


@author Ajay Seth
*/
class OSIMSIMULATION_API Joint : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Joint, ModelComponent);

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(coordinates, Coordinate,
        "List containing the generalized coordinates (q's) that parameterize "
        "this joint.");

    OpenSim_DECLARE_LIST_PROPERTY(frames, PhysicalFrame,
        "Physical frames owned by the Joint that are used to satisfy the Joint's "
        "parent and child frame connections. For examples, PhysicalOffsetFrames "
        "are often used to offset the connection from a Body's origin to another "
        "location of interest (e.g. the joint center). That offset can be added "
        "to the Joint. When the joint is delete so are the Frames in this list.");

//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(parent_frame, PhysicalFrame,
        "The parent frame for the joint.");
    OpenSim_DECLARE_SOCKET(child_frame, PhysicalFrame,
        "The child frame for the joint.");

//==============================================================================
// OUTPUTS
//==============================================================================
    OpenSim_DECLARE_OUTPUT(power, double, calcPower, SimTK::Stage::Acceleration);
    OpenSim_DECLARE_OUTPUT(reaction_on_parent, SimTK::SpatialVec,
        calcReactionOnParentExpressedInGround, SimTK::Stage::Acceleration);
    OpenSim_DECLARE_OUTPUT(reaction_on_child, SimTK::SpatialVec,
        calcReactionOnChildExpressedInGround, SimTK::Stage::Acceleration);

//==============================================================================
// METHODS
//==============================================================================
    /** Default constructor. Create an unnamed Joint with parent and child frame
        sockets that are unsatisfied. */
    Joint();

    /** Convenience constructor. Create a Joint by specifying the parent and
        child frames.

        @param[in] name    the name associated with this joint (should be unique
                           from other joints in the same model)
        @param[in] parent  the parent PhysicalFrame to which this joint connects
        @param[in] child   the child PhysicalFrame to which this joint connects
    */
    Joint(const std::string&    name,
          const PhysicalFrame&  parent,
          const PhysicalFrame&  child);

    /** Backwards-compatible convenience constructor. Construct a Joint where
        the parent and child frames are specified as well as the location and
        orientation of the parent and child frames in their respective physical
        frames.

        @param[in] name    the name associated with this joint (should be unique
                           from other joints in the same model)
        @param[in] parent  the parent PhysicalFrame to which this joint connects
        @param[in] locationInParent     Vec3 of the location of the joint in the
                                        parent frame.
        @param[in] orientationInParent  Vec3 of the XYZ body-fixed Euler angles
                                        of the joint frame orientation in the
                                        parent frame.
        @param[in] child   the child PhysicalFrame to which this joint connects
        @param[in] locationInChild      Vec3 of the location of the joint in the
                                        child frame.
        @param[in] orientationInChild   Vec3 of the XYZ body-fixed Euler angles
                                        of the joint frame orientation in the
                                        child frame.
    */
    Joint(const std::string&    name,
          const PhysicalFrame&  parent,
          const SimTK::Vec3&    locationInParent,
          const SimTK::Vec3&    orientationInParent,
          const PhysicalFrame&  child,
          const SimTK::Vec3&    locationInChild,
          const SimTK::Vec3&    orientationInChild);

    virtual ~Joint();

    // GET & SET
    /**
     * Get the child joint frame.
     *
     * @return const PhysicalFrame reference.
     */
    const PhysicalFrame& getChildFrame() const;

    /**
     * Get the parent frame to which this joint attaches.
     *
     * @return const ref to parent PhysicalFrame.
     */
    const OpenSim::PhysicalFrame& getParentFrame() const;

    /** Convenience method to get a const reference to the Coordinate associated
        with a single-degree-of-freedom Joint. If the Joint has more than one
        Coordinate, you must use get_coordinates() or provide the appropriate
        argument to the getCoordinate() method defined in the derived class. */
    const Coordinate& getCoordinate() const;

    /** Convenience method to get a writable reference to the Coordinate
        associated with a single-degree-of-freedom Joint. If the Joint has more
        than one Coordinate, you must use upd_coordinates() or provide the
        appropriate argument to the updCoordinate() method defined in the
        derived class. */
    Coordinate& updCoordinate();

    // Model building
    int numCoordinates() const { return getProperty_coordinates().size(); }

    // Utility
    bool isCoordinateUsed(const Coordinate& aCoordinate) const;

    // Computation
    /** Given some system mobility (generalized) forces, calculate the 
    equivalent spatial body force for this Joint. Keep in mind that there are 
    typically nm < 6 mobilities per joint with an infinite set of solutions that 
    can map nm gen forces to 6 spatial force components (3 for torque + 3 for 
    force). The solution returned provides the "most" effective force and torque
    in the joint frame. This means the smallest magnitude force and/or torque 
    that will result in the same generalized force. If a generalized force is 
    defined along/about a joint axis, then this should be evident in the 
    reported results as a force or torque on the same axis.  NOTE: Joints 
    comprised of multiple mobilizers and/or constraints, should override this 
    method and account for multiple internal components.

    @param state containing the generalized coordinate and speed values 
    @param mobilityForces for the system as computed by inverse dynamics, 
                          for example 
    @return spatial force, FB_G, acting on the body connected by this joint at 
    its location B, expressed in ground.  */
    SimTK::SpatialVec 
        calcEquivalentSpatialForce(const SimTK::State& state, 
                               const SimTK::Vector &mobilityForces) const;
    
    /// Joint Reaction forces 
    /** Calculate the joint reaction force and moment acting on the parent frame
        and expressed in Ground. 
    @param[in]  state containing the generalized coordinate and speed values 
    @return     SpatialVec of reaction force, RP_G, acting on parent frame, P,
                and expressed in ground, G.  */
    SimTK::SpatialVec
        calcReactionOnParentExpressedInGround(const SimTK::State &state) const {
        return getChildFrame().getMobilizedBody()
            .findMobilizerReactionOnParentAtFInGround(state);
    }
    /** Calculate the joint reaction force and moment acting on the child frame
        and expressed in Ground.
    @param[in]  state containing the generalized coordinate and speed values 
    @return     SpatialVec of reaction force, RP_G, acting on child frame, C,
                and expressed in ground, G.  */
    SimTK::SpatialVec
        calcReactionOnChildExpressedInGround(const SimTK::State &state) const {
        return getChildFrame().getMobilizedBody()
            .findMobilizerReactionOnBodyAtMInGround(state);
    }

    /** Joints in general do not contribute power since the reaction space
        forces are orthogonal to the mobility space. However, when joint motion 
        is prescribed, the internal forces that move the joint will do work. In 
        that case, the power is non-zero and the supplied SimTK::State
        must already have been realized to %Acceleration stage so that 
        constraint forces are available. */
    virtual double calcPower(const SimTK::State &s) const;

    // SCALE
    /**
    * Scale a joint based on XYZ scale factors for PhysicalFrames.
    * Generic behavior is to scale the locations of parent and child offsets
    * according to scale factors of the physical frame upon which they are located.
    *
    * Joint subclasses should invoke this method before scaling joint specific
    * properties
    *
    * @param aScaleSet Set of XYZ scale factors for the bodies.
    */
    virtual void scale(const ScaleSet& aScaleSet);

#ifndef SWIG
    /// @class CoordinateIndex
    /// Unique integer type for local Coordinate indexing
    SimTK_DEFINE_UNIQUE_INDEX_TYPE(CoordinateIndex);

    /** Get the MotionType for a Coordinate that this Joint has enabled by
        its CoordinateIndex (in the Joints list of Coordinates). */
    Coordinate::MotionType getMotionType(CoordinateIndex cix) const;
#endif //SWIG
protected:
    /** A CoordinateIndex member is created by constructCoordinate(). E.g.:
    \code{.cpp}
    class My2DofJoint : public Joint {
    public:
        enum class Coord: unsigned {
            RotationX,
            TranslationX
        };

    private:
        CoordinateIndex rx{ constructCoordinate(Coordinate::MotionType::Rotational,
                            static_cast<unsigned>(Coord::RotationX)) };
        CoordinateIndex tx{ constructCoordinate(Coordinate::MotionType::Translational,
                            static_cast<unsigned>(Coord::TranslationX)) };
        ...
    };
    \endcode
    */
#ifndef SWIG
    /** Utility for a derived Joint to add the Coordinates that correspond to
    the motion it permits. Derived Joints must construct as many Coordinates as
    are reflected by the underlying Mobilizer Qs. The index of the corresponding
    enumeration (enum) is required at construction to ensure the Joint's
    internal list of Coordinates is consistent with its interface; an exception
    is thrown if the Coordinates are not constructed in the same order as the
    enums have been defined. */
    CoordinateIndex constructCoordinate(Coordinate::MotionType mt,
                                        unsigned idx);

    // This is only intended to allow the CustomJoint to set the MotionTypes
    // of its Coordinates
    void setMotionType(CoordinateIndex cix, Coordinate::MotionType mt);
#endif //SWIG

    // build Joint transforms from properties
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    // Methods that allow access for Joint subclasses to data members of
    // Body and Coordinate , which Joint befriends
    const SimTK::MobilizedBodyIndex getMobilizedBodyIndex(const Body& body) const;

    void setChildMobilizedBodyIndex(SimTK::MobilizedBodyIndex index) const;
    void setCoordinateMobilizedBodyIndex(Coordinate *aCoord, SimTK::MobilizedBodyIndex index) const {aCoord->_bodyIndex = index;}
    void setCoordinateMobilizerQIndex(Coordinate *aCoord, int index) const
        { aCoord->_mobilizerQIndex = SimTK::MobilizerQIndex(index);}
    void setCoordinateModel(Coordinate *aCoord, Model *aModel) const {aCoord->_model = aModel;}

    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;

    /** Calculate the equivalent spatial force, FB_G, acting on a mobilized body
        specified by index acting at its mobilizer frame B, expressed in ground. */
    SimTK::SpatialVec 
        calcEquivalentSpatialForceForMobilizedBody(const SimTK::State &s,
            const SimTK::MobilizedBodyIndex mbx, 
            const SimTK::Vector &mobilityForces) const;

    /** Return the equivalent (internal) SimTK::Rigid::Body for the parent/child
        OpenSim::Body. Not valid until after extendAddToSystem on the Body has been called.*/
    const SimTK::Body& getParentInternalRigidBody() const;
    const SimTK::Body& getChildInternalRigidBody() const;


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
        SimTK::Transform inbX = getParentFrame().findTransformInBaseFrame();
        SimTK::Transform outbX = getChildFrame().findTransformInBaseFrame();
        const PhysicalFrame* associatedFrame = nullptr;
        // if the joint is reversed then flip the underlying tree representation
        // of inboard and outboard bodies, although the joint direction will be
        // preserved, the inboard must exist first.
        if (isReversed){
            inb = getChildFrame().getMobilizedBody();
            SimTK::Transform swap = inbX;
            inbX = outbX;
            outbX = swap;

            outb = &getParentInternalRigidBody();
            associatedFrame = _slaveBodyForParent ? _slaveBodyForParent.get() 
                                                  : &getParentFrame();
        }
        else{
            inb = getParentFrame().getMobilizedBody();

            associatedFrame = _slaveBodyForChild ? _slaveBodyForChild.get()
                                                 : &getChildFrame();
        }

        int startingCoordinateIndex = 0;
        T simtkBody = createMobilizedBody<T>(inb, inbX,
                                             *outb, outbX,
                                             startingCoordinateIndex,
                                             associatedFrame);

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
    mobilities the concrete MobilizedBody enables.

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
    @param[in] physicalFrame     (optional) the PhysicalFrame associated with
                                 the MobilizeBody. The MobilizedBody index is
                                 assigned to the associated Body.
    */
    template <typename T>
    T createMobilizedBody(SimTK::MobilizedBody& inboard,
                          const SimTK::Transform& inboardTransform,
                          const SimTK::Body& outboard,
                          const SimTK::Transform& outboardTransform,
                          int& startingCoordinateIndex,
                          const PhysicalFrame* physicalFrame = nullptr) const {
        // CREATE MOBILIZED BODY
        SimTK::MobilizedBody::Direction dir =
            SimTK::MobilizedBody::Direction(isReversed);

        T simtkBody(inboard, inboardTransform, outboard, outboardTransform, dir);

        startingCoordinateIndex = assignSystemIndicesToBodyAndCoordinates(simtkBody,
            physicalFrame,
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
        const OpenSim::PhysicalFrame* mobilized,
        const int& numMobilities,
        const int& startingCoordinateIndex) const;

private:
    void setNull();

    /** Construct the infrastructure of the Joint component.
        Begin with its properties. */
    void constructProperties();

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

    //==========================================================================
    // DATA
    //==========================================================================
protected:
    // Specifies the direction of the Joint in the multibody tree: parent->child
    // (isReversed is false; default) or child->parent (isReversed is true). The
    // Joint's transform and coordinates maintain a parent->child sense even if
    // the joint is reversed.
    bool isReversed = false;

private:
    SimTK::ReferencePtr<Body> _slaveBodyForParent;
    SimTK::ReferencePtr<Body> _slaveBodyForChild;

    SimTK::Array_<Coordinate::MotionType> _motionTypes;

    friend class JointSet;

//==============================================================================
};  // END of class Joint
//==============================================================================
//==============================================================================

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

class JointHasNoCoordinates : public Exception {
public:
    JointHasNoCoordinates(const std::string& file,
                          size_t line,
                          const std::string& func) :
        Exception(file, line, func) {
        std::string mesg = "The Joint has no Coordinates.";

        addMessage(mesg);
    }
};

} // end of namespace OpenSim

#endif // OPENSIM_JOINT_H_
