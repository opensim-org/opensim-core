#ifndef OPENSIM_TWO_FRAME_LINKER_H_
#define OPENSIM_TWO_FRAME_LINKER_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  TwoFrameLinker.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include <OpenSim/Simulation/Model/ForceConsumer.h>
#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <simbody/internal/MobilizedBody.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * TwoFrameLinker is a utility class to extend a Component such that it connects
 * two Frames. For example, a WeldConstraint and BushingForces operate between
 * two frames to restrict their motion. A TwoFrameLinker<Force, PhysicalFrame>,
 * for example, is a Force that operates between two PhyscialFrames and it is
 * the base class for BushingForces.
 * (A class whose super class is a template parameter is called a mixin class.)
 *
 * @code
 *    class BushingForce : public TwoFrameLinker<Force, PhysicalFrame>
 * @endcode
 *
 * @tparam C The base class.
 * @tparam F The type of frame that the class links together.
 *
 * @author Ajay Seth
 */
template <class C = Component, class F = Frame>
class TwoFrameLinker : public C {
    OpenSim_DECLARE_ABSTRACT_OBJECT_T(TwoFrameLinker, C, C);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /// Frames added to satisfy the sockets of this TwoFrameLinker Component
    OpenSim_DECLARE_LIST_PROPERTY(frames, F,
        "Frames created/added to satisfy this component's connections.");

//==============================================================================
// SOCKETS
//==============================================================================

    OpenSim_DECLARE_SOCKET(frame1, F,
            "The first frame participating in this linker.");
    OpenSim_DECLARE_SOCKET(frame2, F,
            "The second frame participating in this linker.");

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** By default, the TwoFrameLinker is not connected to any frames. */
    TwoFrameLinker();

    /** Convenience Constructor.
    Create a TwoFrameLinker Component between two Frames.

    @param[in] name         the name of this TwoFrameLinker component
    @param[in] frame1       the name of the first Frame being linked
    @param[in] frame2       the name of the second Frame being linked
    */
    TwoFrameLinker(const std::string &name, const F& frame1, const F& frame2);

    /** Convenience Constructor.
    Create a TwoFrameLinker Component between two Frames identified by name.

    @param[in] name         the name of this TwoFrameLinker component
    @param[in] frame1Name   the name of the first Frame being linked
    @param[in] frame2Name   the name of the second Frame being linked
    */
    TwoFrameLinker(const std::string &name,
        const std::string& frame1Name,
        const std::string& frame2Name);

    /** Convenience Constructor with two Frames
    Construct a TwoFrameLinker where the two frames are specified by name
    and offset transforms on the respective frames.

    @param[in] name             the name of this TwoFrameLinker component
    @param[in] frame1           the first Frame being linked
    @param[in] offsetOnFrame1   offset Transform on the first frame
    @param[in] frame2           the second Frame being linked
    @param[in] offsetOnFrame2   offset Transform on the second frame
    */
    TwoFrameLinker(const std::string &name,
        const F& frame1, const SimTK::Transform& offsetOnFrame1,
        const F& frame2, const SimTK::Transform& offsetOnFrame2);

    /** Convenience Constructor
    Construct a TwoFrameLinker where the two frames are specified by name
    and offset transforms on the respective frames.

    @param[in] name             the name of this TwoFrameLinker component
    @param[in] frame1Name       the name of the first Frame being linked
    @param[in] offsetOnFrame1   offset Transform on the first frame
    @param[in] frame2Name       the name of the second Frame being linked
    @param[in] offsetOnFrame2   offset Transform on the second frame
    */
    TwoFrameLinker(const std::string &name,
        const std::string& frame1Name,
        const SimTK::Transform& offsetOnFrame1,
        const std::string& frame2Name,
        const SimTK::Transform& offsetOnFrame2);

    /** Backwards compatible Convenience Constructor
    TwoFrameLinker with offsets specified in terms of the location and
    orientation in respective PhysicalFrames.

    @param[in] name             the name of this TwoFrameLinker component
    @param[in] frame1              the first Frame being linked
    @param[in] locationInFrame1    Vec3 of offset location on the first frame
    @param[in] orientationInFrame1 Vec3 of orientation offset expressed as
                                   XYZ body-fixed Euler angles w.r.t frame1.
    @param[in] frame2              the second Frame being linked
    @param[in] locationInFrame2    Vec3 of offset location on the second frame
    @param[in] orientationInFrame2 Vec3 of orientation offset expressed as
                                   XYZ body-fixed Euler angles w.r.t frame2.
    */
    TwoFrameLinker(const std::string &name,
                   const PhysicalFrame& frame1,
                   const SimTK::Vec3& locationInFrame1,
                   const SimTK::Vec3& orientationInFrame1,
                   const PhysicalFrame& frame2,
                   const SimTK::Vec3& locationInFrame2,
                   const SimTK::Vec3& orientationInFrame2);

    /** Backwards compatible Convenience Constructor
    TwoFrameLinker with offsets specified in terms of the location and
    orientation in respective PhysicalFrames.

    @param[in] name             the name of this TwoFrameLinker component
    @param[in] frame1Name       the name of the first Frame being linked
    @param[in] locationInFrame1    Vec3 of offset location on the first frame
    @param[in] orientationInFrame1 Vec3 of orientation offset expressed as
                                   XYZ body-fixed Euler angles w.r.t frame1.
    @param[in] frame2Name       the name of the second Frame being linked
    @param[in] locationInFrame2    Vec3 of offset location on the second frame
    @param[in] orientationInFrame2 Vec3 of orientation offset expressed as
                                   XYZ body-fixed Euler angles w.r.t frame2.
    */
    TwoFrameLinker(const std::string &name,
        const std::string& frame1Name,
        const SimTK::Vec3& locationInFrame1,
        const SimTK::Vec3& orientationInFrame1,
        const std::string& frame2Name,
        const SimTK::Vec3& locationInFrame2,
        const SimTK::Vec3& orientationInFrame2);

    // use compiler generated destructor, copy constructor & assignment operator

    /** Access the first frame the TwoFrameLinker component connects.
        Note, if an offset was introduced at construction, then this will be
        the offset frame. */
    const F& getFrame1() const;
    /** Access the second frame the TwoFrameLinker component connects.
        Note, if an offset was introduced at construction, then this will be
        the offset frame. */
    const F& getFrame2() const;

    /** Compute the relative offset Transform between the two frames linked by
        this TwoFrameLinker component at a given State, expressed in frame1. */
    SimTK::Transform computeRelativeOffset(const SimTK::State& s) const;

    /** Compute the relative spatial velocity between the two frames linked by
        this TwoFrameLinker component at a given State, expressed in frame1. */
    SimTK::SpatialVec computeRelativeVelocity(const SimTK::State& s) const;

    /** Compute the deflection (spatial separation) of the two frames connected
        by the TwoFrameLinker. Angular deflections expressed as XYZ body-fixed
        Euler angles of frame2 w.r.t frame1.
        NOTE: When using deflections to compute spatial forces, these forces
            may not be valid for large deflections, because Euler angles are
            unable to uniquely distinguish an X rotation angle of +/-180 degs,
            and subsequent rotations that are +/-90 degs. It is mainly useful
            for calculating errors for constraints and forces for computing
            restoration forces.
     @return dq     Vec6 of (3) angular and (3) translational deflections. */
    SimTK::Vec6 computeDeflection(const SimTK::State& s) const;
    /** Compute the deflection rate (dqdot) of the two frames connected by
        this TwoFrameLinker component. Angular velocity is expressed as Euler
        (XYZ body-fixed) angle derivatives. Note that the derivatives
        become singular as the second Euler angle approaches 90 degs.
    @return dqdot  Vec6 of (3) angular and (3) translational deflection rates. */
    SimTK::Vec6 computeDeflectionRate(const SimTK::State& s) const;

protected:
    /** @name Component Interface
        These methods adhere to the Component Interface**/
    /**@{**/
    void extendConnectToModel(Model& model) override;
    // update previous model formats for all components linking two frames
    // in one place - here.
    void
    updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;
    /**@}**/

    /** Helper method to convert internal force expressed in the mobility basis
        between frame1 and frame2, dq, as individual spatial forces acting on
        each frame linked by this component. The internal force, f, is directed
        onto frame2 from frame1.
        In computeDeflection(), frame2's pose in frame1 is decomposed in to 6
        internal coordinates dq (similar to the q's of a free joint). Their
        derivatives, dqdot, form a internal coordinates "mobility" basis
        that can be used to express frame2's spatial velocity in frame1 or to
        apply internal (generalized) forces that apply equal and opposite
        spatial forces to the two frames.
    @param state            const State of current system configuration
    @param[in] f            Vec6 of forces in the basis of the deflection
    @param[in,out] F1_G     SpatialVec Force (torque, force) applied to
                            frame1 expressed in ground.
    @param[in,out] F2_G     SpatialVec Force (torque, force) applied to
                            frame2 expressed in ground. */
    void convertInternalForceToForcesOnFrames(
        const SimTK::State& state,
        SimTK::Vec6 f, SimTK::SpatialVec& F1_G, SimTK::SpatialVec& F2_G) const;

    /** Helper method to add in the equivalent system spatial forces given
        internal forces. Internal forces are expressed in the mobility basis as
        parameterized by the deflection between frame1 and frame2, dq, and its
        time derivative, dqdot.
        The intended usage is within a Force::computeForce() which is tasked with
        populating vectors of generalized forces and/or (mobilized body) spatial
        forces. This method will populate (add in) the spatialForces given the
        internal force.
    @param state                const State of current system configuration
    @param[in] f                Vec6 of forces in the basis of the deflection
    @param[in,out] spatialForces   Vector of SpatialVec's (torque, force) on
                                   each PhysicalFrame's base frame in the System.
                                   This is equivalent to each MobilizedBody.
    @see convertInternalForceToForcesOnFrames()
    @see PhysicalFrame::findBaseFrame() */
    void addInPhysicalForcesFromInternal(const SimTK::State& state,
        SimTK::Vec6 f, SimTK::Vector_<SimTK::SpatialVec>& spatialForces) const;

    /**
     * Produces system spatial forces given internal forces.
     *
     * Internal forces are expressed in the mobility basis as parameterized by the
     * deflection between `frame1` and `frame2`, `dq`, and its time derivative, `dqdot`.
     *
     * This helper function is intended to be used as part of a `ForceProducer::implProduceForces`
     * implementation.
     *
     * @param state            the state used to evaluate forces
     * @param f                a `SimTK::Vec6` of forces in the basis of the deflection
     * @param forceConsumer    a `ForceConsumer` that shall recieve each evaluated force
     */
    void producePhysicalForcesFromInternal(
        const SimTK::State& state,
        const SimTK::Vec6& f,
        ForceConsumer& forceConsumer
    ) const;

private:
    // create the frames property
    void constructProperties();

    //hang on to references to the individual frames for fast access
    mutable SimTK::ReferencePtr<const F> _frame1;
    mutable SimTK::ReferencePtr<const F> _frame2;

//=============================================================================
}; // END of class OffsetFrame
//=============================================================================



//=============================================================================
// Implementation of OffsetFrame<C> template methods
//=============================================================================
// Default constructor
template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker() : C()
{
    this->setAuthors("Ajay Seth");
    this->constructProperties();
}

template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker(const std::string &name,
    const F& frame1,
    const F& frame2) : TwoFrameLinker<C, F>()
{
    this->setName(name);
    this->template updSocket<F>("frame1").connect(frame1);
    this->template updSocket<F>("frame2").connect(frame2);
}

// Convenience constructors
template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker(const std::string &name,
    const std::string& frame1Name,
    const std::string& frame2Name) : TwoFrameLinker<C, F>()
{
    this->setName(name);
    this->template updSocket<F>("frame1").setConnecteePath(frame1Name);
    this->template updSocket<F>("frame2").setConnecteePath(frame2Name);
}

template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker(const std::string &name,
    const F& frame1, const SimTK::Transform& transformInFrame1,
    const F& frame2, const SimTK::Transform& transformInFrame2)
    : TwoFrameLinker()
{
    this->setName(name);

    PhysicalOffsetFrame frame1Offset(frame1.getName() + "_offset",
        frame1, transformInFrame1);

    PhysicalOffsetFrame frame2Offset(frame2.getName() + "_offset",
        frame2, transformInFrame2);

    // Append the offset frames to the Joints internal list of frames
    int ix1 = append_frames(frame1Offset);
    int ix2 = append_frames(frame2Offset);
    this->finalizeFromProperties();

    this->connectSocket_frame1(get_frames(ix1));
    this->connectSocket_frame2(get_frames(ix2));

    static_cast<PhysicalOffsetFrame&>(upd_frames(ix1)).setParentFrame(frame1);
    static_cast<PhysicalOffsetFrame&>(upd_frames(ix2)).setParentFrame(frame2);
}


template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker(const std::string &name,
    const std::string& frame1Name, const SimTK::Transform& transformInFrame1,
    const std::string& frame2Name, const SimTK::Transform& transformInFrame2)
    : TwoFrameLinker()
{
    this->setName(name);

    PhysicalOffsetFrame frame1Offset(frame1Name + "_offset",
        frame1Name, transformInFrame1);

    PhysicalOffsetFrame frame2Offset(frame2Name + "_offset",
        frame2Name, transformInFrame2);

    // Append the offset frames to the Joints internal list of frames
    int ix1 = append_frames(frame1Offset);
    int ix2 = append_frames(frame2Offset);
    this->finalizeFromProperties();

    this->connectSocket_frame1(get_frames(ix1));
    this->connectSocket_frame2(get_frames(ix2));
}

template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker(const std::string &name,
               const PhysicalFrame& frame1,
               const SimTK::Vec3& locationInFrame1,
               const SimTK::Vec3& orientationInFrame1,
               const PhysicalFrame& frame2,
               const SimTK::Vec3& locationInFrame2,
               const SimTK::Vec3& orientationInFrame2)
    : TwoFrameLinker(name,
        frame1, SimTK::Transform(SimTK::Rotation(SimTK::BodyRotationSequence,
            orientationInFrame1[0], SimTK::XAxis,
            orientationInFrame1[1], SimTK::YAxis,
            orientationInFrame1[2], SimTK::ZAxis), locationInFrame1),
        frame2, SimTK::Transform(SimTK::Rotation(SimTK::BodyRotationSequence,
            orientationInFrame2[0], SimTK::XAxis,
            orientationInFrame2[1], SimTK::YAxis,
            orientationInFrame2[2], SimTK::ZAxis), locationInFrame2))
{}

template <class C, class F>
TwoFrameLinker<C, F>::TwoFrameLinker(const std::string& name,
    const std::string& frame1Name,
    const SimTK::Vec3& locationInFrame1, const SimTK::Vec3& orientationInFrame1,
    const std::string& frame2Name,
    const SimTK::Vec3& locationInFrame2, const SimTK::Vec3& orientationInFrame2)
    : TwoFrameLinker(name,
        frame1Name, SimTK::Transform(SimTK::Rotation(SimTK::BodyRotationSequence,
            orientationInFrame1[0], SimTK::XAxis,
            orientationInFrame1[1], SimTK::YAxis,
            orientationInFrame1[2], SimTK::ZAxis), locationInFrame1),
        frame2Name, SimTK::Transform(SimTK::Rotation(SimTK::BodyRotationSequence,
            orientationInFrame2[0], SimTK::XAxis,
            orientationInFrame2[1], SimTK::YAxis,
            orientationInFrame2[2], SimTK::ZAxis), locationInFrame2))
{}


template <class C, class F>
void TwoFrameLinker<C, F>::constructProperties()
{
    //Default frames list is empty
    this->constructProperty_frames();
}

template <class C, class F>
const F& TwoFrameLinker<C, F>::getFrame1() const
{
    if (!(this->isObjectUpToDateWithProperties() && this->hasSystem())) {
        _frame1 = &(this->template getSocket<F>("frame1").getConnectee());
    }
    return _frame1.getRef();
}

template <class C, class F>
const F& TwoFrameLinker<C, F>::getFrame2() const
{
    if (!(this->isObjectUpToDateWithProperties() && this->hasSystem())) {
        _frame2 = &(this->template getSocket<F>("frame2").getConnectee());
    }
    return _frame2.getRef();
}

template<class C, class F>
void TwoFrameLinker<C, F>::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model); //connect to frames
    // now keep a reference to the frames
    _frame1 = &(this->template getSocket<F>("frame1").getConnectee());
    _frame2 = &(this->template getSocket<F>("frame2").getConnectee());
}


//=============================================================================
// UTILITY COMPUTATIONS
//=============================================================================
template <class C, class F>
SimTK::Transform TwoFrameLinker<C, F>::computeRelativeOffset(const SimTK::State& s) const
{
    // Define frame1 as the "fixed" frame, F
    SimTK::Transform X_GF = getFrame1().getTransformInGround(s);
    // Define the frame2 as the "moving" frame, M
    SimTK::Transform X_GM = getFrame2().getTransformInGround(s);
    // Express M in F
    return ~X_GF * X_GM;
}

template <class C, class F>
SimTK::Vec6 TwoFrameLinker<C, F>::computeDeflection(const SimTK::State& s) const
{
    SimTK::Transform X_FM = this->computeRelativeOffset(s);

    // Calculate stiffness generalized forces of bushing by first computing
    // the deviation of the two frames measured by dq
    SimTK::Vec6 dq(0);
    // First 3 are rotational deviations
    dq.updSubVec<3>(0) = X_FM.R().convertRotationToBodyFixedXYZ();
    // Second 3 are translational
    dq.updSubVec<3>(3) = X_FM.p();

    return dq;
}

template <class C, class F>
SimTK::SpatialVec TwoFrameLinker<C, F>::computeRelativeVelocity(const SimTK::State& s) const
{
    const F& frame1 = getFrame1();
    const F& frame2 = getFrame2();

    const SimTK::Transform& X_GB1 = frame1.getMobilizedBody().getBodyTransform(s);
    const SimTK::Transform& X_GB2 = frame2.getMobilizedBody().getBodyTransform(s);

    SimTK::Transform X_GF = frame1.getTransformInGround(s);
    SimTK::Transform X_GM = frame2.getTransformInGround(s);
    SimTK::Transform X_FM = ~X_GF * X_GM;
    const SimTK::Rotation& R_GF = X_GF.R();

    // Now evaluate velocities.
    const SimTK::SpatialVec& V_GB1 = frame1.getMobilizedBody().getBodyVelocity(s);
    const SimTK::SpatialVec& V_GB2 = frame2.getMobilizedBody().getBodyVelocity(s);

    // Re-express local vectors in the Ground frame.
    SimTK::Vec3 p_B1F_G = X_GB1.R() * frame1.findTransformInBaseFrame().p();
    SimTK::Vec3 p_B2M_G = X_GB2.R() * frame2.findTransformInBaseFrame().p();
    SimTK::Vec3 p_FM_G = X_GF.R()  * X_FM.p();    // 15 flops

    SimTK::SpatialVec V_GF(V_GB1[0], V_GB1[1] + V_GB1[0] % p_B1F_G);
    SimTK::SpatialVec V_GM(V_GB2[0], V_GB2[1] + V_GB2[0] % p_B2M_G);

    // This is the velocity of M in F, but with the time derivative
    // taken in the Ground frame.
    const SimTK::SpatialVec V_FM_G = V_GM - V_GF;

    // To get derivative in F, we must remove the part due to the
    // angular velocity w_GF of F in G.
    return ~R_GF * SimTK::SpatialVec(V_FM_G[0], V_FM_G[1] - V_GF[0] % p_FM_G);
}

template <class C, class F>
SimTK::Vec6 TwoFrameLinker<C, F>::computeDeflectionRate(const SimTK::State& s) const
{
    SimTK::Vec6 dqdot(0);
    SimTK::Vec6 dq = computeDeflection(s);

    // Evaluate relative transform
    SimTK::Transform X_FM = this->computeRelativeOffset(s);
    // Evaluate velocity
    SimTK::SpatialVec V_FM = computeRelativeVelocity(s);

    // Need angular velocity in M frame for conversion to qdot.
    const SimTK::Vec3  w_FM_M = ~(X_FM.R()) * V_FM[0];
    const SimTK::Mat33 N_FM =
        SimTK::Rotation::calcNForBodyXYZInBodyFrame(dq.getSubVec<3>(0));

    dqdot.updSubVec<3>(0) = N_FM * w_FM_M;
    dqdot.updSubVec<3>(3) = V_FM[1];

    return dqdot;
}

template <class C, class F>
void TwoFrameLinker<C, F>::convertInternalForceToForcesOnFrames(
    const SimTK::State& s,
    SimTK::Vec6 f, SimTK::SpatialVec& F1_G, SimTK::SpatialVec& F2_G) const
{
    // internal force on body 2
    const SimTK::Vec3& fB2_q = f.getSubVec<3>(0); // in q basis
    const SimTK::Vec3& fM_F = f.getSubVec<3>(3); // acts at M, but exp. in F frame

    SimTK::Vec6 dq = computeDeflection(s);

    // get connected frames
    const F& frame1 = getFrame1();
    const F& frame2 = getFrame2();

    //const SimTK::Transform& X_GB1 = frame1.getMobilizedBody().getBodyTransform(s);
    //const SimTK::Transform& X_GB2 = frame2.getMobilizedBody().getBodyTransform(s);

    SimTK::Transform X_GF = frame1.getTransformInGround(s);
    SimTK::Transform X_GM = frame2.getTransformInGround(s);
    SimTK::Transform X_FM = ~X_GF * X_GM;
    const SimTK::Mat33 N_FM =
        SimTK::Rotation::calcNForBodyXYZInBodyFrame(dq.getSubVec<3>(0));

    // Calculate the matrix relating q-space generalized forces to a real-space
    // moment vector. We know qforce = ~H * moment (where H is the
    // the hinge matrix for a mobilizer using qdots as generalized speeds).
    // In that case H would be N^-1, qforce = ~(N^-1)*moment so
    // moment = ~N*qforce. Caution: our N wants the moment in the outboard
    // body frame, in this case M.
    const SimTK::Vec3  mB2_M = ~N_FM * fB2_q; // moment acting on body 2, exp. in M
    const SimTK::Vec3  mB2_G = X_GM.R() * mB2_M; // moment on body 2, now exp. in G

    // Transform force from F frame to ground. This is the force to
    // apply to body 2 at point OM; -f goes on body 1 at the same
    // spatial location. Here we actually apply it at OF so we have to
    // account for the moment produced by the shift from OM.
    const SimTK::Vec3 fM_G = X_GF.R()*fM_F;

    // Re-express local vectors in the Ground frame.
    SimTK::Vec3 p_FM_G = X_GF.R()  * X_FM.p();    // 15 flops

    F2_G = SimTK::SpatialVec( mB2_G, fM_G);
    F1_G = SimTK::SpatialVec(-(mB2_G + p_FM_G % fM_G), -fM_G);
}

// The method only makes sense for applying forces to PhysicalFrames so explicitly
// specialize for PhysicalFrame.
template <class C, class F>
void TwoFrameLinker<C, F>::addInPhysicalForcesFromInternal(
        const SimTK::State& s,
        SimTK::Vec6 f, SimTK::Vector_<SimTK::SpatialVec>& physicalForces) const
{
    // A `ForceConsumer` that applies body spatial vectors from `producePhysicalForcesFromInternal`
    class Adaptor final : public ForceConsumer {
    public:
        explicit Adaptor(SimTK::Vector_<SimTK::SpatialVec>& physicalForces) :
            _physicalForces{&physicalForces}
        {}

    private:
        void implConsumeBodySpatialVec(
            const SimTK::State& state,
            const PhysicalFrame& body,
            const SimTK::SpatialVec& spatialVec) final
        {
            (*_physicalForces)[body.getMobilizedBodyIndex()] += spatialVec;
        }

        SimTK::Vector_<SimTK::SpatialVec>* _physicalForces;
    };

    Adaptor adaptor{physicalForces};
    producePhysicalForcesFromInternal(s, f, adaptor);
}

template<class C, class F>
void TwoFrameLinker<C, F>::producePhysicalForcesFromInternal(
    const SimTK::State& s,
    const SimTK::Vec6& f,
    ForceConsumer& forceConsumer) const
{
    SimTK::SpatialVec F_GF;
    SimTK::SpatialVec F_GM;
    // convert the internal force to spatial forces on the two frames
    convertInternalForceToForcesOnFrames(s, f, F_GF, F_GM);

    // get connected frames
    const F& frame1 = getFrame1();
    const F& frame2 = getFrame2();

    const SimTK::Transform& X_GB1 = frame1.getMobilizedBody().getBodyTransform(s);
    const SimTK::Transform& X_GB2 = frame2.getMobilizedBody().getBodyTransform(s);

    SimTK::Vec3 p_B1F_G = X_GB1.R() * frame1.findTransformInBaseFrame().p();
    SimTK::Vec3 p_B2M_G = X_GB2.R() * frame2.findTransformInBaseFrame().p();

    // Shift forces to body origins.
    SimTK::SpatialVec F_GB2(F_GM[0] + p_B2M_G % F_GM[1], F_GM[1]);
    SimTK::SpatialVec F_GB1(F_GF[0] + p_B1F_G % F_GF[1], F_GF[1]);

    // Produce the body forces as body spatial vectors.
    forceConsumer.consumeBodySpatialVec(s, frame2, F_GB2);
    forceConsumer.consumeBodySpatialVec(s, frame1, F_GB1);
}


template <class C, class F>
void TwoFrameLinker<C, F>::updateFromXMLNode(SimTK::Xml::Element& aNode,
                                            int versionNumber)
{
    using SimTK::Vec3;
    int documentVersion = versionNumber;
    if (documentVersion < XMLDocument::getLatestVersion()) {
        if (documentVersion < 30505) {
            // replace old properties with latest use of PhysicalOffsetFrames
            // properties
            SimTK::Xml::element_iterator body1Element =
                aNode.element_begin("body_1");
            SimTK::Xml::element_iterator body2Element =
                aNode.element_begin("body_2");
            SimTK::Xml::element_iterator locBody1Elt =
                aNode.element_begin("location_body_1");
            SimTK::Xml::element_iterator orientBody1Elt =
                aNode.element_begin("orientation_body_1");
            SimTK::Xml::element_iterator locBody2Elt =
                aNode.element_begin("location_body_2");
            SimTK::Xml::element_iterator orientBody2Elt =
                aNode.element_begin("orientation_body_2");

            // The names of the two PhysicalFrames this bushing connects
            std::string frame1Name("");
            std::string frame2Name("");

            if (body1Element != aNode.element_end()) {
                body1Element->getValueAs<std::string>(frame1Name);
            }

            if (body2Element != aNode.element_end()) {
                body2Element->getValueAs<std::string>(frame2Name);
            }

            Vec3 locationInFrame1(0);
            Vec3 orientationInFrame1(0);
            Vec3 locationInFrame2(0);
            Vec3 orientationInFrame2(0);

            if (locBody1Elt != aNode.element_end()) {
                locBody1Elt->getValueAs<Vec3>(locationInFrame1);
            }
            if (orientBody1Elt != aNode.element_end()) {
                orientBody1Elt->getValueAs<Vec3>(orientationInFrame1);
            }
            if (locBody2Elt != aNode.element_end()) {
                locBody2Elt->getValueAs<Vec3>(locationInFrame2);
            }
            if (orientBody2Elt != aNode.element_end()) {
                orientBody2Elt->getValueAs<Vec3>(orientationInFrame2);
            }

            // The value of the connectee name depends on if we need to insert
            // offset frames.
            std::string frame1_connectee_name;
            std::string frame2_connectee_name;

            // now append updated frames to the property list if they are not
            // identity transforms.
            if ((locationInFrame1.norm() > 0.0) ||
                (orientationInFrame1.norm() > 0.0)) {
                frame1_connectee_name = frame1Name + "_offset";
                XMLDocument::addPhysicalOffsetFrame30505_30517(aNode,
                        frame1_connectee_name,
                        frame1Name, locationInFrame1, orientationInFrame1);
            }
            // Prior to 4.0, all bodies lived in the Model's BodySet and
            // Constraints and Forces (current TwoFrameLinker) were always
            // in the ConstraintSet and ForceSet, which means we need to go
            // two levels up (../../)  and into the bodyset to find the
            // Bodies (frames) being linked.
            else {
                frame1_connectee_name =
                        XMLDocument::updateConnecteePath30517("bodyset",
                                                              frame1Name);
            }

            // again for the offset frame on the child
            if ((locationInFrame2.norm() > 0.0) ||
                (orientationInFrame2.norm() > 0.0)) {
                frame2_connectee_name = frame2Name + "_offset";
                XMLDocument::addPhysicalOffsetFrame30505_30517(aNode,
                        frame2_connectee_name,
                        frame2Name, locationInFrame2, orientationInFrame2);
                body2Element->setValue(frame2Name + "_offset");
            } else {
                frame2_connectee_name =
                        XMLDocument::updateConnecteePath30517("bodyset",
                                                              frame2Name);
            }


            // Now we know whether to use the original body_1 and body_2
            // strings or if we should use the offsets.
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                "frame1", frame1_connectee_name);
            XMLDocument::addConnector(aNode, "Connector_PhysicalFrame_",
                "frame2", frame2_connectee_name);
        }
    }
    Super::updateFromXMLNode(aNode, versionNumber);
}


} // end of namespace OpenSim
#endif // OPENSIM_TWO_FRAME_LINKER_H_
