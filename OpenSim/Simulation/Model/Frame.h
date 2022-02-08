#ifndef OPENSIM_FRAME_H_
#define OPENSIM_FRAME_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Frame.h                             *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Geometry.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A Frame is an OpenSim representation of a reference frame. It consists of
 * a right-handed set of three orthogonal axes and an origin point. Frames are
 * intended to provide convenient reference frames for locating physical
 * structures (such as joints and muscle attachments) as well as provide a
 * convenient basis for performing spatial calculations. For example, if your
 * system involves contact, you might define a Frame that is aligned with the
 * normal direction of a contact surface and whose origin is at the
 * center-of-pressure.
 *
 * Every Frame is capable of providing its SimTK::Transform (translation of
 * the origin and the orientation of its axes) in the Ground frame as a
 * function of the Model's (SimTK::MultibodySystem's) state.
 *
 * The Frame class also provides convenience methods for re-expressing vectors
 * from one Frame to another.
 *
 * As already noted, Frames are useful for locating physical structures such as
 * bodies, their joints, and the locations where constraints can be connected
 * and forces can be applied. It is perhaps less evident that Frames can be
 * extremely useful for relating a multitude of reference frames together to
 * form chains and trees. For example, a Frame to specify muscle attachments
 * (M) and a Frame to specify a joint location (J) could themselves be
 * specified in an anatomical Frame (A) defined by bony landmarks identified
 * by surface markers or tagged on CT or MRI images. The body (B), to which the
 * anatomical frame (A) is attached, can be thought of as a "Base" frame or a
 * root of a tree from which a set of descendant frames arise. In particular, a
 * Base frame and all its descendants have the property that they share the
 * same angular velocity, since they are affixed to the same underlying Frame
 * (in this case a Body).
 * <pre>
 *         M---muscle points
 *        /
 *   B---A
 *        \
 *         J---joint axes
 * </pre>
 * Therefore, a useful concept is that of a Base frame, and a Frame can always
 * provide a Base frame. If a Frame is not affixed to another frame, its Base
 * frame is itself.
 *
 * @see SimTK::Transform
 *
 * @author Matt DeMers
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Frame : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Frame, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(frame_geometry, FrameGeometry,
        "The geometry used to display the axes of this Frame.");
    OpenSim_DECLARE_LIST_PROPERTY(attached_geometry, Geometry,
        "List of geometry attached to this Frame. Note, the geometry "
        "are treated as fixed to the frame and they share the transform "
        "of the frame when visualized");
//=============================================================================
// OUTPUTS
//=============================================================================
    OpenSim_DECLARE_OUTPUT(position, SimTK::Vec3, getPositionInGround,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(rotation, SimTK::Rotation, getRotationInGround,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(transform, SimTK::Transform, getTransformInGround,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(velocity, SimTK::SpatialVec, getVelocityInGround,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(angular_velocity, SimTK::Vec3, getAngularVelocityInGround,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(linear_velocity, SimTK::Vec3, getLinearVelocityInGround,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(acceleration, SimTK::SpatialVec, getAccelerationInGround,
        SimTK::Stage::Acceleration);
    OpenSim_DECLARE_OUTPUT(angular_acceleration, SimTK::Vec3, getAngularAccelerationInGround,
        SimTK::Stage::Acceleration);
    OpenSim_DECLARE_OUTPUT(linear_acceleration, SimTK::Vec3, getLinearAccelerationInGround,
        SimTK::Stage::Acceleration);

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Frame();
    virtual ~Frame() {};

    /** @name Spatial Operations for Frames
    These methods allow access to the frame's transform and some convenient
    operations that could be performed with this transform.*/
    /**@{**/

    /**
    Get the transform of this frame (F) relative to the ground frame (G).
    It transforms quantities expressed in F into quantities expressed
    in G. This is mathematically stated as:
        vec_G = X_GF*vec_F ,
    where X_GF is the transform returned by getTransformInGround.

    @param state       The state applied to the model when determining the
                       transform.
    @return transform  The transform between this frame and the ground frame
    */
    const SimTK::Transform&
        getTransformInGround(const SimTK::State& state) const;

    /** The spatial velocity V_GF {omega; v} of this Frame, measured with
        respect to and expressed in the ground frame. It can be used to compute
        the velocity of any stationary point on F, located at r_F (Vec3), in
        ground, G, as:
            v_G = V_GF[1] + SimTK::cross(V_GF[0], r_F);
        Is only valid at Stage::Velocity or higher. */
    const SimTK::SpatialVec&
        getVelocityInGround(const SimTK::State& state) const;

    /** The angular velocity of this Frame, measured with respect to and
        expressed in the ground frame (i.e., the first half of the SpatialVec
        returned by getVelocityInGround()). */
    const SimTK::Vec3&
        getAngularVelocityInGround(const SimTK::State& state) const;

    /** The linear velocity of this Frame, measured with respect to and
        expressed in the ground frame (i.e., the second half of the SpatialVec
        returned by getVelocityInGround()). */
    const SimTK::Vec3&
        getLinearVelocityInGround(const SimTK::State& state) const;

    /** The spatial acceleration A_GF {alpha; a} of this Frame, measured with
        respect to and expressed in the ground frame. It can also be used to
        compute the acceleration of any stationary point on F, located at r_F
        (Vec3), in ground, G, as:
            a_G = A_GF[1] + SimTK::cross(A_GF[0], r_F) + 
                  SimTK::cross(V_GF[0], SimTK::cross(V_GF[0], r_F));
        Is only valid at Stage::Acceleration or higher. */
    const SimTK::SpatialVec&
        getAccelerationInGround(const SimTK::State& state) const;

    /** The angular acceleration of this Frame, measured with respect to and
        expressed in the ground frame (i.e., the first half of the SpatialVec
        returned by getAccelerationInGround()). */
    const SimTK::Vec3&
        getAngularAccelerationInGround(const SimTK::State& state) const;

    /** The linear acceleration of this Frame, measured with respect to and
        expressed in the ground frame (i.e., the second half of the SpatialVec
        returned by getAccelerationInGround()). */
    const SimTK::Vec3&
        getLinearAccelerationInGround(const SimTK::State& state) const;


    /**
    Find the transform that describes this frame (F) relative to another
    frame (A). It transforms quantities expressed in F to quantities expressed
    in A. This is mathematically stated as:
        vec_A = X_AF*vec_F ,
    where X_AF is the transform returned by this method.

    @param state       The state applied to the model when determining the
                       transform.
    @param otherFrame  a second frame
    @return transform  The transform between this frame and otherFrame
    */
    SimTK::Transform findTransformBetween(const SimTK::State& state,
                                          const Frame& otherFrame) const;

    /**
    Take a vector expressed in this frame (F) and re-express the same vector
    in another frame (A). This re-expression accounts for the difference
    in orientation between the frames. This is mathematically stated as:
        vec_A = R_AF*vec_F
    which does not translate the vector. This is intended to re-express
    physical vector quantities such as a frame's angular velocity or an
    applied force, from one frame to another without changing the physical
    quantity. If you have a position vector and want to change the point from
    which the position is measured, you want findStationLocationInAnotherFrame().

    @param state       The state of the model.
    @param vec_F       The vector to be re-expressed.
    @param otherFrame  The frame in which the vector will be re-expressed
    @return vec_A      The expression of the vector in otherFrame.
    */
    SimTK::Vec3 expressVectorInAnotherFrame(const SimTK::State& state,
                        const SimTK::Vec3& vec_F,
                        const Frame& otherFrame) const;

    /**
    Take a vector in this frame (F) and re-express the same vector
    in Ground (G). This method is equivalent to expressVectorInAnotherFrame()
    where the "other Frame" is always Ground.
    @param state       The state of the model.
    @param vec_F       The vector to be re-expressed.
    @return vec_G      The expression of the vector in Ground.
    */
    SimTK::Vec3 expressVectorInGround(const SimTK::State& state,
                        const SimTK::Vec3& vec_F) const;

    /**
    Take a station located and expressed in this frame (F) and determine
    its location relative to and expressed in another frame (A). The transform
    accounts for the difference in orientation and translation between the 
    frames.
    This is mathematically stated as: 
        loc_A = X_AF*station_F

    @param state       The state of the model.
    @param station_F   The position Vec3 from frame F's origin to the station.
    @param otherFrame  The frame (A) in which the station's location 
                       will be relative to and expressed.
    @return loc_A      The location of the station in another frame (A).
    */
    SimTK::Vec3 findStationLocationInAnotherFrame(const SimTK::State& state, 
                    const SimTK::Vec3& station_F, const Frame& otherFrame) const;

    /**
    Take a station located and expressed in this frame (F) and determine
    its location relative to and expressed in Ground (G). This method is
    equivalent to findStationLocationInAnotherFrame() where the "other Frame" is
    always Ground.

    Note that if you have added an OpenSim::Station, you should use the
    Station's %getLocationInGround() method instead.

    @param state       The state of the model.
    @param station_F   The position Vec3 from frame F's origin to the station.
    @return loc_G      The location of the station in Ground.
    */
    SimTK::Vec3 findStationLocationInGround(const SimTK::State& state,
                    const SimTK::Vec3& station_F) const;

    /**
    Take a station located and expressed in this frame (F) and determine
    its velocity relative to and expressed in Ground (G).

    Note that if you have added an OpenSim::Station, you should use the
    Station's %getVelocityInGround() method instead.

    @param state       The state of the model.
    @param station_F   The position Vec3 from frame F's origin to the station.
    @return vel_G      The velocity of the station in Ground.
    */
    SimTK::Vec3 findStationVelocityInGround(const SimTK::State& state,
        const SimTK::Vec3& station_F) const;

    /**
    Take a station located and expressed in this frame (F) and determine
    its acceleration relative to and expressed in Ground (G).

    Note that if you have added an OpenSim::Station, you should use the
    Station's %getAccelerationInGround() method instead.

    @param state       The state of the model.
    @param station_F   The position Vec3 from frame F's origin to the station.
    @return acc_G      The acceleration of the station in Ground.
    */
    SimTK::Vec3 findStationAccelerationInGround(const SimTK::State& state,
        const SimTK::Vec3& station_F) const;

    /**@}**/

    /** @name Advanced: A Frame's Base Frame and Transform 
    A base Frame is the most ancestral Frame (itself, its parent, 
    grandparent, great-grandparent, etc, down the family tree)
    whose angular velocity is identical to this Frame. That is they belong to
    the same rigid spatial entity. For example, anatomical frames may
    be used to identify points of interest (muscle attachments) and joint
    connections on a body in a convenient way, but their movement is dictated
    by the body.  That body, in this case, would be a base frame for any of the
    anatomical frames attached to the body including frames subsequently
    attached to the anatomical frames and so on.
    */
    ///@{
    /** 
    Find this Frame's base Frame. See the "Advanced" note, above.

    @return baseFrame     The Frame that is the base for this Frame.
    */
    const Frame& findBaseFrame() const;

    /**
    Find the equivalent Transform of this Frame (F) in its base (B) Frame.
    That is find X_BF, such that vecB = X_BF*vecF
    For a Frame that is itself a base, this returns the identity Transform.
    @return X_BF     The Transform of F in B
    */
    SimTK::Transform findTransformInBaseFrame() const;

    /** Accessor for position of the origin of the Frame in Ground. */
    SimTK::Vec3 getPositionInGround(const SimTK::State& state) const {
        return getTransformInGround(state).p();
    };

    /** Accessor for Rotation matrix of the Frame in Ground. */
    SimTK::Rotation_<double> getRotationInGround(const SimTK::State& state) const {
        return getTransformInGround(state).R();
    };

    // End of Base Frame and Transform accessors
    ///@}

    /** Attach Geometry to this Frame and have this Frame take ownership of
        it by adding it to this Frame's \<attached_geometry\> property list.
        The Geometry is treated as being fixed to this Frame such that the
        transform used to position the Geometry is that of this Frame. */
    void attachGeometry(OpenSim::Geometry* geom);

    void scaleAttachedGeometry(const SimTK::Vec3& scaleFactors);

    /** Scales Geometry components that reside in the Frame's
        `attached_geometry` list property. Note that Geometry residing elsewhere
        (e.g., in the `components` list property of a Frame or any other
        Component) will not be scaled. Note also that ContactGeometry derives
        from ModelComponent so the classes derived from ContactGeometry are
        responsible for scaling themselves. (However, `scale()` is not currently
        implemented on ContactGeometry or classes derived therefrom so they will
        not scale with the Model.) */
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

protected:
    /** @name Extension of calculations of Frame kinematics.
    Concrete Frame types must override these calculations.
    Results of the calculations are cached by the Frame and made accessible
    via the corresponding getTransformInGround, getVelocityInGround,
    getAccelerationInGround public methods (above). */
    /**@{**/

    /** Calculate the transform related to this Frame with respect to ground.
    This method returns the transform X_GF, converting quantities expressed
    in this frame, F, to quantities expressed in the ground, G, frame.
    This is mathematically stated as:
        vec_G = X_GF*vec_F  */
    virtual SimTK::Transform
        calcTransformInGround(const SimTK::State& state) const = 0;

    /** The spatial velocity {omega; v} of this Frame in ground. */
    virtual SimTK::SpatialVec
        calcVelocityInGround(const SimTK::State& state) const = 0;

    /** The spatial acceleration {alpha; a} for this Frame in ground */
    virtual SimTK::SpatialVec
        calcAccelerationInGround(const SimTK::State& state) const = 0;
    /**@}**/

    /** @name Component Extension methods.
    Frame types override these Component methods. */
    /**@{**/
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    /**@}**/

private:
    /** Extend how concrete Frame determines its base Frame. */
    virtual const Frame& extendFindBaseFrame() const = 0;
    virtual SimTK::Transform extendFindTransformInBaseFrame() const = 0;

    mutable CacheVariable<SimTK::Transform> _transformCV;
    mutable CacheVariable<SimTK::SpatialVec> _velocityCV;
    mutable CacheVariable<SimTK::SpatialVec> _accelerationCV;
//=============================================================================
};  // END of class Frame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FRAME_H_


