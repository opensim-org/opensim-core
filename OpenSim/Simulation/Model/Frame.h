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
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>

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
    where X_GF is the transform returned by getGroundTransform.

    @param state       The state applied to the model when determining the
                       transform.
    @return transform  The transform between this frame and the ground frame
    */
    const SimTK::Transform& getGroundTransform(const SimTK::State& state) const;

    /**
    Find the transform that describes this frame (F) relative to another
    frame (A). It transforms quantities expressed in F to quantities expressed
    in A. This is mathematically stated as:
    This is mathematically stated as:
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
    which does not translate the vector. This is intended to reexpress
    physical vector quantities such as a frame's angular velocity or an
    applied force, from one frame to another without changing the physical
    quantity. If you have a position vector and want to change the point from
    which the position is measured, you want findLocationInAnotherFrame().

    @param state       The state of the model.
    @param vec_F       The vector to be re-expressed.
    @param otherFrame  The frame in which the vector will be re-expressed
    @return vec_A; the expression of the vector in otherFrame.
    */
    SimTK::Vec3 expressVectorInAnotherFrame(const SimTK::State& state,
                        const SimTK::Vec3& vec_F,
                        const Frame& otherFrame) const;

    /**
    Take a point located and expressed in this frame (F) and determine
    its location expressed in another frame (A). The transform accounts for
    the difference in orientation and translation between the frames.
    This is mathematically stated as: 
        point_A = X_AF*point_F

    @param state       The state of the model.
    @param point_F     The point to be re-expressed.
    @param otherFrame  The frame in which the point will be re-expressed
    @return point_A    The re-expression of the point in another frame.
    */
    SimTK::Vec3 findLocationInAnotherFrame(const SimTK::State& state, 
                    const SimTK::Vec3& point_F, const Frame& otherFrame) const;
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
    Find this Frame's base Frame.

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

    // End of Base Frame and Transform accessors
    ///@}

    /** Add a Mesh specified by file name to the list of Geometry owned by the Frame.
    Scale defaults to 1.0 but can be changed on the call line. For convenience 
    */
    void addMeshGeometry(const std::string &aGeometryFileName, const SimTK::Vec3 scale = SimTK::Vec3(1));

protected:
    /** @name Component Extension methods.
        Frame types override these Component methods. */
    /**@{**/
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeTopology(SimTK::State& s) const override;

    /// override default extendAddGeometry to set Frame to this Object
    void extendAddGeometry(OpenSim::Geometry& geom) override;

    /**@}**/

private:
    /** @name Extension methods.
        Concrete Frame types must override these methods. */
    /**@{**/

    /** Calculate the transform related to this Frame with respect to ground.
    This method returns the transform X_GF, converting quantities expressed
    in this frame, F, to quantities expressed in the ground, G, frame.
    This is mathematically stated as:
        vec_G = X_GF*vec_F  */
    virtual SimTK::Transform
        calcGroundTransform(const SimTK::State& state) const = 0;

    /** Extend how concrete Frame determines its base Frame. */
    virtual const Frame& extendFindBaseFrame() const = 0;
    virtual SimTK::Transform extendFindTransformInBaseFrame() const = 0;
    /**@}**/

    mutable SimTK::CacheEntryIndex groundTransformIndex;

//=============================================================================
};  // END of class Frame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FRAME_H_


