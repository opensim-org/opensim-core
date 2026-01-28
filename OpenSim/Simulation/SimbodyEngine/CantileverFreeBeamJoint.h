#ifndef OPENSIM_CANTILEVER_FREE_BEAM_JOINT_H_
#define OPENSIM_CANTILEVER_FREE_BEAM_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim:  CantileverFreeBeamJoint.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include "Joint.h"

namespace OpenSim {

class Model;

/**
 * A class implementing a CantileverFreeBeam joint. The underlying
 * implementation in Simbody is a SimTK::MobilizedBody::CantileverFreeBeam. For
 * full details on the Simbody implementation, see the Simbody API
 * documentation. A condensed version of that documentation is provided here.
 *
 * This joint has only three generalized coordinates (all rotational), but the
 * parent frame translates relative to the child frame as a specified function
 * of rotational generalized coordinates q₀, q₁, q₂. The specified function is
 * based on the deflection shape of a cantilever-free beam, whose free end is
 * subjected to a transverse point load. Although this shape is inspired by a
 * cantilever-free beam, this joint does not actually model a beam, in that
 * there is no elastic restoring forces/torques when q₀ or q₁ or q₂ ≠ 0. In the
 * undeformed state (when q₀ = q₁ = q₂ = 0), the position from Fo (the child
 * frame F's origin) to Mo (the parent frame M's origin) is L Fz.
 *
 * \image html cantileverFreeBeamJoint.gif
 *
 * The generalized coordinates q are the same as for a Gimbal joint, that is,
 * an X-Y-Z body-fixed Euler sequence. The three generalized speeds u for
 * this joint are also the same as for a Gimbal joint: the time derivatives
 * of the generalized coordinates, that is, uᵢ = q̇ᵢ (i.e., uᵢ = qdotᵢ for
 * i = 0, 1, 2).
 *
 * The first two rotations, q₀ and q₁, induce translations according to the beam
 * deflection formula for a cantilever beam under a transverse point load
 * applied at the end of the beam. The change in the beam's endpoint position in
 * the Fz direction is governed by apparent shortening of a beam due to bending.
 * Denoting the position vector from Fo (frame F's origin) to Mo (frame M's
 * origin), expressed in frame F as p₀ Fx + p₁ Fy + p₂ Fz, then p₀, p₁, and p₂
 * are given by:
 *
 * p₀ = 2⁄3 q₁ L
 *
 * p₁ = −2⁄3 q₀ L
 *
 * p₂ = L − 4⁄15 (q₀² + q₁²) L
 *
 * The full derivation for these expressions can be found in the Simbody API
 * documentation for SimTK::MobilizedBody::CantileverFreeBeam. The third
 * rotation, q₂, induces a rotation about the parent frame's Z axis, which is
 * always tangent to the beam at the beam's endpoint.
 *
 * Note that while the endpoint shortens in the Fz direction, the total length
 * of the beam actually lengthens slightly (about 3% for a rotation of π/4
 * radians about X or Y), since the beam formula for an end-loaded cantilever
 * beam assumes finite deflections and is derived from a simplified form of the
 * beam curvature term in the Euler-Bernoulli beam equation. However, since the
 * primary utility of this joint is to provide a lightweight way for
 * modeling flexible structures (e.g., the bending of the spinal column in a
 * human or animal skeleton), and not necessarily to accurately model large beam
 * deflections, the beam lengthening can be accounted for with appropriate
 * modeling adjustments.
 *
 * While this joint provides arbitrary orientation, the Euler angle derivatives
 * are singular when q₁ (the middle rotation) is near ±π/2 radians. That means
 * you should not attempt to do any dynamics in that configuration.
 */
class OSIMSIMULATION_API CantileverFreeBeamJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(CantileverFreeBeamJoint, Joint);

public:
    /**
     * Indices of Coordinates for use as arguments to getCoordinate() and
     * updCoordinate().
     *
     *  <b>C++ example</b>
     *  \code{.cpp}
     *  const auto& rx = myCantileverFreeBeamJoint.getCoordinate(
     *      CantileverFreeBeamJoint::Coord::RotationX);
     *  \endcode
     *
     *  <b>Python example</b>
     *  \code{.py}
     *  import opensim
     *  rx = myCantileverFreeBeamJoint.getCoordinate(
     *      opensim.CantileverFreeBeamJoint.Coord_RotationX)
     *  \endcode
     *
     *  <b>Java example</b>
     *  \code{.java}
     *  rx = myCantileverFreeBeamJoint.getCoordinate(
     *      CantileverFreeBeamJoint.Coord.RotationX);
     *  \endcode
     *
     *  <b>MATLAB example</b>
     *  \code{.m}
     *  rx = myCantileverFreeBeamJoint.get_coordinates(0);
     *  \endcode
    */
    enum class Coord: unsigned {
        Rotation1X = 0u, ///< 0
        Rotation2Y = 1u, ///< 1
        Rotation3Z = 2u  ///< 2
    };

private:
    /**
     * Specify the Coordinates of the CantileverFreeBeamJoint.
     */
    CoordinateIndex rx{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation1X)) };
    CoordinateIndex ry{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation2Y)) };
    CoordinateIndex rz{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation3Z)) };

public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(beam_length, SimTK::Real,
        "The length of the beam (default: 1.0).");

//=============================================================================
// METHODS
//=============================================================================

    /**
     * Default constructor.
     */
    CantileverFreeBeamJoint();

    /**
     * Convenience constructor. Create a CantileverFreeBeamJoint by specifying
     * the parent and child frames and the beam length.
     *
     * @param[in] name   the name associated with this joint (should be unique
     *                   from other joints in the same model)
     * @param[in] parent the parent PhysicalFrame to which this joint connects
     * @param[in] child  the child PhysicalFrame to which this joint connects
     * @param[in] beamLength the length of the beam
     */
    CantileverFreeBeamJoint(const std::string& name,
            const PhysicalFrame& parent, const PhysicalFrame& child,
            const SimTK::Real& beamLength);

    /**
     * Convenience constructor. Construct a CantileverFreeBeamJoint where
     * the parent and child frames are specified as well as the location and
     * orientation of the parent and child frames in their respective physical
     * frames. Also, specify the beam length.
     *
     * @param[in] name    the name associated with this joint (should be unique
     *                    from other joints in the same model)
     * @param[in] parent  the parent PhysicalFrame to which this joint connects
     * @param[in] locationInParent     Vec3 of the location of the joint in the
     *                                 parent frame.
     * @param[in] orientationInParent  Vec3 of the XYZ body-fixed Euler angles
     *                                 of the joint frame orientation in the
     *                                 parent frame.
     * @param[in] child  the child PhysicalFrame to which this joint connects
     * @param[in] locationInChild      Vec3 of the location of the joint in the
     *                                 child frame.
     * @param[in] orientationInChild   Vec3 of the XYZ body-fixed Euler angles
     *                                 of the joint frame orientation in the
     *                                 child frame.
     * @param[in] beamLength the length of the beam
    */
    CantileverFreeBeamJoint(const std::string& name,
            const PhysicalFrame& parent, const SimTK::Vec3& locationInParent,
            const SimTK::Vec3& orientationInParent, const PhysicalFrame& child,
            const SimTK::Vec3& locationInChild,
            const SimTK::Vec3& orientationInChild,
            const SimTK::Real& beamLength);

    /**
     * Use Joint's constructors.
     * @see Joint
     */
    using Joint::Joint;

    /**
     * Exposes getCoordinate() method defined in base class (overloaded below).
     * @see Joint
     */
    using Joint::getCoordinate;

    /**
     * Exposes updCoordinate() method defined in base class (overloaded below).
     * @see Joint
     */
    using Joint::updCoordinate;

    /**
     * Get a const reference to a Coordinate associated with this Joint.
     * @see Coord
     */
    const Coordinate& getCoordinate(Coord idx) const {
        return get_coordinates( static_cast<unsigned>(idx) );
    }

    /**
     * Get a writable reference to a Coordinate associated with this Joint.
     * @see Coord
     */
    Coordinate& updCoordinate(Coord idx) {
        return upd_coordinates( static_cast<unsigned>(idx) );
    }

protected:
    // MODEL COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // COMPONENT INTERFACE
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_CANTILEVER_FREE_BEAM_JOINT_H_
