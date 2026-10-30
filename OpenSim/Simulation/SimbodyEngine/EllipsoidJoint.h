#ifndef OPENSIM_ELLIPSOID_JOINT_H_
#define OPENSIM_ELLIPSOID_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  EllipsoidJoint.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2026 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 * Contributor(s): Nicholas Bianco                                            *
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

/**
 * A class implementing a Ellipsoid joint. The underlying implementation
 * in Simbody is a SimTK::MobilizedBody::Ellipsoid.
 *
 * An Ellipsoid joint provides three mobilities: coordinated rotation and
 * translation along the surface of an ellipsoid fixed to the parent body. The
 * three generalized speeds u for this mobilizer are also the same as a
 * BallJoint, that is the three measure numbers of the angular velocity vector
 * w_FM, the relative angular velocity of the joint's child frame (M) in the
 * joint's parent frame (F), expressed in the F frame. Note that qdot != u for
 * this joint.
 */
class OSIMSIMULATION_API EllipsoidJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(EllipsoidJoint, Joint);

public:
    /**
     * Indices of Coordinates for use as arguments to getCoordinate() and
     * updCoordinate().
     *
     * <b>C++ example</b>
     * \code{.cpp}
     * const auto& rx = myEllipsoidJoint.
     *                  getCoordinate(EllipsoidJoint::Coord::Rotation1X);
     * \endcode
     *
     * <b>Python example</b>
     * \code{.py}
     * import opensim as osim
     * rx = myEllipsoidJoint.getCoordinate(osim.EllipsoidJoint.Coord_Rotation1X)
     * \endcode
     *
     * <b>Java example</b>
     * \code{.java}
     * rx = myEllipsoidJoint.getCoordinate(EllipsoidJoint.Coord.Rotation1X);
     * \endcode
     *
     * <b>MATLAB example</b>
     * \code{.m}
     * rx = myEllipsoidJoint.get_coordinates(0);
     * \endcode
     */
    enum class Coord : unsigned {
        Rotation1X = 0u, ///< 0
        Rotation2Y = 1u, ///< 1
        Rotation3Z = 2u  ///< 2
    };

    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "Default appearance attributes for this EllipsoidJoint");

private:
    /**
     * Specify the Coordinates of the EllipsoidJoint.
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
    OpenSim_DECLARE_PROPERTY(radii_x_y_z, SimTK::Vec3,
        "Radii of the ellipsoid fixed to the parent frame, "
        "specified as a Vec3(rX, rY, rZ).");

//==============================================================================
// METHODS
//==============================================================================

    /**
     * Default constructor.
     */
    EllipsoidJoint();

    /**
     * Convenience constructor. Create an EllipsoidJoint by specifying
     * the parent and child frames and the ellipsoid radii.
     *
     * @param[in] name   the name associated with this joint (should be unique
     *                   from other joints in the same model)
     * @param[in] parent the parent PhysicalFrame to which this joint connects
     * @param[in] child  the child PhysicalFrame to which this joint connects
     * @param[in] radii  the radii of the ellipsoid
     */
    EllipsoidJoint(const std::string& name, const PhysicalFrame& parent,
            const PhysicalFrame& child, const SimTK::Vec3& radii);

    /**
     * Convenience constructor. Construct an EllipsoidJoint where the parent and
     * child frames are specified as well as the location and orientation of the
     * parent and child frames in their respective physical frames. Also,
     * specify the ellipsoid radii.
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
     * @param[in] radii the radii of the ellipsoid
     */
    EllipsoidJoint(const std::string& name, const PhysicalFrame& parent,
            const SimTK::Vec3& locationInParent,
            const SimTK::Vec3& orientationInParent, const PhysicalFrame& child,
            const SimTK::Vec3& locationInChild,
            const SimTK::Vec3& orientationInChild, const SimTK::Vec3& radii);

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
        return get_coordinates(static_cast<unsigned>(idx));
    }

    /**
     * Get a writable reference to a Coordinate associated with this Joint.
     * @see Coord
     */
    Coordinate& updCoordinate(Coord idx) {
        return upd_coordinates(static_cast<unsigned>(idx));
    }

    /**
     * Set the ellipsoid radii.
     *
     * This method internally sets the `radii_x_y_z` property value. Therefore,
     * the change will only take effect in the joint after the system is
     * recreated (e.g., `Model::initSystem()`).
     */
    void setEllipsoidRadii(const SimTK::Vec3& radii);

    /**
     * Set whether the ellipsoid surface is drawn by generateDecorations().
     */
    void setEllipsoidVisible(bool visible) {
        upd_Appearance().set_visible(visible);
    };

    // SCALING INTERFACE
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;

protected:
    // MODEL COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    // COMPONENT INTERFACE
    void generateDecorations(bool, const ModelDisplayHints&,
        const SimTK::State&,
        SimTK::Array_<SimTK::DecorativeGeometry>&) const override;

private:
    void constructProperties();
};

} // namespace OpenSim

#endif // OPENSIM_ELLIPSOID_JOINT_H_
