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

// INCLUDE
#include "Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**

A class implementing a Ellipsoid joint. The underlying implementation
in Simbody is a SimTK::MobilizedBody::Ellipsoid. An Ellipsoid joint provides three
mobilities – coordinated rotation and translation along the surface of an ellipsoid
 fixed to the parent body. The ellipsoid surface is determined by an input Vec3 which
describes the ellipsoid radius. Generalized speeds are equal to the computed angular
velocities (\f$\vec{u} = \vec{\omega}\f$), not a differentiation of
position (\f$\vec{u} \neq \dot{\vec{q}}\f$)

\image html ellipsoid.gif

@author Ajay Seth
*/
class OSIMSIMULATION_API EllipsoidJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(EllipsoidJoint, Joint);

public:
    /** Indices of Coordinates for use as arguments to getCoordinate() and
        updCoordinate().

        <b>C++ example</b>
        \code{.cpp}
        const auto& rx = myEllipsoidJoint.
                         getCoordinate(EllipsoidJoint::Coord::Rotation1X);
        \endcode

        <b>Python example</b>
        \code{.py}
        import opensim
        rx = myEllipsoidJoint.getCoordinate(opensim.EllipsoidJoint.Coord_Rotation1X)
        \endcode

        <b>Java example</b>
        \code{.java}
        rx = myEllipsoidJoint.getCoordinate(EllipsoidJoint.Coord.Rotation1X);
        \endcode

        <b>MATLAB example</b>
        \code{.m}
        rx = myEllipsoidJoint.get_coordinates(0);
        \endcode
    */
    enum class Coord: unsigned {
        Rotation1X = 0u, ///< 0
        Rotation2Y = 1u, ///< 1
        Rotation3Z = 2u  ///< 2
    };

private:
    /** Specify the Coordinates of the EllipsoidJoint */
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

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    EllipsoidJoint();
    /** Convenience Joint like Constructor */
    EllipsoidJoint(const std::string&    name,
                   const PhysicalFrame&  parent,
                   const PhysicalFrame&  child,
                   const SimTK::Vec3&    ellipsoidRadii);

    /** Deprecated Joint Constructor*/
    EllipsoidJoint(const std::string&    name,
                   const PhysicalFrame&  parent,
                   const SimTK::Vec3&    locationInParent,
                   const SimTK::Vec3&    orientationInParent,
                   const PhysicalFrame&  child,
                   const SimTK::Vec3&    locationInChild,
                   const SimTK::Vec3&    orientationInChild,
                   const SimTK::Vec3&    ellipsoidRadii);

    //Set properties
    void setEllipsoidRadii(const SimTK::Vec3& radii);

    /** Exposes getCoordinate() method defined in base class (overloaded below).
        @see Joint */
    using Joint::getCoordinate;

    /** Exposes updCoordinate() method defined in base class (overloaded below).
        @see Joint */
    using Joint::updCoordinate;

    /** Get a const reference to a Coordinate associated with this Joint.
        @see Coord */
    const Coordinate& getCoordinate(Coord idx) const {
        return get_coordinates( static_cast<unsigned>(idx) );
    }

    /** Get a writable reference to a Coordinate associated with this Joint.
        @see Coord */
    Coordinate& updCoordinate(Coord idx) {
        return upd_coordinates( static_cast<unsigned>(idx) );
    }

    // SCALE
    void scale(const ScaleSet& aScaleSet) override;

protected:
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    // Visual support in SimTK visualizer
    void generateDecorations(
        bool fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const override;

private:
    void constructProperties();

//=============================================================================
};  // END of class EllipsoidJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ELLIPSOID_JOINT_H_
