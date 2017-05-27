#ifndef OPENSIM_UNIVERSAL_JOINT_H_
#define OPENSIM_UNIVERSAL_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  UniversalJoint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
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

A class implementing a Universal joint. The underlying implementation
in Simbody is a SimTK::MobilizedBody::Universal.
Universal provides two DoF: rotation about the x axis of the joint frames,
followed by a rotation about the new y axis. The joint is badly behaved when the
second rotation is near 90 degrees. 

\image html universalJoint.gif

@author Tim Dorn
 */
class OSIMSIMULATION_API UniversalJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(UniversalJoint, Joint);

public:
    /** Indices of Coordinates for use as arguments to getCoordinate() and
        updCoordinate().

        <b>C++ example</b>
        \code{.cpp}
        const auto& rx = myUniversalJoint.
                         getCoordinate(UniversalJoint::Coord::Rotation1X);
        \endcode

        <b>Python example</b>
        \code{.py}
        import opensim
        rx = myUniversalJoint.getCoordinate(opensim.UniversalJoint.Coord_Rotation1X)
        \endcode

        <b>Java example</b>
        \code{.java}
        rx = myUniversalJoint.getCoordinate(UniversalJoint.Coord.Rotation1X);
        \endcode

        <b>MATLAB example</b>
        \code{.m}
        rx = myUniversalJoint.get_coordinates(0);
        \endcode
    */
    enum class Coord: unsigned {
        Rotation1X = 0u, ///< 0
        Rotation2Y = 1u  ///< 1
    };

private:
    /** Specify the Coordinates of the UniversalJoint */
    CoordinateIndex rx{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation1X)) };
    CoordinateIndex ry{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation2Y)) };

public:
    /** Use Joint's constructors. @see Joint */
    using Joint::Joint;

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

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
//=============================================================================
};  // END of class UniversalJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_UNIVERSAL_JOINT_H_
