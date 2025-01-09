#ifndef OPENSIM_GIMBAL_JOINT_H_
#define OPENSIM_GIMBAL_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  GimbalJoint.h                            *
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

/**
A class implementing a Gimbal joint. The underlying implementation Simbody is a
SimTK::MobilizedBody::Gimbal. The opensim Gimbal joint implementation uses a 
 X-Y-Z body fixed Euler sequence for generalized coordinates calculation.
Gimbal joints have a singularity when Y is near \f$\frac{\pi}{2}\f$.
Generalized speeds are equal to the Euler angle derivatives  (\f$\vec{u} = \dot{\vec{q}}\f$)

\image html gimbalJoint.gif

@author Tim Dorn
@author Ajay Seth
*/
class OSIMSIMULATION_API GimbalJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(GimbalJoint, Joint);

public:
    /** Indices of Coordinates for use as arguments to getCoordinate() and
        updCoordinate().

        <b>C++ example</b>
        \code{.cpp}
        const auto& rx = myGimbalJoint.
                         getCoordinate(GimbalJoint::Coord::Rotation1X);
        \endcode

        <b>Python example</b>
        \code{.py}
        import opensim
        rx = myGimbalJoint.getCoordinate(opensim.GimbalJoint.Coord_Rotation1X)
        \endcode

        <b>Java example</b>
        \code{.java}
        rx = myGimbalJoint.getCoordinate(GimbalJoint.Coord.Rotation1X);
        \endcode

        <b>MATLAB example</b>
        \code{.m}
        rx = myGimbalJoint.get_coordinates(0);
        \endcode
    */
    enum class Coord: unsigned {
        Rotation1X = 0u, ///< 0
        Rotation2Y = 1u, ///< 1
        Rotation3Z = 2u  ///< 2
    };

private:
    /** Specify the Coordinates of the GimbalJoint */
    CoordinateIndex rx{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation1X)) };
    CoordinateIndex ry{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation2Y)) };
    CoordinateIndex rz{ constructCoordinate(Coordinate::MotionType::Rotational,
                                   static_cast<unsigned>(Coord::Rotation3Z)) };

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
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

//=============================================================================
};  // END of class GimbalJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_GIMBAL_JOINT_H_
