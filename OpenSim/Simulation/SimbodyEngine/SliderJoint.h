#ifndef OPENSIM_SLIDER_JOINT_H_
#define OPENSIM_SLIDER_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  SliderJoint.h                           *
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
A class implementing a Slider joint. The underlying implementation in Simbody
is a SimTK::MobilizedBody::Slider. The Slider provides a single coordinate
along the common X-axis of the parent and child joint frames.

\image html sliderJoint.gif

@author Ajay Seth
*/
class OSIMSIMULATION_API SliderJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(SliderJoint, Joint);

public:
    /** Index of Coordinate for use as an argument to getCoordinate() and
        updCoordinate().

        <b>C++ example</b>
        \code{.cpp}
        const auto& tx = mySliderJoint.
                         getCoordinate(SliderJoint::Coord::TranslationX);
        \endcode

        <b>Python example</b>
        \code{.py}
        import opensim
        tx = mySliderJoint.getCoordinate(opensim.SliderJoint.Coord_TranslationX)
        \endcode

        <b>Java example</b>
        \code{.java}
        tx = mySliderJoint.getCoordinate(SliderJoint.Coord.TranslationX);
        \endcode

        <b>MATLAB example</b>
        \code{.m}
        tx = mySliderJoint.get_coordinates(0);
        \endcode
    */
    enum class Coord: unsigned {
        TranslationX = 0u ///< 0
    };

private:
    /** Specify the Coordinate of the SliderJoint */
    CoordinateIndex tx{ constructCoordinate(Coordinate::MotionType::Translational,
                                    static_cast<unsigned>(Coord::TranslationX)) };

public:
    /** Use Joint's constructors. @see Joint */
    using Joint::Joint;

    /** Exposes getCoordinate() method defined in base class (overloaded below).
        @see Joint */
    using Joint::getCoordinate;

    /** Exposes updCoordinate() method defined in base class (overloaded below).
        @see Joint */
    using Joint::updCoordinate;

    /** Get a const reference to the Coordinate associated with this Joint.
        @see Coord */
    const Coordinate& getCoordinate(Coord idx) const {
        return get_coordinates( static_cast<unsigned>(idx) );
    }

    /** Get a writable reference to the Coordinate associated with this Joint.
        @see Coord */
    Coordinate& updCoordinate(Coord idx) {
        return upd_coordinates( static_cast<unsigned>(idx) );
    }

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
//=============================================================================
};  // END of class SliderJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_SLIDER_JOINT_H_
