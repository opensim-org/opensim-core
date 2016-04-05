#ifndef OPENSIM_POINT_H_
#define OPENSIM_POINT_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Point.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A Point is an OpenSim representation for any location in space. Points are 
 * intended to locate physical structures (such as origins of joints
 * and points of muscle attachments) as well as embody the results of spatial
 * calculations. For example, if your system involves contact, you can define
 * a Point that describes the location of the center-of-pressure as one
 * element rolls over another.
 *
 * A Point provides its location in the Ground frame as a function of the 
 * Model's (SimTK::MultibodySystem's) state.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Point : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Point, ModelComponent);

public:
    OpenSim_DECLARE_OUTPUT(location, SimTK::Vec3, getLocationInGround,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(velocity, SimTK::Vec3, calcVelocityInGround,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(acceleration, SimTK::Vec3, calcAccelerationInGround,
        SimTK::Stage::Acceleration);
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Point();

    virtual ~Point() {};

    /** @name Spatial Operations for Point
    Convenient spatial operations with Points.*/
    /**@{**/

    /**
    Get the location of this Point relative to the Ground frame.
    @param state       The state applied to the model when determining the
                       location of the Point.
    @return location   The location of the point expressed in the Ground frame
    */
    const SimTK::Vec3& getLocationInGround(const SimTK::State& state) const;

    // End of Point's Spatial Operations
    ///@}

protected:
    /** @name Point Extension methods.
    Concrete Point types must override these methods. */
    /**@{**/

    /** Calculate the location of this Point with respect to and expressed in
        ground as a function of the state. */
    virtual SimTK::Vec3
        calcLocationInGround(const SimTK::State& state) const = 0;
    /** Calculate the velocity of this Point with respect to and expressed in
        ground as a function of the state. */
    virtual SimTK::Vec3
        calcVelocityInGround(const SimTK::State& state) const = 0;
    /** Calculate the acceleration of this Point with respect to and expressed
        in ground as a function of the state. */
    virtual SimTK::Vec3
        calcAccelerationInGround(const SimTK::State& state) const = 0;

    /**@}**/

    /** @name Component Extension methods.
        Point types override these Component methods. */
    /**@{**/
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeTopology(SimTK::State& s) const override;
    /**@}**/

private:

    mutable SimTK::CacheEntryIndex _groundLocationIndex;

//=============================================================================
};  // END of class Point
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_POINT_H_


