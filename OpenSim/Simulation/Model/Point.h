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
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class Frame;
//=============================================================================
//=============================================================================
/**
 * A Point is an OpenSim abstraction for any location in space. Points
 * are intended to locate physical structures (such as points of constraints
 * and points of muscle attachments) as well as embody the results of spatial
 * calculations. For example, if your system involves contact, you can define
 * a Point that describes the location of the center-of-pressure as one
 * element rolls over another.
 *
 * A Point provides its location, velocity and acceleration in the Ground frame
 * as a function of the Model's (SimTK::MultibodySystem's) state, which is
 * accessible when the state has been realized to the corresponding
 * SimTK::Stage (i.e. Position, Velocity and Acceleration). 
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Point : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Point, ModelComponent);

public:
    OpenSim_DECLARE_OUTPUT(location, SimTK::Vec3, getLocationInGround,
        SimTK::Stage::Position);
    OpenSim_DECLARE_OUTPUT(velocity, SimTK::Vec3, getVelocityInGround,
        SimTK::Stage::Velocity);
    OpenSim_DECLARE_OUTPUT(acceleration, SimTK::Vec3, getAccelerationInGround,
        SimTK::Stage::Acceleration);
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Point();

    virtual ~Point() {};

    /** @name The three dimensional kinematics of the Point.
    Convenient access to the kinematics of a Point expressed in Ground.*/
    /**@{**/

    /**
    Get the location, r_GP, of this Point, P, relative to and expressed in 
    the Ground. Point position only valid at Stage::Position or higher.
    @param state       The state applied to the model when determining the
                       location of the Point.
    @return location   The location of the point expressed in the Ground. */
    const SimTK::Vec3& getLocationInGround(const SimTK::State& state) const;

    /** The velocity v_GP of this Point, P, relative to and expressed in Ground.
        Point's velocity is only valid at Stage::Velocity or higher.
    @param state       The state applied to the model when determining the
                       velocity of the Point.
    @return velocity   The velocity of the point expressed in the Ground. */
    const SimTK::Vec3& getVelocityInGround(const SimTK::State& state) const;

    /** The acceleration a_GP, of this Point, P, relative to and expressed in
    Ground. Point's acceleration is only valid at Stage::Acceleration or higher. 
    @param state       The state applied to the model when determining the
                       acceleration of the Point.
    @return velocity   The acceleration of the point expressed in Ground */
    const SimTK::Vec3& getAccelerationInGround(const SimTK::State& state) const;

    // End of Point's Spatial Kinematics
    ///@}

    /** Calculate the distance between this Point and some other Point
    @param state      The current State of the model.
    @param other      The other Point to which we want to get the distance between.
    @return distance  The distance (positive scalar). */
    double calcDistanceBetween(const SimTK::State& state, const Point& other) const;

    /** Calculate the distance between this Point and some other described as a
    location in some other frame.
    @param state      The current State of the model.
    @param frame      The other frame in which the location is defined.
    @param location   The location in the other frame.
    @return distance  The distance (positive scalar). */
    double calcDistanceBetween(const SimTK::State& state, const Frame& frame,
        const SimTK::Vec3& location) const;

    /** Calculate the relative speed between this Point and some other Point.
    It is the derivative of the distance with respect to time.
    A positive speed is growing the distance and negative is coming closer.
    @param state      The current State of the model.
    @param other      The other Point to which we want to get the speed between.
    @return speed     The speed (distance time derivative) which is a scalar. */
    double calcSpeedBetween(const SimTK::State& state, const Point& other) const;

protected:
    /** @name Point Extension methods.
    Concrete Point types must override these calculations.
    Results of the calculations are cached by the Point and made accessible
    via the corresponding getLocationInGround, getVelocityInGround, 
    getAccelerationInGround public methods (above). */
    /**@{**/

    /** Calculate the location of this Point with respect to and expressed in
        Ground as a function of the state. Can expect state to be realized to 
        at least SimTK::Stage::Position */
    virtual SimTK::Vec3
        calcLocationInGround(const SimTK::State& state) const = 0;
    /** Calculate the velocity of this Point with respect to and expressed in
        Ground as a function of the state. Can expect state to be realized to
        at least SimTK::Stage::Velocity */
    virtual SimTK::Vec3
        calcVelocityInGround(const SimTK::State& state) const = 0;
    /** Calculate the acceleration of this Point with respect to and expressed
        in Ground as a function of the state. Can expect state to be realized 
        to SimTK::Stage::Acceleration */
    virtual SimTK::Vec3
        calcAccelerationInGround(const SimTK::State& state) const = 0;

    /**@}**/

    /** @name Component Extension methods.
        Point types override these Component methods. */
    /**@{**/
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /**@}**/

private:

    mutable CacheVariable<SimTK::Vec3> _locationCV;
    mutable CacheVariable<SimTK::Vec3> _velocityCV;
    mutable CacheVariable<SimTK::Vec3> _accelerationCV;

//=============================================================================
};  // END of class Point
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_POINT_H_
