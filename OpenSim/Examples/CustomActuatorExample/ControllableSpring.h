#ifndef OPENSIM_CONTROLLABLE_SPRING_H_
#define OPENSIM_CONTROLLABLE_SPRING_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ControllableSpring.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matt S. DeMers                                                  *
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

#include "PistonActuator.h"

//=============================================================================
//=============================================================================
/**
 * A class that implements a variable stiffness spring actuator acting between 
 * two points on two bodies.
 * This actuator has no states; the control simply scales the optimal force
 * so that control*optimalForce = stiffness.
 *
 * @author Matt DeMers
 */

namespace OpenSim {

class ControllableSpring : public PistonActuator {
OpenSim_DECLARE_CONCRETE_OBJECT(ControllableSpring, PistonActuator);

//=============================================================================
// DATA
//=============================================================================
protected:
    // Additional Properties specific to a controllable spring need to be
    // defined.
    OpenSim_DECLARE_PROPERTY(rest_length, double,
            "rest length of the spring.");
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ControllableSpring() { constructProperties(); }
    /** The constructor has the same form as the PistonActuator constructor. */
    ControllableSpring(const PhysicalFrame& frameA, const PhysicalFrame& frameB)
        : PistonActuator(frameA, frameB)
    {
        constructProperties();
    }

    //--------------------------------------------------------------------------
    // GET and SET
    //--------------------------------------------------------------------------

private:
    /* Define private utilities to be used by the constructors. */
    void constructProperties() {
        constructProperty_rest_length(1.0);
    }
    
public:
    // Getter and setter.
    void setRestLength(double aLength) { set_rest_length(aLength); };
    double getRestLength() const { return get_rest_length(); };

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------

    /* The computeForce method is the meat of this simple actuator example. It
     * computes the direction and distance between the two application points.
     * It then uses the difference between its current length and rest length
     * to determine the force magnitude, then applies the force at the
     * application points, in the direction between them. */
    void computeForce(const SimTK::State& s, 
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
            SimTK::Vector& generalizedForces) const override
    {
        const PhysicalFrame& frameA = getFrameA();
        const PhysicalFrame& frameB = getFrameB();

        // We need points A and B expressed both in their frame and expressed in
        // ground.
        SimTK::Vec3 pointA_inGround;
        SimTK::Vec3 pointB_inGround;

        SimTK::Vec3 pointA = get_pointA();
        SimTK::Vec3 pointB = get_pointB();
        const Ground& ground = getModel().getGround();
        if (get_points_are_global())
        {
            pointA_inGround = pointA;
            pointB_inGround = pointB;
            pointA = ground.findStationLocationInAnotherFrame(s, pointA, frameA);
            pointB = ground.findStationLocationInAnotherFrame(s, pointB, frameB);
        }
        else
        {
            pointA_inGround = frameA.findStationLocationInGround(s, pointA);
            pointB_inGround = frameB.findStationLocationInGround(s, pointB);
        }

        // Find the direction along which the actuator applies its force.
        SimTK::Vec3 r = pointA_inGround - pointB_inGround;
        SimTK::UnitVec3 direction(r);
        double length = r.norm();

        /* find the stiffness.  computeActuation is defined in PistonActuator and
         * just returns the product of the actuator's control and its
         * _optimalForce.  We're using this to mean stiffness. */
        double stiffness = computeActuation(s);

        // find the force magnitude and set it. then form the force vector
        double forceMagnitude = (get_rest_length() - length)*stiffness;
        setActuation(s, forceMagnitude);
        SimTK::Vec3 force = forceMagnitude*direction;

        // Apply equal and opposite forces to the bodies.
        applyForceToPoint(s, frameA, pointA, force, bodyForces);
        applyForceToPoint(s, frameB, pointB, -force, bodyForces);
    }

//=============================================================================
}; // END of class ControllableSpring  

} // namespace OpenSim
//=============================================================================
//=============================================================================

#endif // OPENSIM_CONTROLLABLE_SPRING_H_
