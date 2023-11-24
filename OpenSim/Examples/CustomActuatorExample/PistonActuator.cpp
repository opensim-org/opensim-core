/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PistonActuator.cpp                        *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "PistonActuator.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTORS
//=============================================================================
PistonActuator::PistonActuator()
{
    constructProperties();
}

PistonActuator::PistonActuator(const PhysicalFrame& frameA,
        const PhysicalFrame& frameB) : PistonActuator() {
    setFrameA(frameA);
    setFrameB(frameB);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
void PistonActuator::constructProperties()
{
    setAuthors("Matt S. DeMers");

    constructProperty_points_are_global(false);
    SimTK::Vec3 x(0.0, 0.0, 0.0);
    constructProperty_pointA(x);
    constructProperty_pointB(x);
    constructProperty_optimal_force(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
void PistonActuator::setFrameA(const PhysicalFrame& frameA) 
{
    // This function is created by the OpenSim_DECLARE_SOCKET macro.
    connectSocket_frameA(frameA);
}

void PistonActuator::setFrameB(const PhysicalFrame& frameB) 
{
    connectSocket_frameB(frameB);
}

const PhysicalFrame& PistonActuator::getFrameA() const
{
    // getConnectee() is a function in Component for accessing the component
    // connected to a socket. Inside the angle brackes, we provide the type
    // (class) of the socket.
    return getConnectee<PhysicalFrame>("frameA");
}

const PhysicalFrame& PistonActuator::getFrameB() const
{
    return getConnectee<PhysicalFrame>("frameB");
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
void PistonActuator::setOptimalForce(double aOptimalForce)
{
    set_optimal_force(aOptimalForce);
}
double PistonActuator::getOptimalForce() const
{
    return get_optimal_force();
}

double PistonActuator::getStress( const SimTK::State& s) const
{
    return fabs(getActuation(s) / get_optimal_force());
}


//=============================================================================
// FORCE INTERFACE
//=============================================================================
void PistonActuator::computeForce(
    const SimTK::State& s,
    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
    SimTK::Vector& generalizedForces) const
{
    const PhysicalFrame& frameA = getFrameA();
    const PhysicalFrame& frameB = getFrameB();

    // We need points A and B expressed in local A and B frame respectively.
    SimTK::Vec3 pointA   = get_pointA();
    SimTK::Vec3 pointB   = get_pointB();
    const Ground& ground = getModel().getGround();
    if (get_points_are_global()) {
        pointA = ground.findStationLocationInAnotherFrame(s, pointA, frameA);
        pointB = ground.findStationLocationInAnotherFrame(s, pointB, frameB);
    }

    // Calculate the force magnitude and the force vector.
    double forceMagnitude = computeActuation(s);
    setActuation(s, forceMagnitude);
    SimTK::Vec3 force = forceMagnitude * calcDirectionBAInGround(s);

    // Apply equal and opposite forces to the bodies.
    applyForceToPoint(s, frameA, pointA, force, bodyForces);
    applyForceToPoint(s, frameB, pointB, -force, bodyForces);
}

SimTK::UnitVec3 PistonActuator::calcDirectionBAInGround(
    const SimTK::State& s) const
{
    // Get pointA and pointB positions in the local frame of bodyA and bodyB,
    // respectively.
    // Points may have been supplied either global or local frame.
    const bool pointsAreGlobal = get_points_are_global();
    const SimTK::Vec3& pointA  = getPointA();
    const SimTK::Vec3& pointB  = getPointB();

    SimTK::Vec3 pointA_inGround =
        pointsAreGlobal ? pointA
                        : getFrameA().findStationLocationInGround(s, pointA);
    SimTK::Vec3 pointB_inGround =
        pointsAreGlobal ? pointB
                        : getFrameB().findStationLocationInGround(s, pointB);

    // Find the direction along which the actuator applies its force.
    const SimTK::Vec3 r = pointA_inGround - pointB_inGround;
    const SimTK::UnitVec3 direction(r); // normalize

    return direction;
}

//=============================================================================
// SCALAR ACTUATOR INTERFACE
//=============================================================================
double PistonActuator::getSpeed(const SimTK::State& s) const
{
    // Speed is zero for constant points defined in the same frame.
    if (get_points_are_global()) {
        return 0.;
    }

    const SimTK::Vec3& pointA_inBodyA = getPointA();
    const SimTK::Vec3& pointB_inBodyB = getPointB();

    // Get the relative velocity of the points in ground.
    SimTK::Vec3 velA_G =
        getFrameA().findStationVelocityInGround(s, pointA_inBodyA);
    SimTK::Vec3 velB_G =
        getFrameB().findStationVelocityInGround(s, pointB_inBodyB);
    SimTK::Vec3 velAB_G = velA_G - velB_G;
    // Speed used to compute power is the speed along the line connecting
    // the two bodies.
    double speed = ~velAB_G * calcDirectionBAInGround(s);
    return speed;
}

//=============================================================================
// ACTUATOR INTERFACE
//=============================================================================
double PistonActuator::computeActuation(const SimTK::State& s) const
{
    return getControl(s) * getOptimalForce();
}
