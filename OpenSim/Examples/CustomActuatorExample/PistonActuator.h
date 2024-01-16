#ifndef OPENSIM_PISTON_ACTUATOR_H_
#define OPENSIM_PISTON_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PistonActuator.h                         *
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

/*
 * Author: Matt DeMers
 */

#include <OpenSim/OpenSim.h>

//=============================================================================
//=============================================================================
/** A class that implements a force actuator acting between two points on two
 * bodies. The direction of the force is along the line between the points,
 * with a positive value acting to expand the distance between them.  This
 * actuator has no states; the control is simply the force to be applied to the
 * model.
 *
 * @author Matt DeMers */
namespace OpenSim { 

class PhysicalFrame;
class Model;

class PistonActuator : public ScalarActuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PistonActuator, ScalarActuator);

//=============================================================================
// DATA
//=============================================================================
protected:
    // PROPERTIES

    // frameA is described below..
    OpenSim_DECLARE_PROPERTY(pointA, SimTK::Vec3,
    "Point of application on frameA.");

    OpenSim_DECLARE_PROPERTY(pointB, SimTK::Vec3,
    "Point of application on frameB.");

    OpenSim_DECLARE_PROPERTY(points_are_global, bool,
    "bool to indicate whether or not the points are expressed in global frame.");

    OpenSim_DECLARE_PROPERTY(optimal_force, double,
    "Force = control * optimal_force.");

    // SOCKETS

    // Sockets allow this actuator to access the PhysicalFrames (e.g., bodies)
    // to which this actuator applies force.
    OpenSim_DECLARE_SOCKET(frameA, PhysicalFrame,
    "The frame to which this actuator applies force.");

    OpenSim_DECLARE_SOCKET(frameB, PhysicalFrame,
    "The frame to which this actuator applies an equal and opposite force.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PistonActuator();
    PistonActuator(const PhysicalFrame& frameA, const PhysicalFrame& frameB);
private:
    void constructProperties();

public:

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    /** Set the PhysicalFrame (e.g., Body) to which the Body actuator is
     * applied. */
    void setFrameA(const PhysicalFrame& frameA);
    /** Set the generalized Body to which the equal and opposite Body actuation
     * is applied. */
    void setFrameB(const PhysicalFrame& frameB);
    const PhysicalFrame& getFrameA() const;
    const PhysicalFrame& getFrameB() const;

    // Force points of application
    void setPointA(SimTK::Vec3 aPosition) { set_pointA(aPosition); } ;
    SimTK::Vec3 getPointA() const { return get_pointA(); };
    void setPointB(SimTK::Vec3 aPosition) { set_pointA(aPosition); } ;
    SimTK::Vec3 getPointB() const { return get_pointB(); };

    // flag for reference frame
    void setPointsAreGlobal(bool aBool) {set_points_are_global(aBool); };
    bool getPointsAreGlobal() {return get_points_are_global(); };

    // OPTIMAL FORCE
    /** The force applied by this actuator is its control signal multiplied
     * by this optimal force. */
    void setOptimalForce(double aOptimalForce);
    double getOptimalForce() const override;
    /** This is the absolute value of the force generated by this actuator
     * divided by its optimal force. */
    double getStress(const SimTK::State& s) const override;
    //--------------------------------------------------------------------------
    // FORCE INTERFACE
    //--------------------------------------------------------------------------
    /** Apply the actuator force to frameA and frameB. */
    void computeForce(const SimTK::State& s, 
                              SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                              SimTK::Vector& generalizedForces) const override;

    //--------------------------------------------------------------------------
    // ACTUATOR INTERFACE
    //--------------------------------------------------------------------------
    /** Compute the control-dependent force magnitude applied by this actuator.
     */
    double computeActuation(const SimTK::State& s) const override;

    //--------------------------------------------------------------------------
    // SCALAR ACTUATOR INTERFACE
    //--------------------------------------------------------------------------
    /** Compute the speed along the force direction. */
    double getSpeed(const SimTK::State& s) const override;

    private:
    /** Compute the direction of the actuator force in ground frame. */
    SimTK::UnitVec3 calcDirectionBAInGround(const SimTK::State& s) const;

//=============================================================================
}; // END of class PistonActuator

} //namespace OpenSim
//=============================================================================
//=============================================================================

#endif // OPENSIM_PISTON_ACTUATOR_H_
