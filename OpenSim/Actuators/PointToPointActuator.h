#ifndef OPENSIM_POINT_TO_POINT_ACTUATOR_H_
#define OPENSIM_POINT_TO_POINT_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  PointToPointActuator.h                      *
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim { 

class Body;
class Model;

//=============================================================================
//                     POINT TO POINT ACTUATOR
//=============================================================================
/**
 * A class that implements a force actuator acting between two points on two bodies.
 * The direction of the force is along the line between the points, with a positive
 * value acting to expand the distance between them.  This actuator has no states; 
 * the control is simply the force to be applied to the model.
 *
 * @author Matt DeMers
 */
class OSIMACTUATORS_API PointToPointActuator : public ScalarActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(PointToPointActuator, ScalarActuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyA, std::string,
        "Name of Body to which the point-to-point actuator is applied.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyB, std::string,
        "Name of Body to which the equal and opposite force is applied.");
    /** The default is points_are_global=false. **/
    OpenSim_DECLARE_PROPERTY(points_are_global, bool, 
        "Interpret points in Ground frame if true; otherwise, corresponding "
        "body's frame.");
    /** The default location for pointA is bodyA's origin. **/
    OpenSim_DECLARE_PROPERTY(pointA, SimTK::Vec3,
        "Point of application on body A.");
    /** The default location for pointB is bodyB's origin. **/
    OpenSim_DECLARE_PROPERTY(pointB, SimTK::Vec3,
        "Point of application on body B.");
    /** The default for optimal force is 1. **/
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum force produced by this actuator when fully activated.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves body names unspecified. **/
    PointToPointActuator();
    /** Construct with specified body names. **/
    PointToPointActuator(const std::string& bodyNameA, 
                         const std::string& bodyNameB);
    
    /** %Set the 'pointA' property to the supplied value; frame is interpreted
    according to the 'points_are_global' property. **/
    void setPointA(const SimTK::Vec3& pointAPos) 
    {   set_pointA(pointAPos); } ;
    /** Return the current value of the 'pointA' property. **/
    const SimTK::Vec3& getPointA() const 
    {   return get_pointA(); };
    /** %Set the 'pointB' property to the supplied value; frame is interpreted
    according to the 'points_are_global' property. **/
    void setPointB(const SimTK::Vec3& pointBPos) 
    {   set_pointB(pointBPos); } ;
    /** Return the current value of the 'pointB' property. **/
    const SimTK::Vec3& getPointB() const 
    {   return get_pointB(); };

    /** %Set the 'points_are_global' property that determines how to interpret
    the 'pointA' and 'pointB' location vectors: if not global (Ground frame) 
    then they are in the local frame of 'bodyA' and 'bodyB' respectively. **/
    void setPointsAreGlobal(bool isGlobal) 
    {   set_points_are_global(isGlobal); };
    /** Return the current value of the 'points_are_global' property. **/
    bool getPointsAreGlobal() const
    {   return get_points_are_global(); };

    /** %Set the 'optimal_force' property. **/
    void setOptimalForce(double optimalForce)
    {   set_optimal_force(optimalForce); }
    /** Get the current value of the 'optimal_force' property. **/
    double getOptimalForce() const override // Part of Actuator interface.
    {   return get_optimal_force(); }

    // default destructor, copy constructor, copy assignment

protected:
    //--------------------------------------------------------------------------
    // Implement ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    void constructProperties();

    // Set the bodies to which this actuator applies; setting these pointers
    // also sets the corresponding body name properties.
    void setBodyA(Body* bodyp);
    void setBodyB(Body* bodyp);
    Body* getBodyA() const {return _bodyA.get();}
    Body* getBodyB() const {return _bodyB.get();}    

    //--------------------------------------------------------------------------
    // Implement Force interface
    //--------------------------------------------------------------------------
    void computeForce(const SimTK::State& state, 
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                      SimTK::Vector& mobilityForces) const override;

    //--------------------------------------------------------------------------
    // Implement Actuator interface (also see getOptimalForce() above)
    //--------------------------------------------------------------------------
    double computeActuation(const SimTK::State& s) const override;
    // Return the stress, defined as abs(force/optimal_force).
    double getStress(const SimTK::State& s) const override;
    /** Get speed along force vector. */
    double getSpeed(const SimTK::State& s) const override;
    /** Computes speed along force vector. */
    double calcSpeed(const SimTK::State& s) const;

    //--------------------------------------------------------------------------
    // Implement ModelComponent interface
    //--------------------------------------------------------------------------
    // Setup method initializes Body reference pointers to match the names.
    void extendConnectToModel(Model& aModel) override;

//=============================================================================
// DATA
//=============================================================================
    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

    // The bodies on which this point-to-point actuator acts.
    SimTK::ReferencePtr<Body> _bodyA, _bodyB;

    // CachedVariables: Speed- and direction along force, used to compute power
    mutable CacheVariable<double> _speedCV;
    mutable CacheVariable<SimTK::UnitVec3> _directionCV;

    SimTK::UnitVec3 getDirectionBAInGround(const SimTK::State& s) const;
//=============================================================================
};  // END of class PointToPointActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_POINT_TO_POINT_ACTUATOR_H_


