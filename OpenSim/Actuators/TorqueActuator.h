#ifndef OPENSIM_TORQUE_ACTUATOR_H_
#define OPENSIM_TORQUE_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  TorqueActuator.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Matt S. DeMers                                       *
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
 * Author: Ajay Seth, Matt DeMers
 */


#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim { 

class PhysicalFrame;
class Model;

//==============================================================================
//                           TORQUE ACTUATOR
//==============================================================================
/**
 * A TorqueActuator applies equal and opposite torques on the two bodies 
 * (bodyA and B) that it connects. The torque is applied about an axis
 * specified in ground (global) by default, otherwise it is in bodyA's frame. 
 * The magnitude of the torque is equal to the product of the optimal_force of 
 * the actuator and its control signal.
 *
 * @author Ajay Seth, Matt DeMers
 */
class OSIMACTUATORS_API TorqueActuator : public ScalarActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(TorqueActuator, ScalarActuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyA, std::string,
        "Name of Body to which the torque actuator is applied.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(bodyB, std::string,
        "Name of Body to which the equal and opposite torque is applied.");
    /** The default is torque_is_global=true. **/
    OpenSim_DECLARE_PROPERTY(torque_is_global, bool, 
        "Interpret axis in Ground frame if true; otherwise, body A's frame.");
    /** The default direction for the axis is z (0,0,1). **/
    OpenSim_DECLARE_PROPERTY(axis, SimTK::Vec3,
        "Fixed direction about which torque is applied, in Ground or body A "
        "frame depending on 'torque_is_global' property.");
    /** The default for optimal force is 1. **/
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum torque produced by this actuator when fully activated.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves body names unspecified. **/
    TorqueActuator();

    /** Convenience Constructor.
     Create a torque actuator that applies equal and opposite torques
     on the two bodies (bodyA and B) that it connects. The torque is applied 
     about an axis specified in ground if axisInGround is true, otherwise
     it is specified in bodyA's body frame. 

     @param bodyA   the body that the actuator applies torque to
     @param bodyB   the body that the actuator applies the opposite torque to
     @param axis    the axis about which the torque is applied
     @param axisInGround flag to indicate the axis is expressed in ground
                           otherwise, it is expressed in bodyA's frame
    */
    TorqueActuator(const PhysicalFrame& bodyA, const PhysicalFrame& bodyB,
                   const SimTK::Vec3& axis, bool axisInGround=true);

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.
    
    /** %Set the 'axis' property to the supplied value; frame is interpreted
    according to the 'torque_is_global' property. **/
    void setAxis(const SimTK::Vec3& axis) 
    {   set_axis(axis); }
    /** Return the current value of the 'axis' property. **/
    const SimTK::Vec3& getAxis() const 
    {   return get_axis(); }

    /** %Set the 'torque_is_global' property that determines how to interpret
    the 'axis' vector; if not global (Ground frame) it is in body A's frame. **/
    void setTorqueIsGlobal(bool isGlobal) 
    {   set_torque_is_global(isGlobal); }
    /** Return the current value of the 'torque_is_global' property. **/
    bool getTorqueIsGlobal() const
    {   return get_torque_is_global(); }

    /** %Set the 'optimal_force' property. **/
    void setOptimalForce(double optimalForce)
    {   set_optimal_force(optimalForce); }
    /** Get the current value of the 'optimal_force' property. **/
    double getOptimalForce() const override // Part of Actuator interface.
    {   return get_optimal_force(); }

    /** %Set the first body (bodyA) to which this actuator applies torque. */
    void setBodyA(const PhysicalFrame& body);
    /** %Set the second body (bodyB) to which this actuator applies torque. */
    void setBodyB(const PhysicalFrame& body);
    /** Get the first body (bodyA) to which this actuator applies torque. */
    const PhysicalFrame& getBodyA() const {return *_bodyA;}
    /** Get the second body (bodyB) to which this actuator applies torque. */
    const PhysicalFrame& getBodyB() const {return *_bodyB;}

protected:
    //--------------------------------------------------------------------------
    // Implement ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

//==============================================================================
// PRIVATE
//==============================================================================
private:
    void constructProperties();

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
    double getStress(const SimTK::State& state) const override; 
    //* Get speed along force vector. */
    double getSpeed(const SimTK::State& s) const override;
    //* Compute speed along force vector. */
    double calcSpeed(const SimTK::State& s) const;

    //--------------------------------------------------------------------------
    // Implement ModelComponent interface
    //--------------------------------------------------------------------------
    // Setup method initializes Body reference pointers to match the names.
    void extendConnectToModel(Model& model) override;

    //--------------------------------------------------------------------------
    // Implement Object interface.
    //--------------------------------------------------------------------------
    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        override;

//==============================================================================
// DATA
//==============================================================================
    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

    // Corresponding Body to which the torque actuator is applied.
    SimTK::ReferencePtr<const PhysicalFrame> _bodyA;

    // Corresponding Body to which the equal and opposite torque is applied.
    SimTK::ReferencePtr<const PhysicalFrame> _bodyB;

    // CachedVariable: Speed used to compute power.
    mutable CacheVariable<double> _speedCV;

//==============================================================================
};  // END of class TorqueActuator

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_TORQUE_ACTUATOR_H_


