#ifndef OPENSIM_POINT_ACTUATOR_H_
#define OPENSIM_POINT_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PointActuator.h                          *
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

/*
 * Author: Ajay Seth
 */


#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>


namespace OpenSim { 

class Body;
class Model;

//=============================================================================
//                              POINT ACTUATOR
//=============================================================================
/**
 * A class that implements a point actuator acting on the model.
 * This actuator has no states; the control is simply the force to
 * be applied to the model.
 *
 * @author Ajay Seth
 */
class OSIMACTUATORS_API PointActuator : public ScalarActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(PointActuator, ScalarActuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(body, std::string, 
        "Name of Body to which this actuator is applied.");
    OpenSim_DECLARE_PROPERTY(point, SimTK::Vec3,
        "Location of application point; in body frame unless "
        "point_is_global=true");
    /** The default is point_is_global=false. **/
    OpenSim_DECLARE_PROPERTY(point_is_global, bool,
        "Interpret point in Ground frame if true; otherwise, body frame.");
    OpenSim_DECLARE_PROPERTY(direction, SimTK::Vec3,
        "Force application direction; in body frame unless "
        "force_is_global=true.");
    /** The default is force_is_global=false. **/
    OpenSim_DECLARE_PROPERTY(force_is_global, bool,
        "Interpret direction in Ground frame if true; otherwise, body frame.");
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum force produced by this actuator when fully activated.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor or construct with body name given. An empty 
    name ("") is treated as though it were unspecified. **/
    explicit PointActuator(const std::string& bodyName="");

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

    /** %Set the 'optimal_force' property. **/
    void setOptimalForce(double aOptimalForce);
    /** Get the current value of the 'optimal_force' property. **/
    double getOptimalForce() const override; // Part of Actuator interface.

protected:

    //--------------------------------------------------------------------------
    // Implement ModelComponent Interface
    //--------------------------------------------------------------------------
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

private:
    void setNull();
    void constructProperties();

    // Set the body to which this actuator applies; setting this pointer
    // also sets the corresponding body name property.
    void setBody(Body* aBody);
    Body* getBody() const;

    //--------------------------------------------------------------------------
    // Implement ScalarActuator interface
    //--------------------------------------------------------------------------
    /** Get speed along force vector. */
    double getSpeed(const SimTK::State& s) const override;

    /** Computes speed along force vector. */
    double calcSpeed(const SimTK::State& s) const;

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
    double getStress(const SimTK::State& s) const override;

    //--------------------------------------------------------------------------
    // Implement ModelComponent interface
    //--------------------------------------------------------------------------
    // Setup method to initialize Body reference
    void extendConnectToModel(Model& model) override;

    //--------------------------------------------------------------------------
    // Implement Object interface.
    //--------------------------------------------------------------------------
    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber=-1)
        override;

    // Corresponding Body to which the point actuator is applied.
    SimTK::ReferencePtr<Body> _body;

    // CacheVariable for storing computed speed along force direction.
    mutable CacheVariable<double> _speedCV;

//=============================================================================
};  // END of class PointActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_POINT_ACTUATOR_H_


