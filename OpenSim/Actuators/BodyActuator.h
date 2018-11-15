#ifndef OPENSIM_BODY_ACTUATOR_H_
#define OPENSIM_BODY_ACTUATOR_H_
/* ------------------------------------------------------------------------- *
*                         OpenSim:  BodyActuator.h                           *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2017 Stanford University and the Authors                *
* Author(s): Soha Pouya, Michael Sherman                                     *
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

#include <OpenSim/Actuators/osimActuatorsDLL.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

namespace OpenSim {

class Model;

//=============================================================================
//                              BODY ACTUATOR
//=============================================================================
/**
 * Apply a spatial force (that is, [torque, force]) on a given point of the 
 * given body. That is, the force is applied at the given point; torques don't
 * have associated points. This actuator has no states; the control signal  
 * should provide a 6D vector including [torque(3D), force(3D)] that is supposed 
 * to be applied to the body.
 * The associated controller can generate the spatial force [torque, force] both
 * in the body and global (ground) frame. The default is assumed to be global 
 * frame. The point of application can be specified both in the body and global 
 * (ground) frame. The default is assumed to be the body frame.
 *
 * @author Soha Pouya, Michael Sherman
 */
class OSIMACTUATORS_API BodyActuator : public Actuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(BodyActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(point, SimTK::Vec3,
        "Location of application point; in body frame unless "
        "point_is_global=true");
    /** The default is point_is_global=false. **/
    OpenSim_DECLARE_PROPERTY(point_is_global, bool,
        "Interpret point in Ground frame if true; otherwise, body frame.");
    /** The default is spatial_force_is_global=true. **/
    OpenSim_DECLARE_PROPERTY(spatial_force_is_global, bool,
        "Interpret axis in Ground frame if true; otherwise, body's frame.");
    
//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(body, Body,
        "The body on which to apply the spatial force.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================

    /** Default constructor leaves body names unspecified. **/
    BodyActuator();

    /** Convenience Constructor.
    Create a body actuator that applies a vector of spatial forces in the form 
    of [torque, force] on a body. The torque is applied about the axis specified
    in ground frame.

    @param[in] body                 the body that the actuator applies torque to
    @param[in] point                the point that the force is applied to
    @param[in] pointIsGlobal        a flag to set if the point is in global frame
    @param[in] spatialForceIsGlobal a flag to set if the force is in global frame
    */

    explicit BodyActuator(const Body& body,
                          const SimTK::Vec3& point = SimTK::Vec3(0),
                          bool pointIsGlobal = false,
                          bool spatialForceIsGlobal = true);
    //--------------------------------------------------------------------------
    // Get & Set
    //--------------------------------------------------------------------------
    /** %Set the 'point' property that determines where the force vector should
    be applied. The default is the origin of the body Vec3(0). **/
    void setPoint(SimTK::Vec3& point)
    {set_point(point);}
    /** Return the current value of the 'point' property. **/
    const SimTK::Vec3& getPoint() const
    {return get_point();}


    /** %Set the 'point_is_global' property that determines whether the point is  
    specified in inertial coordinates or in the body's local coordinates. **/
    void setPointForceIsGlobal(bool isGlobal)
    {set_point_is_global(isGlobal); }
    /** Return the current value of the 'point_is_global' property. **/
    bool getPointIsGlobal() const
    {return get_point_is_global();  }


    /** %Set the 'spatial_force_is_global' property that determines how to 
    interpret the 'axis' vector; if not global (Ground frame) it is in body's  
    frame. **/
    void setSpatialForceIsGlobal(bool isGlobal)
    {set_spatial_force_is_global(isGlobal);}
    /** Return the current value of the 'spatial_force_is_global' property. **/
    bool getSpatialForceIsGlobal() const
    {return get_spatial_force_is_global();}


    /* %Set the body to which this actuator applies spatial forces. */
    void setBody(const Body& body);
    const Body& getBody() const;


    /* %Set the body name to which this actuator applies spatial forces. */
    void setBodyName(const std::string& name);
    const std::string& getBodyName() const;


private:
    //--------------------------------------------------------------------------
    // Implement Component interface
    //--------------------------------------------------------------------------
    /** Construct the infrastructure of the BodyActuator component.
    Begin with its properties. */
    void constructProperties();

    //--------------------------------------------------------------------------
    // Implement Force interface
    //--------------------------------------------------------------------------
    void computeForce(const SimTK::State& state,
                    SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                    SimTK::Vector& mobilityForces) const override;

    //--------------------------------------------------------------------------
    // Implement Actuator interface.
    //--------------------------------------------------------------------------
    int numControls() const override { return 6; }
    double getPower(const SimTK::State& s) const override;

    //=========================================================================
};  // END of class BodyActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_BODY_ACTUATOR_H_


