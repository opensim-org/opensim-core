#ifndef _OPENSIM_BODY_DRAG_FORCE_PLUGIN_H_
#define _OPENSIM_BODY_DRAG_FORCE_PLUGIN_H_
/* -------------------------------------------------------------------------- *
 *                             BodyDragForce.h                                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//=============================================================================
// INCLUDES
//=============================================================================
// Headers define the various property types that OpenSim objects can read 
#include <string>
#include "osimPluginDLL.h"
#include <OpenSim/Simulation/Model/Force.h>


//=============================================================================
//=============================================================================
/*
 * A class template for writing a custom Force plugin. 
 * Applies a body drag force to a given OpenSim Body at
 * it's center of mass location.
 *
 * F_drag = coefficnent * (V_body_com_in_ground_frame)^exponent
 *
 * @author Tim Dorn
 * @version 3.0
 */
namespace OpenSim {

class OSIMPLUGIN_API BodyDragForce : public Force  
{
OpenSim_DECLARE_CONCRETE_OBJECT(BodyDragForce, Force);
public:
//=============================================================================
// PROPERTIES
//=============================================================================

    /** String property containing the name of the body names*/
    OpenSim_DECLARE_PROPERTY(body_name, std::string, 
    "Body name to apply drag force to.");

    /** Double property containing the drag coefficient*/
    OpenSim_DECLARE_PROPERTY(coefficient, double, 
    "Names of the bodies on which to perform the analysis."
    "The key word 'All' indicates that the analysis should be performed for all bodies.");

    /** Double property containing the drag exponent*/
    OpenSim_DECLARE_PROPERTY(exponent, double, 
    "Names of the bodies on which to perform the analysis."
    "The key word 'All' indicates that the analysis should be performed for all bodies.");


    // Here are some examples of other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //// My string property
    //OpenSim_DECLARE_PROPERTY(string_property, std::string, 
    //"My string property."); 

    //// My int property
    //OpenSim_DECLARE_PROPERTY(int_property, int, 
    //"My int property."); 

    //// My bool property
    //OpenSim_DECLARE_PROPERTY(bool_property, bool, 
    //"My bool property."); 

    //// My double property
    //OpenSim_DECLARE_PROPERTY(double_property, double, 
    //"My double property."); 


//=============================================================================
// METHODS
//=============================================================================
public:
    // Default Constructor
    BodyDragForce();


    //--------------------------------------------------------------------------
    // COMPUTATION
    //--------------------------------------------------------------------------
    /** Compute the bushing force contribution to the system and add in to appropriate
      * bodyForce and/or system generalizedForce. The bushing force is [K]*dq + [D]*dqdot
      * where, [K] is the spatial 6dof stiffness matrix between the two frames 
               dq is the deflection in body spatial coordinates with rotations in Euler angles
      *        [D] is the spatial 6dof damping matrix opposing the velocity between the frames
      *        dqdot is the relative spatial velocity of the two frames
      * BodyDragForce implementation based SimTK::Force::LinearBushing
      * developed and implemented by Michael Sherman.
      */
    void computeForce(const SimTK::State& s, 
                      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                      SimTK::Vector& generalizedForces) const override;

    /** Potential energy is determined by the elastic energy storage of the bushing.
        In spatial terms, U = ~dq*[K]*dq, with K and dq defined above. */
    double computePotentialEnergy(const SimTK::State& s) const override;

    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double>
    getRecordValues(const SimTK::State& state) const override;

protected:
    virtual void connectToModel(Model& aModel);


private:
    void setNull();
    void constructProperties();

    
//=============================================================================
};  // END of class BodyDragForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // _OPENSIM_BODY_DRAG_FORCE_PLUGIN_H_


