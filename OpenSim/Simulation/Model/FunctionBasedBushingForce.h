#ifndef OPENSIM_FUNCTION_BASED_BUSHING_FORCE_H_
#define OPENSIM_FUNCTION_BASED_BUSHING_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  FunctionBasedBushingForce.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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


// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "Force.h"
#include <OpenSim/Simulation/Model/LinkTwoFrames.h>

namespace OpenSim {

class Function;

//==============================================================================
//                             FUNCTION BASED BUSHING FORCE
//==============================================================================
/**
 * A class implementing a bushing force driven by functions relating forces to 
 * deviations.  Thes functions are user defined functions that can caputure
 * the nonlinearities of biologic structures.  This FunctionBasedBushing
 * does not capture coupling between the deflections (e.g. force in x due to 
 * rotation in z).
 *
 * A bushing force is the resistive force due to deviation between two frames. 
 * One can think of the Bushing as being composed of 3 translational and 3 
 * torsional spring-dampers, which act along or about the bushing frame axes. 
 * Orientations are measured as x-y-z body-fixed Euler rotations.
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API FunctionBasedBushingForce
    : public LinkTwoFrames<Force, PhysicalFrame> {
OpenSim_DECLARE_CONCRETE_OBJECT(FunctionBasedBushingForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(m_x_theta_x_function, Function,
        "Function defining the contribution of theta_x deflection to the moment about body_2's x axis.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(m_y_theta_y_function, Function,
        "Function defining the contribution of theta_y deflection to the moment about body_2's y axis.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(m_z_theta_z_function, Function,
        "Function defining the contribution of theta_z deflection to the moment about body_2's z axis.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(f_x_delta_x_function, Function,
        "Function defining the contribution of x deflection to the force in the x direction.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(f_y_delta_y_function, Function,
        "Function defining the contribution of y deflection to the force in the y direction.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(f_z_delta_z_function, Function,
        "Function defining the contribution of z deflection to the force in the z direction.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(visual_aspect_ratio, double,
        "Scalar number signifying the ratio of length/diameter used to display the force and "
        "moment vectors.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(moment_visual_scale, double,
        "Value multiplying the bushing moments before displaying the moment vector");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_visual_scale, double,
        "Value multiplying the bushing forces before displaying the force vector");

    //-------------------------------------------------------
    OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
        "Damping parameters resisting relative angular velocity.");
    OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
        "Damping parameters resisting relative translational velocity.");

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves bodies unspecified, sets the bushing frames
      * to be at their body origins, and sets all bushing parameters to zero. **/
    FunctionBasedBushingForce();
    /** This convenience constructor defines and sets the bushing frames on 
      * each body, and sets all bushing functions to zero.  **/
    FunctionBasedBushingForce(const std::string& name, 
                            const std::string& frame1Name,
                            const SimTK::Vec3& point1,
                            const SimTK::Vec3& orientation1,
                            const std::string& frame2Name,
                            const SimTK::Vec3& point2,
                            const SimTK::Vec3& orientation2);
    /** This convenience constructor defines a bushing that behaves like a
      * primitive bushing.  Stiffnesses are used to define linear functions for
      * force deflection profiles.**/
    FunctionBasedBushingForce(const std::string& name,
                            const std::string& frame1Name,
                            const SimTK::Vec3& point1,
                            const SimTK::Vec3& orientation1,
                            const std::string& frame2Name,
                            const SimTK::Vec3& point2,
                            const SimTK::Vec3& orientation2,
                            const SimTK::Vec3& transStiffness,
                            const SimTK::Vec3& rotStiffness,
                            const SimTK::Vec3& transDamping,
                            const SimTK::Vec3& rotDamping);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

    /** Set the value used to scale the bushing moment on body2 when drawing it to screen.  
      * A moment of magnitude |M| will be drawn on screen with a length of (|M|*scale).  **/
    void setMomentVisualScale(double scale) {set_moment_visual_scale(scale);};
    /** Set the value used to scale the bushing force on body2 when drawing it to screen.  
      * A force of magnitude |F| will be drawn on screen with a length of (|F|*scale).  **/
    void setForceVisualScale(double scale) {set_force_visual_scale(scale);}
    /** Set the aspect ratio used to control the thickness of the bushing force and moment
        in drawn in the visualizer.  ratio = length/diameter.*/
    void setVisualAspectRatio(double ratio) {set_visual_aspect_ratio(ratio);}

    //--------------------------------------------------------------------------
    // COMPUTATION
    //--------------------------------------------------------------------------
    /** Compute the bushing force contribution to the system and add in to appropriate
      * bodyForce and/or system generalizedForce. 
      */
    void computeForce (const SimTK::State& s, 
                        SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
                        SimTK::Vector& generalizedForces) const override;

    //--------------------------------------------------------------------------
    // Reporting
    //--------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) 
     * to be reported.
     */
    OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;

protected:
    //--------------------------------------------------------------------------
    // Visual support in SimTK visualizer
    // -------------------------------------------------------------------------
    void generateDecorations(
        bool fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const override;
    void ComputeForcesAtBushing(const SimTK::State& state, 
                                SimTK::SpatialVec& forces_on_M_in_ground, 
                                SimTK::SpatialVec& forces_on_F_in_ground) const;

private:

    void setNull();
    void constructProperties();

//==============================================================================
};  // END of class FunctionBasedBushingForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FUNCTION_BASED_BUSHING_FORCE_H_


