#ifndef OPENSIM_EXPRESSION_BASED_BUSHING_FORCE_H_
#define OPENSIM_EXPRESSION_BASED_BUSHING_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                  OpenSim: ExpressionBasedBushingForce.h                    *
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


// INCLUDE
#include <OpenSim/Simulation/Model/ForceProducer.h>
#include <OpenSim/Simulation/Model/TwoFrameLinker.h>

namespace OpenSim {

//==============================================================================
//                          EXPRESSION BASED BUSHING FORCE
//==============================================================================
/**
 * A class implementing a bushing force specified by expressions of the
 * deflection between two bushing frames. These expressions are user specified
 * as strings that are interpreted during a simulation.
 * Each expression is a function of the bushing's rotational deflections 
 * (theta_x, theta_y, theta_z) and translational deflections, (delta_x, delta_y,
 * delta_z). These user defined expressions can capture nonlinearities and 
 * coupling common in biologic structures.  
 *
 * A bushing force is the resistive force due to deflection between two frames. 
 * One can think of the Bushing as being composed of 3 translational and 3 
 * torsional spring-dampers, which act along or about the bushing frame axes. 
 * Orientations are measured as x-y-z body-fixed Euler rotations.
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API ExpressionBasedBushingForce 
    : public TwoFrameLinker<ForceProducer, PhysicalFrame> {
OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedBushingForce, TwoFrameLinker);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(Mx_expression, std::string,
        "Expression defining the contribution of theta_x deflection to the "
        "moment about body_2's x axis. The expression is a string and can not "
        "have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(My_expression, std::string,
        "Expression defining the contribution of theta_y deflection to the "
        "moment about body_2's y axis. The expression is a string and can not "
        "have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Mz_expression, std::string,
        "Expression defining the contribution of theta_z deflection to the "
        "moment about body_2's z axis. The expression is a string and can not "
        "have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Fx_expression, std::string,
        "Expression defining the contribution of x deflection to the force in "
        "the x direction. The expression is a string and can not have any "
        "whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Fy_expression, std::string,
        "Expression defining the contribution of y deflection to the force in "
        "the y direction. The expression is a string and can not have any "
        "whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Fz_expression, std::string,
        "Expression defining the contribution of z deflection to the force in "
        "the z direction. The expression is a string and can not have any "
        "whitespace separating characters.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(visual_aspect_ratio, double,
        "Scalar number signifying the ratio of length/diameter used to display "
        "the force and moment vectors.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(moment_visual_scale, double,
        "Scale factor applied to the bushing moments before displaying them.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_visual_scale, double,
        "Scale factor applied to the bushing forces before displaying them.");

    OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
        "Damping parameters resisting angular deflection rate (theta_dot).");
    OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
        "Damping parameters resisting translational deflection (delta_dot) .");

//==============================================================================
// OUTPUTS
//==============================================================================
    OpenSim_DECLARE_OUTPUT(bushing_force, SimTK::Vec6, calcBushingForce,
            SimTK::Stage::Dynamics);

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves bodies unspecified, sets the bushing frames
      * to be at their body origins, and sets all bushing parameters to zero. **/
    ExpressionBasedBushingForce();

    /** This convenience constructor sets the bushing frames and sets all
     * bushing functions to zero.  **/
    ExpressionBasedBushingForce(const std::string& name,
                                const PhysicalFrame& frame1,
                                const PhysicalFrame& frame2);

    /** This convenience constructor defines and sets the bushing frames by name
     * and sets all bushing functions to zero.  **/
    ExpressionBasedBushingForce(const std::string& name,
                                const std::string& frame1Name,
                                const std::string& frame2Name);

    /** This convenience constructor defines and sets the bushing frames on
      * each body, and sets all bushing functions to zero.  **/
    ExpressionBasedBushingForce(const std::string& name,
                                const PhysicalFrame& frame1,
                                const SimTK::Vec3& point1,
                                const SimTK::Vec3& orientation1,
                                const PhysicalFrame& frame2,
                                const SimTK::Vec3& point2,
                                const SimTK::Vec3& orientation2);

    /** This convenience constructor defines and sets the bushing frames on 
      * each body, and sets all bushing functions to zero.  **/
    ExpressionBasedBushingForce(const std::string& name, 
                 const std::string& frame1Name,
                 const SimTK::Vec3& point1, 
                 const SimTK::Vec3& orientation1,
                 const std::string& frame2Name,
                 const SimTK::Vec3& point2, 
                 const SimTK::Vec3& orientation2);

    /** This convenience constructor defines a bushing that behaves like a
      * primitive bushing.  Stiffnesses are used to define linear functions for
      * force deflection profiles.**/
    ExpressionBasedBushingForce(const std::string& name,
                                const PhysicalFrame& frame1,
                                const SimTK::Vec3& point1,
                                const SimTK::Vec3& orientation1,
                                const PhysicalFrame& frame2,
                                const SimTK::Vec3& point2,
                                const SimTK::Vec3& orientation2,
                                const SimTK::Vec3& transStiffness,
                                const SimTK::Vec3& rotStiffness,
                                const SimTK::Vec3& transDamping,
                                const SimTK::Vec3& rotDamping);

    /** This convenience constructor defines a bushing that behaves like a
      * primitive bushing.  Stiffnesses are used to define linear functions for
      * force deflection profiles.**/
    ExpressionBasedBushingForce(const std::string& name,
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


    /** Set the value used to scale the bushing moment on body2 when drawing it to
        screen. A moment of magnitude |M| will be drawn on screen with a length of 
        (|M|*scale).  **/
    void setMomentVisualScale(double scale) {set_moment_visual_scale(scale);};
    /** Set the value used to scale the bushing force on body2 when drawing it to
        screen. A force of magnitude |F| will be drawn on screen with a length of
        (|F|*scale).  **/
    void setForceVisualScale(double scale) {set_force_visual_scale(scale);}
    /** Set the aspect ratio used to control the thickness of the bushing force
        and moment in drawn in the visualizer.  ratio = length/diameter.**/
    void setVisualAspectRatio(double ratio) {set_visual_aspect_ratio(ratio);}
    /** Set the expression defining Mx as a function of the bushing deflections 
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    void setMxExpression(std::string expression);
    /** Set the expression defining My as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    void setMyExpression(std::string expression);
    /** Set the expression defining Mz as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    void setMzExpression(std::string expression);
    /** Set the expression defining Fx as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    void setFxExpression(std::string expression);
    /** Set the expression defining Fy as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    void setFyExpression(std::string expression);
    /** Set the expression defining Fz as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    void setFzExpression(std::string expression);
    /** Get the expression defining Mx as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    std::string getMxExpression() { return get_Mx_expression(); }
    /** Get the expression defining My as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    std::string getMyExpression() { return get_My_expression(); }
    /** Get the expression defining Mz as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    std::string getMzExpression() { return get_Mz_expression(); }
    /** Get the expression defining Fx as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    std::string getFxExpression() { return get_Fx_expression(); }
    /** Get the expression defining Fy as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    std::string getFyExpression() { return get_Fy_expression(); }
    /** Get the expression defining Fz as a function of the bushing deflections
        theta_x, theta_y, theta_z, delta_x, delta_y, delta_z **/
    std::string getFzExpression() { return get_Fz_expression(); }

    //--------------------------------------------------------------------------
    // COMPUTATION
    //--------------------------------------------------------------------------
    /** Calculate the bushing force contribution due to its stiffness. This is 
        a function of the deflection between the bushing frames. It is the force
        on frame2 from frame1 in the basis of the deflection (dq). */
    SimTK::Vec6 calcStiffnessForce(const SimTK::State& state) const;

    /** Calculate the bushing force contribution due to its damping. This is a
        function of the deflection rate between the bushing frames. It is the 
        force on frame2 from frame1 in the basis of the deflection rate (dqdot).*/
    SimTK::Vec6 calcDampingForce(const SimTK::State& state) const;

    /** Calculate the total bushing force. This is the sum of the stiffness and
        damping force contributions. */
    SimTK::Vec6 calcBushingForce(const SimTK::State& state) const;

    //--------------------------------------------------------------------------
    // Reporting
    //--------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) 
     * to be reported.
     */
    virtual OpenSim::Array<std::string> getRecordLabels() const override;
    /**
    *  Provide the value(s) to be reported that correspond to the labels
    */
    virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const override;

protected:
    /** Potential energy calculation is not implemented */
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
    /**
     * Implements the `ForceProducer` interface by computing the bushing force.
     */
    void implProduceForces(const SimTK::State&, ForceConsumer&) const override;

    //--------------------------------------------------------------------------
    // Implement ModelComponent interface.
    //--------------------------------------------------------------------------
    void extendFinalizeFromProperties() override;

    void setNull();
    void constructProperties();

    SimTK::Mat66 _dampingMatrix{ 0.0 };

    // parser programs for efficiently evaluating the expressions
    Lepton::ExpressionProgram MxProg, MyProg, MzProg, FxProg, FyProg, FzProg;

//==============================================================================
};  // END of class ExpressionBasedBushingForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_EXPRESSION_BASED_BUSHING_FORCE_H_


