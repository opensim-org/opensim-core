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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include "Force.h"
#include <OpenSim/Common/osimCommon.h>
#include <Vendors/lepton/include/Lepton.h>


namespace OpenSim {

//==============================================================================
//                             EXPRESSION BASED BUSHING FORCE
//==============================================================================
/**
 * A class implementing a bushing force driven by expressions relating forces to 
 * deviations.  These expressions are user defined expressions 
 * provided as strings that are parsed and evaluated during a simulation.
 * Each expression can be a function of the bushing's rotational deflections 
 * (theta_x, theta_y, theta_z) and translational deflections, (delta_x, delta_y,
 * delta_z).  These user defined expressions can capture nonlinearities and 
 * coupling common in biologic structures.  
 *
 * A bushing force is the force increasing due to deviation between two frames. 
 * One can think of the Bushing as being composed of 3 translational and 3 
 * torsional spring-dampers, which act along or about the bushing frames. 
 * Orientations are measured as x-y-z body-fixed Euler rotations.
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing.
 *
 * @author Matt DeMers
 */
class OSIMSIMULATION_API ExpressionBasedBushingForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedBushingForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body_1, std::string,
		"One of the two bodies connected by the bushing.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body_2, std::string,
		"The other of the two bodies connected by the bushing.");
	OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
		"Location of bushing frame on body 1.");
	OpenSim_DECLARE_PROPERTY(orientation_body_1, SimTK::Vec3,
		"Orientation of bushing frame in body 1 as x-y-z, body fixed Euler rotations.");
	OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
		"Location of bushing frame on body 2.");
	OpenSim_DECLARE_PROPERTY(orientation_body_2, SimTK::Vec3,
		"Orientation of bushing frame in body 2 as x-y-z, body fixed Euler rotations.");
    OpenSim_DECLARE_PROPERTY(Mx_expression, std::string,
        "Expression defining the contribution of theta_x deflection to the moment about body_2's x axis."
		"The expression is a string and can not have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(My_expression, std::string,
        "Expression defining the contribution of theta_y deflection to the moment about body_2's y axis."
		"The expression is a string and can not have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Mz_expression, std::string,
        "Expression defining the contribution of theta_z deflection to the moment about body_2's z axis."
		"The expression is a string and can not have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Fx_expression, std::string,
        "Expression defining the contribution of x deflection to the force in the x direction."
		"The expression is a string and can not have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Fy_expression, std::string,
        "Expression defining the contribution of y deflection to the force in the y direction."
		"The expression is a string and can not have any whitespace separating characters.");
    OpenSim_DECLARE_PROPERTY(Fz_expression, std::string,
        "Expression defining the contribution of z deflection to the force in the z direction."
		"The expression is a string and can not have any whitespace separating characters.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(visual_aspect_ratio, double,
        "Scalar number signifying the ratio of length/diameter used to display the force and "
        "moment vectors.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(moment_visual_scale, double,
        "Value multiplying the bushing moments before displaying the moment vector");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(force_visual_scale, double,
        "Value multiplying the bushing forces before displaying the force vector");
    // To do:  Delete these stiffness properties
    //--------------------------------------------------------
    /*
	OpenSim_DECLARE_PROPERTY(rotational_stiffness, SimTK::Vec3,
		"Stiffness parameters resisting relative rotation.");
	OpenSim_DECLARE_PROPERTY(translational_stiffness, SimTK::Vec3,
		"Stiffness parameters resisting relative translation.");
    */
    //-------------------------------------------------------
	OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
		"Damping parameters resisting relative angular velocity.");
	OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
		"Damping parameters resisting relative translational velocity.");
    /**@}**/

protected:
    /** how to display the bushing */
	VisibleObject _displayer;
private:
	// parser programs for efficiently evaluating the expressions
	Lepton::ExpressionProgram MxProg, MyProg, MzProg, FxProg, FyProg, FzProg; 
	// underlying SimTK system elements
	// the mobilized bodies involved
	const SimTK::MobilizedBody *_b1;
	const SimTK::MobilizedBody *_b2;
	// The bushing frames affixed to the mobilized bodies
	SimTK::Transform _inb1;
	SimTK::Transform _inb2;
    

public:
//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor leaves bodies unspecified, sets the bushing frames
      * to be at their body origins, and sets all bushing parameters to zero. **/
	ExpressionBasedBushingForce();
    /** This convenience constructor defines and sets the bushing frames on 
      * each body, and sets all bushing functions to zero.  **/
    ExpressionBasedBushingForce(const std::string& body1Name, 
                 const SimTK::Vec3& point1, 
                 const SimTK::Vec3& orientation1,
		         const std::string& body2Name, 
                 const SimTK::Vec3& point2, 
                 const SimTK::Vec3& orientation2);
    /** This convenience constructor defines a bushing that behaves like a
      * primitive bushing.  Stiffnesses are used to define linear functions for
      * force deflection profiles.**/
	ExpressionBasedBushingForce(const std::string& body1Name, 
                 const SimTK::Vec3& point1, 
                 const SimTK::Vec3& orientation1,
		         const std::string& body2Name, 
                 const SimTK::Vec3& point2, 
                 const SimTK::Vec3& orientation2,
				 const SimTK::Vec3& transStiffness, 
                 const SimTK::Vec3& rotStiffness, 
                 const SimTK::Vec3& transDamping, 
                 const SimTK::Vec3& rotDamping);

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

	/** Set the name of the Body that will serve as body 1 for this bushing. **/
	void setBody1ByName(const std::string& aBodyName);
    /** Set the location and orientation (optional) for bushing frame on 
      * body 1. **/
	void setBody1BushingLocation(const SimTK::Vec3& location, 
                                 const SimTK::Vec3& orientation=SimTK::Vec3(0));
	/** Set the name of the Body that will serve as body 2 for this bushing. **/
	void setBody2ByName(const std::string& aBodyName);
    /** Set the location and orientation (optional) for bushing frame on 
      * body 2. **/
	void setBody2BushingLocation(const SimTK::Vec3& location, 
                                 const SimTK::Vec3& orientation=SimTK::Vec3(0));
    /** Set the value used to scale the bushing moment on body2 when drawing it to screen.  
      * A moment of magnitude |M| will be drawn on screen with a length of (|M|*scale).  **/
    void setMomentVisualScale(double scale) {set_moment_visual_scale(scale);};
    /** Set the value used to scale the bushing force on body2 when drawing it to screen.  
      * A force of magnitude |F| will be drawn on screen with a length of (|F|*scale).  **/
    void setForceVisualScale(double scale) {set_force_visual_scale(scale);}
    /** Set the aspect ratio used to control the thickness of the bushing force and moment
      * in drawn in the visualizer.  ratio = length/diameter.**/
    void setVisualAspectRatio(double ratio) {set_visual_aspect_ratio(ratio);}
	/** Set the expression defining Mx as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	void setMxExpression(std::string expression);
	/** Set the expression defining My as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	void setMyExpression(std::string expression);
	/** Set the expression defining Mz as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	void setMzExpression(std::string expression);
	/** Set the expression defining Fx as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	void setFxExpression(std::string expression);
	/** Set the expression defining Fy as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	void setFyExpression(std::string expression);
	/** Set the expression defining Fz as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	void setFzExpression(std::string expression);
	/** Get the expression defining Mx as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	std::string getMxExpression() { return get_Mx_expression(); }
	/** Get the expression defining My as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	std::string getMyExpression() { return get_My_expression(); }
	/** Get the expression defining Mz as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	std::string getMzExpression() { return get_Mz_expression(); }
	/** Get the expression defining Fx as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	std::string getFxExpression() { return get_Fx_expression(); }
	/** Get the expression defining Fy as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	std::string getFyExpression() { return get_Fy_expression(); }
	/** Get the expression defining Fz as a function of the bushing deflections theta_x, 
	  * theta_y, theta_z, delta_x, delta_y, delta_z **/
	std::string getFzExpression() { return get_Fz_expression(); }
	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	/** Compute the deflection (spatial separation) of the two frames connected
	  * by the bushing force. Angualar displacement expressed in Euler angles.
	  * The force and potential energy are determined by the deflection.  **/
	virtual SimTK::Vec6 computeDeflection(const SimTK::State& s) const;

	/** Compute the bushing force contribution to the system and add in to appropriate
	  * bodyForce and/or system generalizedForce. 
	  */
	virtual void computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

    /** Potential energy calculation is not implemented */

    

	//--------------------------------------------------------------------------
	// Reporting
	//--------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) 
     * to be reported.
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

protected:
    //--------------------------------------------------------------------------
    // Visual support in SimTK visualizer
    // -------------------------------------------------------------------------
    void generateDecorations(
        bool fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   geometryArray) const;
    
	void ComputeForcesAtBushing(const SimTK::State& state, 
                                SimTK::SpatialVec& forces_on_M_in_ground, 
                                SimTK::SpatialVec& forces_on_F_in_ground) const;

private:
	//--------------------------------------------------------------------------
	// Implement ModelComponent interface.
	//--------------------------------------------------------------------------
	void connectToModel(Model& aModel) OVERRIDE_11;
	// Create a SimTK::Force::LinarBushing which implements this ExpressionBasedBushingForce.
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    //--------------------------------------------------------------------------
	// Visible Object Support for Java Gui
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);
	virtual void updateGeometry(const SimTK::State& s);
	void setNull();
	void constructProperties();

//==============================================================================
};	// END of class ExpressionBasedBushingForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_EXPRESSION_BASED_BUSHING_FORCE_H_


