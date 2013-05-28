#ifndef OPENSIM_EXPRESSION_BASED_POINT_TO_POINT_FORCE_H_
#define OPENSIM_EXPRESSION_BASED_POINT_TO_POINT_FORCE_H_
/* -------------------------------------------------------------------------- *
 *              OpenSim:  ExpressionBasedPointToPointForce.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
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

#include "Force.h"
#include <OpenSim/Common/VisibleObject.h>
#include <Vendors/lepton/include/Lepton.h>

//==============================================================================
//                    EXPRESSION BASED POINT TO POINT FORCE
//==============================================================================
/**
 * A point-to-point Force who's force magnitude is determined by a user-defined
 * expression, with the distance (d) and its time derivative (ddot) as variables. 
 * The direction of the force is directed along the line connecting the two 
 * points. 
 *
 * "d" and "ddot" are the variables names expected by the expression parser.
 * Common C math library functions such as: exp(), pow(), sqrt(), sin(), ...
 * are permitted. See Lepton/Operation.h for a complete list.
 * 
 * For example: string expression = "-1.5*exp(10*(d-0.25)^2)*(1 + 2.0*ddot)"
 *				provides a model of a nonlinear point-to point spring, while 
 *				expression = "1.25/(rd^2)" is an electric field force between
 *				charged particles at points separated by the distance, d.
 *              i.e. K*q1*q2 = 1.25
 *
 * @author Ajay Seth
 */
namespace OpenSim { 

class OSIMSIMULATION_API ExpressionBasedPointToPointForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedPointToPointForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(body1, std::string,
		"Name of the Body to which the 1st point of the force is attached.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body2, std::string,
		"Name of the Body to which the 2nd point of the force is attached.");
	OpenSim_DECLARE_PROPERTY(point1, SimTK::Vec3,
		"Force application point on body1.");
	OpenSim_DECLARE_PROPERTY(point2, SimTK::Vec3,
		"Force application point on body2.");
    OpenSim_DECLARE_PROPERTY(expression, std::string,
		"Expression of the point-to-point force magnitude as a function of "
		"the distance (d) between the points and its time derivative (ddot). "
		"Note, expression cannot have any whitespace separating characters.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor. **/
	ExpressionBasedPointToPointForce();
    /** Convenience constructor for API users.
    @param body1Name    name of the first body to which the p2p force is attached
    @param point1       first point location attached to body1
    @param body2Name    name of the second body to which the p2p force is attached
    @param point2       second  point location attached to body2
	@param expression	the expression used to compute the force action at points
    **/
	ExpressionBasedPointToPointForce(
				const std::string& body1Name, const SimTK::Vec3& point1, 
		        const std::string& body2Name, const SimTK::Vec3& point2, 
                const std::string& expression );

    // default destructor, copy constructor, copy assignment
	
	//-----------------------------------------------------------------------------
	// GET and SET Spring parameters
	//-----------------------------------------------------------------------------
	/**
	* Force end point bodies 
	*/
	void setBody1Name(const std::string& body1Name) 
    {   set_body1(body1Name); }
	void setBody2Name(const std::string& body2Name) 
    {   set_body2(body2Name); }
	const std::string& getBody1Name() const {return get_body1();}
	const std::string& getBody2Name() const {return get_body2();}

	/**
	* Force end points 
	*/
	void setPoint1(SimTK::Vec3 aPosition) { set_point1(aPosition); }
	const SimTK::Vec3& getPoint1() const { return get_point1(); }
	void setPoint2(SimTK::Vec3 aPosition) { set_point2(aPosition); }
	const SimTK::Vec3& getPoint2() const { return get_point2(); }

	/**
	* Set the mathematical expression that defines the force magnitude of this
	* point-to-point force in terms of the point-to-point distance (d) and its
	* time derivative (ddot). Expressions with C-mathematical operations
	* such as +,-,*,/ and common functions: exp, pow, sqrt, sin, cos, tan, 
	* and so on are acceptable.
	* NOTE: a limitation is that the expression may not contain whitespace
	* @param expression    string containing the mathematical expression that
	*					   defines the point-to-point force 
	*/
	void setExpression(const std::string& expression);


	/** 
	* Get the computed Force magnitude determined by evaluating the 
	* expression above. Note, computeForce must be evaluated first,
	* and this is done automatically if the system is realized to Dynamics
	* @param state    const state (reference) for the model
	* @return         const double ref to the force magnitude
	*/
	const double& getForceMagnitude(const SimTK::State& state);


	//--------------------------------------------------------------------------
	// COMPUTATION
	//--------------------------------------------------------------------------
	/** Compute the point-to-point force based on the user-defined expression 
	    and apply it to the model */
	void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const OVERRIDE_11;


	//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	OpenSim::Array<std::string> getRecordLabels() const OVERRIDE_11;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	OpenSim::Array<double> getRecordValues(const SimTK::State& state) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	VisibleObject* getDisplayer() const;
	void updateDisplayer(const SimTK::State& s);
	void updateGeometry(const SimTK::State& s);


protected:

	//-----------------------------------------------------------------------------
	// ModelComponent interface
	//-----------------------------------------------------------------------------
	void connectToModel(Model& model) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	/** how to display the Spring */
	VisibleObject _displayer;

private:
	void setNull();
	void constructProperties();

	// parser programs for efficiently evaluating the expressions
	Lepton::ExpressionProgram _forceProg;

	SimTK::ReferencePtr<const SimTK::MobilizedBody> _b1; 
	SimTK::ReferencePtr<const SimTK::MobilizedBody> _b2;

//==============================================================================
};	// END of class OVERRIDE_11

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_EXPRESSION_BASED_POINT_TO_POINT_FORCE_H_
