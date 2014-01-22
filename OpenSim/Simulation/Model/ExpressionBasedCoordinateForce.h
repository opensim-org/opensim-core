#ifndef OPENSIM_EXPRESSION_BASED_COORDINATE_FORCE_H_
#define OPENSIM_EXPRESSION_BASED_COORDINATE_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                OpenSim:  ExpressionBasedCoordinateForce.h                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Nabeel Allana                                                   *
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
#include "Force.h"
#include <OpenSim/Common/VisibleObject.h>
#include <Vendors/lepton/include/Lepton.h>

namespace OpenSim {

class OSIMSIMULATION_API ExpressionBasedCoordinateForce : public Force
{
OpenSim_DECLARE_CONCRETE_OBJECT(ExpressionBasedCoordinateForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(coordinate, std::string,
        "Coordinate (name) to apply force to.");
    OpenSim_DECLARE_PROPERTY(expression, std::string,
		"Expression of the force magnitude as a function of the coordinate value (q)"
        "and its time derivative (qdot). Note, expression cannot have any whitespace"
		"seperating characters");
    /**@}**/
//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor. **/
	ExpressionBasedCoordinateForce();
    /** Convenience constructor for API users.
    @param coordinate   name of the coordinate to apply the force to
    @param expression   the expression used to compute the force magnitude
    **/
	ExpressionBasedCoordinateForce(
				const std::string& coordinate, const std::string& expression);
//==============================================================================
// GET and SET parameters
//==============================================================================
	/**
	* Coordinate
	*/
	void setCoordinateName(const std::string& coord) 
    {   set_coordinate(coord); }
	const std::string& getCoordinateName() const {return get_coordinate();}

	/**
	* Set the mathematical expression that defines the force magnitude of this
	* coordinate force in terms of the coordinate value (q) and its
	* time derivative (qdot). Expressions with C-mathematical operations
	* such as +,-,*,/ and common functions: exp, pow, sqrt, sin, cos, tan, 
	* and so on are acceptable.
	* NOTE: a limitation is that the expression may not contain whitespace
	* @param expression    string containing the mathematical expression that
	*					   defines the coordinate force 
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


//==============================================================================
// COMPUTATION
//==============================================================================
	/** Compute the coordinate force based on the user-defined expression 
	    and apply it to the model */
    void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const OVERRIDE_11;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    /** Force calculation operator. **/
    double calcExpressionForce( const SimTK::State& s) const;

//==============================================================================
// Reporting
//==============================================================================
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	OpenSim::Array<std::string> getRecordLabels() const OVERRIDE_11;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	OpenSim::Array<double> getRecordValues(const SimTK::State& state) const OVERRIDE_11;

	

protected:
//==============================================================================
// ModelComponent interface
//==============================================================================
	void connectToModel(Model& model) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;


private:
	void setNull();
	void constructProperties();

	// parser programs for efficiently evaluating the expressions
	Lepton::ExpressionProgram _forceProg;

    // Corresponding generalized coordinate to which the force
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;

}; //  class ExpressionBasedCoordinateForce

}; // namespace OpenSim

#endif // OPENSIM_EXPRESSION_BASED_COORDINATE_FORCE_H_