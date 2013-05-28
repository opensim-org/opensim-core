#ifndef OPENSIM_POINT_TO_POINT_SPRING_H_
#define OPENSIM_POINT_TO_POINT_SPRING_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  PointToPointSpring.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//==============================================================================
//                          POINT TO POINT SPRING
//==============================================================================
/**
 * A simple point to point spring with a resting length and stiffness.
 * Points are connected to bodies and are defined in the body frame.
 *
 * @author Ajay Seth
 */
namespace OpenSim { 

class OSIMSIMULATION_API PointToPointSpring : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(PointToPointSpring, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(body1, std::string,
		"Name of Body to which 1 end of the spring is attached.");
	OpenSim_DECLARE_OPTIONAL_PROPERTY(body2, std::string,
		"Name of Body to which the 2nd end of the spring is attached.");
	OpenSim_DECLARE_PROPERTY(point1, SimTK::Vec3,
		"Force application point on body1.");
	OpenSim_DECLARE_PROPERTY(point2, SimTK::Vec3,
		"Force application point on body2.");
	OpenSim_DECLARE_PROPERTY(stiffness, double,
		"Spring stiffness (N/m).");
	OpenSim_DECLARE_PROPERTY(rest_length, double,
		"Spring resting length.");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor. **/
	PointToPointSpring();
    /** Convenience constructor for API users.
    @param body1Name    name of the first body to which the spring is attached
    @param point1       location where spring is attached on body1
    @param body2Name    name of the second body to which the spring is attached
    @param point2       location where spring is attached on body2 
    @param stiffness    spring stiffness
    @param restlength   the resting (zero force) length of the spring
    **/
	PointToPointSpring( std::string body1Name, SimTK::Vec3 point1, 
		                std::string body2Name, SimTK::Vec3 point2, 
                        double stiffness, double restlength );

    // default destructor, copy constructor, copy assignment

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);
	virtual void updateGeometry(const SimTK::State& s);
	
	//-----------------------------------------------------------------------------
	// GET and SET Spring parameters
	//-----------------------------------------------------------------------------
	/**
	* Spring end point bodies 
	*/
	void setBody1Name(const std::string& body1Name) 
    {   set_body1(body1Name); }
	void setBody2Name(const std::string& body2Name) 
    {   set_body2(body2Name); }
	const std::string& getBody1Name() const {return get_body1();}
	const std::string& getBody2Name() const {return get_body2();}

	/**
	* Spring end points 
	*/
	void setPoint1(SimTK::Vec3 aPosition) { set_point1(aPosition); }
	const SimTK::Vec3& getPoint1() const { return get_point1(); }
	void setPoint2(SimTK::Vec3 aPosition) { set_point2(aPosition); }
	const SimTK::Vec3& getPoint2() const { return get_point2(); }

	/**
	* Spring stiffness
	* @param stiffness 
	*/
	void setStiffness(double stiffness) {set_stiffness(stiffness);}
	double getStiffness() const {return get_stiffness();}
	/**
	* Spring resting length
	* @param restLength 
	*/
	void setRestlength(double restLength) {set_rest_length(restLength);}
	double getRestlength() const {return get_rest_length();}

	//-----------------------------------------------------------------------------
	// ModelComponent interface
	//-----------------------------------------------------------------------------
	void connectToModel(Model& model) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	//-----------------------------------------------------------------------------
	// Reporting
	//-----------------------------------------------------------------------------
	/** 
	 * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const ;
	/**
	*  Provide the value(s) to be reported that correspond to the labels
	*/
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;

protected:
	/** how to display the Spring */
	VisibleObject _displayer;

private:
	void setNull();
	void constructProperties();

//==============================================================================
};	// END of class PointToPointSpring

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_POINT_TO_POINT_SPRING_H_
