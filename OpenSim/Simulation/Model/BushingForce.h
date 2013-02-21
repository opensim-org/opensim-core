#ifndef OPENSIM_BUSHING_FORCE_H_
#define OPENSIM_BUSHING_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  BushingForce.h                          *
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


// INCLUDE
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Property.h>
#include "Force.h"

namespace OpenSim {

//==============================================================================
//                             BUSHING FORCE
//==============================================================================
/**
 * A class implementing a Bushing Force.
 * A Bushing Force is the force proportional to the deviation of two frames. 
 * One can think of the Bushing as being composed of 3 linear and 3 torsional
 * spring-dampers, which act along or about the bushing frames. Orientations
 * are measured as x-y-z body-fixed Euler rotations, which are treated as
 * though they were uncoupled. That makes this bushing model suitable only for
 * relatively small relative orientation between the frames.
 * The underlying Force in Simbody is a SimtK::Force::LinearBushing.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API BushingForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(BushingForce, Force);
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
	OpenSim_DECLARE_PROPERTY(rotational_stiffness, SimTK::Vec3,
		"Stiffness parameters resisting relative rotation (Nm/rad).");
	OpenSim_DECLARE_PROPERTY(translational_stiffness, SimTK::Vec3,
		"Stiffness parameters resisting relative translation (N/m).");
	OpenSim_DECLARE_PROPERTY(rotational_damping, SimTK::Vec3,
		"Damping parameters resisting relative angular velocity. (Nm/(rad/s))");
	OpenSim_DECLARE_PROPERTY(translational_damping, SimTK::Vec3,
		"Damping parameters resisting relative translational velocity. (N/(m/s)");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/** Default constructor leaves bodies unspecified, sets the bushing frames
    to be at their body origins, and sets all bushing parameters to zero. **/
	BushingForce();
    /** This constructor provides everything needed to create a real 
    bushing. See property declarations for more information. **/
	BushingForce(const std::string& body1Name, 
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
    body 1. **/
	void setBody1BushingLocation(const SimTK::Vec3& location, 
                                 const SimTK::Vec3& orientation=SimTK::Vec3(0));
	/** Set the name of the Body that will serve as body 2 for this bushing. **/
	void setBody2ByName(const std::string& aBodyName);
    /** Set the location and orientation (optional) for bushing frame on 
    body 2. **/
	void setBody2BushingLocation(const SimTK::Vec3& location, 
                                 const SimTK::Vec3& orientation=SimTK::Vec3(0));

	/** Potential energy is determine by the elastic energy storage of the 
    bushing. **/
	virtual double computePotentialEnergy(const SimTK::State& s) const;

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

private:
	//--------------------------------------------------------------------------
	// Implement ModelComponent interface.
	//--------------------------------------------------------------------------
	void connectToModel(Model& aModel) OVERRIDE_11;
	// Create a SimTK::Force::LinarBushing which implements this BushingForce.
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	void setNull();
	void constructProperties();

//==============================================================================
};	// END of class BushingForce
//==============================================================================
//==============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_BUSHING_FORCE_H_


