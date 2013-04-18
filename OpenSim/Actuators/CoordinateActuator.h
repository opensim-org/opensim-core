#ifndef OPENSIM_COORDINATE_ACTUATOR_H_
#define OPENSIM_COORDINATE_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  CoordinateActuator.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
 * Contributor(s): Frank C. Anderson                                          *
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
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Simulation/Model/Actuator.h>

namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

//==============================================================================
//                           COORDINATE ACTUATOR
//==============================================================================
/**
 * An actuator that applies a generalized force to along a generalized
 * a generalized coordinate, which is proportional to its input control.
 * It replaces the GeneralizeForce class implemented by Frank C. Anderson
 *
 * @author Ajay Seth
 * @author Frank C. Anderson
 */
class OSIMACTUATORS_API CoordinateActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateActuator, Actuator);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate, std::string,
		"Name of the generalized coordinate to which the actuator applies.");
	OpenSim_DECLARE_PROPERTY(optimal_force, double,
		"The maximum generalized force produced by this actuator.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
    /** Default constructor leaves coordinate name unspecified, or you can
    provide it. **/
	explicit CoordinateActuator(const std::string& coordinateName="");

    // Uses default (compiler-generated) destructor, copy constructor, copy 
    // assignment operator.

	/** Set the 'optimal_force' property. **/
	void setOptimalForce(double optimalForce);
    /** Get the current setting of the 'optimal_force' property. **/
	double getOptimalForce() const OVERRIDE_11; // part of Actuator interface

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static ForceSet* CreateForceSetOfCoordinateActuatorsForModel(const SimTK::State& s, Model& aModel,double aOptimalForce = 1,bool aIncludeLockedAndConstrainedCoordinates = true);

	bool isCoordinateValid() const;
	double getSpeed( const SimTK::State& s) const;

    /** Set the reference pointer to point to the given Coordinate and set
    the 'coordinate' name property also. **/
    void setCoordinate(Coordinate* aCoordinate);
    /** Get a pointer to the Coordinate to which this actuator refers. **/
	Coordinate* getCoordinate() const;

//==============================================================================
// PRIVATE
//==============================================================================
private:
	//--------------------------------------------------------------------------
	// Implement Force interface
	//--------------------------------------------------------------------------
	void computeForce(const SimTK::State& state, 
					  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
					  SimTK::Vector& mobilityForces) const OVERRIDE_11;


	//--------------------------------------------------------------------------
	// Implement Actuator interface (also see getOptimalForce() above)
	//--------------------------------------------------------------------------
	double  computeActuation( const SimTK::State& s) const OVERRIDE_11;
	// Return the stress, defined as abs(force/optimal_force).
	double getStress( const SimTK::State& s ) const OVERRIDE_11;


	//--------------------------------------------------------------------------
	// Implement ModelComponent interface
	//--------------------------------------------------------------------------
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

	//--------------------------------------------------------------------------
	// Implement Object interface.
	//--------------------------------------------------------------------------
	void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1)
        OVERRIDE_11;

	void setNull();
	void constructProperties();


    // Note: reference pointers are automatically set to null on construction 
    // and also on copy construction and copy assignment.

	// Corresponding generalized coordinate to which the coordinate actuator
    // is applied.
    SimTK::ReferencePtr<Coordinate> _coord;
//==============================================================================
};	// END of class CoordinateActuator

}; //namespace
//==============================================================================
//==============================================================================

#endif // OPENSIM_COORDINATE_ACTUATOR_H_


