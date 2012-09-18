#ifndef OPENSIM_PATH_ACTUATOR_H_
#define OPENSIM_PATH_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathActuator.h                          *
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

#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "Actuator.h"
#include "GeometryPath.h"

//=============================================================================
//=============================================================================

namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

/**
 * This is the base class for actuators that apply controllable tension along 
 * a geometry path. %PathActuator has no states; the control is simply the 
 * tension to be applied along a geometry path (i.e. tensionable rope).
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API PathActuator : public Actuator {
OpenSim_DECLARE_CONCRETE_OBJECT(PathActuator, Actuator);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations
    These are the serializable properties associated with the %PathActuator
    class. Note that objects derived from this class inherit these
    properties. **/
    /**@{**/
    OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath,
		"The set of points defining the path of the muscle.");
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum force this actuator can produce.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================
	PathActuator();

    // default destructor, copy constructor, copy assignment

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// Path
	GeometryPath& updGeometryPath() { return upd_GeometryPath(); }
	const GeometryPath& getGeometryPath() const 
    {   return get_GeometryPath(); }
	virtual bool hasGeometryPath() const { return true;};

	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;

	// Length and Speed of actuator
	virtual double getLength(const SimTK::State& s) const;
	virtual double getLengtheningSpeed(const SimTK::State& s) const;

	// Power: Since lengthening is positive and tension always shortens, positive power
	// is when muscle is shortening under tension.
	virtual double getPower(const SimTK::State& s) const 
    {   return -getForce(s)*getSpeed(s); }


	// STRESS
	virtual double getStress( const SimTK::State& s ) const;

    // Convenience method to add PathPoints
	 /** Note that this function does not maintain the State and so should be used only
		before a valid State is created */
	 void addNewPathPoint( const std::string& proposedName, OpenSim::Body& aBody, 
						   const SimTK::Vec3& aPositionOnBody);

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce( const SimTK::State& state, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s) const;
	virtual double computeMomentArm( const SimTK::State& s, Coordinate& aCoord) const;

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

	//--------------------------------------------------------------------------
	// Visible Object Support
	//--------------------------------------------------------------------------
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);

protected:
	// Setup method to initialize coordinate reference
	void connectToModel(Model& aModel) OVERRIDE_11;


private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class PathActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PATH_ACTUATOR_H_


