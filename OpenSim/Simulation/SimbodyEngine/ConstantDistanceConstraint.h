#ifndef __ConstantDistanceConstraint_h__
#define __ConstantDistanceConstraint_h__
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  ConstantDistanceConstraint.h                   *
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
#include <OpenSim/Common/PropertyDbl.h>
#include "Constraint.h"
#include "Body.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a constraint that maintains a constant distance between
 * between two points.  The underlying Constraint in Simbody
 * is a Constraint::Rod
 *
 * @author Matt DeMers
 * @version 1.0
 */
class OSIMSIMULATION_API ConstantDistanceConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstantDistanceConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
public:
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
	OpenSim_DECLARE_PROPERTY(body_1, std::string,
		"Specify first of two bodies connected together by the constraint.");
	OpenSim_DECLARE_PROPERTY(body_2, std::string,
		"Specify second of two bodies connected together by the constraint.");
	OpenSim_DECLARE_PROPERTY(location_body_1, SimTK::Vec3,
        "Location of the point in first body specified in body1 "
        "reference frame.");
	OpenSim_DECLARE_PROPERTY(location_body_2, SimTK::Vec3,
        "Location of the point in second body specified in body2 "
        "reference frame.");
	OpenSim_DECLARE_PROPERTY(constant_distance, double, "constant distance "
        "to be rigidly maintained between the two points "
        "fixed on each body.");
    /**@}**/

protected:
	/** First body point constraint joins. */
	Body *_body1;

	/** Second body point constraint joins. */
	Body *_body2;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	ConstantDistanceConstraint();
	ConstantDistanceConstraint(OpenSim::Body& body1, SimTK::Vec3& locationBody1, OpenSim::Body& body2, SimTK::Vec3& locationBody2, double& distance);
	virtual ~ConstantDistanceConstraint();
	
	//SET 
	void setBody1ByName(std::string aBodyName);
	void setBody1PointLocation(SimTK::Vec3 location);
	void setBody2ByName(std::string aBodyName);
	void setBody2PointLocation(SimTK::Vec3 location);
	void setConstantDistance(double distance);

	/** Method to set point location of contact during an induced acceleration analysis */
	//virtual void setContactPointForInducedAccelerations(const SimTK::State &s, SimTK::Vec3 point);


protected:
	void connectToModel(Model& aModel) OVERRIDE_11;
	/**
	 * Create a SimTK::Constraint::Ball which implements this Point constraint.
	 */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class ConstantDistanceConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ConstantDistanceConstraint_h__


