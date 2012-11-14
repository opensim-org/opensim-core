#ifndef __PointOnLineConstraint_h__
#define __PointOnLineConstraint_h__
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  PointOnLineConstraint.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Samuel R. Hamner                                                *
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
#include "Constraint.h"
#include "Body.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Point On Line Constraint.  The underlying Constraint 
 * in Simbody is a Constraint::PointOnLine
 *
 * @author Samuel Hamner
 * @version 1.0
 */
class OSIMSIMULATION_API PointOnLineConstraint : public Constraint {
OpenSim_DECLARE_CONCRETE_OBJECT(PointOnLineConstraint, Constraint);

//=============================================================================
// DATA
//=============================================================================
protected:

	OpenSim_DECLARE_PROPERTY(line_body, std::string, "Specify the body on which the line is defined");

	OpenSim_DECLARE_PROPERTY(line_direction_vec, SimTK::Vec3, "Direction of the line in line body specified in the line body frame.");

	OpenSim_DECLARE_PROPERTY(point_on_line, SimTK::Vec3, "Specify the default point on the line in the line body frame.");
	
	OpenSim_DECLARE_PROPERTY(follower_body, std::string, "Specify the follower body constrained to the line.");

	OpenSim_DECLARE_PROPERTY(point_on_follower, SimTK::Vec3, "Specify the  point on the follower bocy constrained to the line in the follower body reference frame.");

	/** Line body */
	Body *_lineBody;

	/** Follower body */
	Body *_followerBody;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	PointOnLineConstraint();
	PointOnLineConstraint(OpenSim::Body& lineBody, SimTK::Vec3 lineDirection, SimTK::Vec3 pointOnLine,
		OpenSim::Body& followerBody, SimTK::Vec3 followerPoint);
	virtual ~PointOnLineConstraint();

	//SET 
	void setLineBodyByName(std::string aBodyName);
	void setFollowerBodyByName(std::string aBodyName);
	void setLineDirection(SimTK::Vec3 direction);
	void setPointOnLine(SimTK::Vec3 point);
	void setPointOnFollower(SimTK::Vec3 point);

protected:
	void connectToModel(Model& aModel) OVERRIDE_11;
	/**
	 * Create a SimTK::Constraint::PointOnLine which implements this constraint.
	 */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;


private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class PointOnLineConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PointOnLineConstraint_h__


