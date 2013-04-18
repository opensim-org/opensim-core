#ifndef __PointForceDirection_h__
#define __PointForceDirection_h__
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  PointForceDirection.h                       *
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

// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

class Body;
//=============================================================================
//=============================================================================
/** Convenience class for a generic representation of geometery of a complex
    Force (or any other object) with multiple points of contact through
	which forces are applied to bodies. This represents one such point and an
	array of these objects defines a complete Force distrbitution (ie. path).
 *
 * @author Ajay Seth
 * @version 1.0
 */

class OSIMSIMULATION_API PointForceDirection
{

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	/** Point of "contact" with a body, defined in the body frame */
	SimTK::Vec3 _point;
	/** The body in which the point is defined */
	const Body &_body;
	/** Direction of the force at the point, defined in ground */
	SimTK::Vec3 _direction;
	/** Optional parameter to scale the force that results from a scalar 
	    (tension) multiplies the direction */
	double _scale;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~PointForceDirection() {};
	/** Default constructor takes the point, body, direction and scale
	    as arguments */
	PointForceDirection(SimTK::Vec3 point, Body &body, SimTK::Vec3 direction, double scale=1):
		_point(point), _direction(direction), _body(body), _scale(scale) 
	{};

	/** get point of "conact" with on a body defined in the body frame */
	SimTK::Vec3 point() {return _point; };
	/** get the body in which the point is defined */
	const Body& body() {return _body; };
	/** get direction of the force at the point defined in ground */
	SimTK::Vec3 direction() {return _direction; };
	/** get the scale factor on the force */
	double scale() {return _scale; };

	/** replace the current direction with the resultant with a new direction */
	void addToDirection(SimTK::Vec3 newDirection) {_direction+=newDirection;}

//=============================================================================
};	// END of class PointForceDirection
//=============================================================================
} // namespace

#endif // __PointForceDirection_h__
