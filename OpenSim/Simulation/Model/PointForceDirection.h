#ifndef __PointForceDirection_h__
#define __PointForceDirection_h__

// PointForceDirection.h
// Authors: Ajay Seth
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

class Body;
//=============================================================================
//=============================================================================
/** Convenience class for a generic representation of geomtery of a complex
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
	/** Point of "conact" with on a body defined in the body frame */
	SimTK::Vec3 _point;
	/** The body in which the point isdefined */
	const Body &_body;
	/** Direction of the force at the point defined in ground */
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
