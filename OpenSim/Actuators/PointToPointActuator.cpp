// PointToPointActuator.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009-12, Stanford University. All rights reserved. 
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

/* 
 * Author: Matt DeMers
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include "PointToPointActuator.h"

using namespace OpenSim;
using std::string;
using SimTK::Vec3; using SimTK::Vector_; using SimTK::Vector; 
using SimTK::SpatialVec; using SimTK::UnitVec3; using SimTK::State;

//==============================================================================
// CONSTRUCTOR(S)
//==============================================================================
// default destructor and copy construction & assignment
//_____________________________________________________________________________
// Default constructor.
PointToPointActuator::PointToPointActuator()
{
	constructProperties();
}
//_____________________________________________________________________________
// Constructor with given body names.
PointToPointActuator::PointToPointActuator(const string& bodyNameA, 
                                           const string& bodyNameB)
{
	constructProperties();

    if (!bodyNameA.empty()) setProperty_bodyA(bodyNameA);
    if (!bodyNameB.empty()) setProperty_bodyB(bodyNameB);
}

//_____________________________________________________________________________
// Construct and initialize properties.
void PointToPointActuator::constructProperties()
{
    constructProperty_bodyA();
    constructProperty_bodyB();
    constructProperty_points_are_global(false);
	constructProperty_pointA(Vec3(0));  // origin
	constructProperty_pointB(Vec3(0));
    constructProperty_optimal_force(1.0);
}


//==============================================================================
// GET AND SET
//==============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the Body actuator is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void PointToPointActuator::setBodyA(Body* aBody)
{
	_bodyA = aBody;
	if(aBody)
		setProperty_bodyA(aBody->getName());
}
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the equal and opposite Body actuation 
 * is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void PointToPointActuator::setBodyB(Body* aBody)
{
	_bodyB = aBody;
	if(aBody)
		setProperty_bodyB(aBody->getName());
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PointToPointActuator::getStress( const SimTK::State& s) const
{
	return std::abs(getForce(s) / getOptimalForce()); 
}
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 *
 * @param s current SimTK::State 
 */

double PointToPointActuator::computeActuation( const SimTK::State& s ) const
{
	if(_model==NULL) return 0;

	// FORCE
	return getControl(s) * getOptimalForce();
}



//==============================================================================
// APPLICATION
//==============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 *
 * @param s current SimTK::State
 */
void PointToPointActuator::computeForce(const SimTK::State& s, 
							    SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							    SimTK::Vector& generalizedForces) const
{
	const bool pointsAreGlobal = getPointsAreGlobal();
	const SimTK::Vec3& pointA = getPointA();
	const SimTK::Vec3& pointB = getPointB();

	if(_model==NULL) return;
	const SimbodyEngine& engine = getModel().getSimbodyEngine();
	
	if(_bodyA ==NULL || _bodyB ==NULL)
		return;
	
	// Get pointA and pointB positions in both the global frame, and in 
    // the local frame of bodyA and bodyB, respectively. Points may have
    // been supplied either way.

	SimTK::Vec3 pointA_inGround, pointB_inGround, 
                pointA_inBodyA, pointB_inBodyB;

	if (pointsAreGlobal)
	{
		pointA_inGround = pointA;
		pointB_inGround = pointB;
		engine.transformPosition(s, engine.getGroundBody(), pointA_inGround, 
                                 *_bodyA, pointA_inBodyA);
		engine.transformPosition(s, engine.getGroundBody(), pointB_inGround, 
                                 *_bodyB, pointB_inBodyB);
	}
	else
	{
		pointA_inBodyA = pointA;
		pointB_inBodyB = pointB;
		engine.transformPosition(s, *_bodyA, pointA_inBodyA, 
                                 engine.getGroundBody(), pointA_inGround);
		engine.transformPosition(s, *_bodyB, pointB_inBodyB, 
                                 engine.getGroundBody(), pointB_inGround);
	}

	// Find the direction along which the actuator applies its force.
    // NOTE: this will fail if the points are coincident.
	const SimTK::Vec3 r = pointA_inGround - pointB_inGround;
	const SimTK::UnitVec3 direction(r); // normalize

	// Find the force magnitude and set it. Then form the force vector.
	double forceMagnitude;

    if( isForceOverriden(s) ) {
       forceMagnitude = computeOverrideForce(s);
    } else {
       forceMagnitude = computeActuation(s);
    }
    setForce(s,  forceMagnitude );

	const SimTK::Vec3 force = forceMagnitude*direction;

	// Apply equal and opposite forces to the bodies.
	applyForceToPoint(s, *_bodyA, pointA_inBodyA, force, bodyForces);
	applyForceToPoint(s, *_bodyB, pointB_inBodyB, -force, bodyForces);

	// Get the relative velocity of the points in ground.
	SimTK::Vec3 velA_G, velB_G, velAB_G;
	engine.getVelocity(s, *_bodyA, pointA_inBodyA, velA_G);
	engine.getVelocity(s, *_bodyB, pointB_inBodyB, velB_G);
	velAB_G = velA_G-velB_G;
	// Speed used to compute power is the speed along the line connecting 
    // the two bodies.
	setSpeed(s, ~velAB_G*direction);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body references _bodyA and _bodyB
 */
void PointToPointActuator::setup(Model& model)
{
	Super::setup(model);

    if (getProperty_bodyA().empty() || getProperty_bodyB().empty())
        throw OpenSim::Exception(
            "PointToPointActuator::setup(): body name properties were not set.");

    // Look up the bodies by name in the Model, and record pointers to the
    // corresponding body objects.
	_bodyA = &updModel().updBodySet().get(getProperty_bodyA());
	_bodyB = &updModel().updBodySet().get(getProperty_bodyB());
}

//==============================================================================
// XML
//==============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 *
 * This method simply calls Object::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void PointToPointActuator::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	Super::updateFromXMLNode(aNode, versionNumber);

    // Look up the bodies by name in the Model, and record pointers to the
    // corresponding body objects.
    if (!(getProperty_bodyA().empty()|| getProperty_bodyB().empty())) {
	    _bodyA = &updModel().updBodySet().get(getProperty_bodyA());
	    _bodyB = &updModel().updBodySet().get(getProperty_bodyB());
    }
}	
