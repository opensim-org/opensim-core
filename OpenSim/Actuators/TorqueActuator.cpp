// TorqueActuator.cpp
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
 * Author: Ajay Seth
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

#include "TorqueActuator.h"

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
TorqueActuator::TorqueActuator()
{
	constructProperties();
}
//_____________________________________________________________________________
// Constructor with given body names.
TorqueActuator::TorqueActuator(const string& bodyNameA, 
                               const string& bodyNameB)
{
	constructProperties();

    if (!bodyNameA.empty()) set_bodyA(bodyNameA);
    if (!bodyNameB.empty()) set_bodyB(bodyNameB);
}

//_____________________________________________________________________________
// Construct and initialize properties.
void TorqueActuator::constructProperties()
{
    constructProperty_bodyA();
    constructProperty_bodyB();
    constructProperty_torque_is_global(true);
	constructProperty_axis(Vec3(0,0,1)); // z direction
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
void TorqueActuator::setBodyA(Body* aBody)
{
	_bodyA = aBody;
	if(aBody)
		set_bodyA(aBody->getName());
}
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the equal and opposite Body actuation 
 * is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void TorqueActuator::setBodyB(Body* aBody)
{
	_bodyB = aBody;
	if(aBody)
		set_bodyB(aBody->getName());
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
// Calculate the stress of the force.
double TorqueActuator::getStress(const State& s) const
{
	return std::abs(getForce(s) / getOptimalForce()); 
}
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double TorqueActuator::computeActuation(const State& s) const
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
 */
void TorqueActuator::computeForce(const State& s, 
							      Vector_<SpatialVec>& bodyForces, 
							      Vector& generalizedForces) const
{
	if(_model==NULL) return;
	const SimbodyEngine& engine = getModel().getSimbodyEngine();

	const bool torqueIsGlobal = getTorqueIsGlobal();
	const Vec3& axis = getAxis();
	
    double force;

    if( isForceOverriden(s) ) {
       force = computeOverrideForce(s);
    } else {
       force = computeActuation(s);
    }
    setForce(s,  force );

	if(!_bodyA)
		return;
	

    setForce(s, force );
	Vec3 torque = force*UnitVec3(axis);
	
	if (!torqueIsGlobal)
		engine.transform(s, *_bodyA, torque, engine.getGroundBody(), torque);
	
	applyTorque(s, *_bodyA, torque, bodyForces);

	// if bodyB is not specified, use the ground body by default
	if(!_bodyB)
		applyTorque(s, *_bodyB, -torque, bodyForces);

	// get the angular velocity of the body in ground
	Vec3 omega(0);
	engine.getAngularVelocity(s, *_bodyA, omega);
	// the speed of the body about the axis the torque is applied is the "speed" of the actuator used to compute power
	setSpeed(s, ~omega*axis);
}
//_____________________________________________________________________________
/**
 * Sets the actual Body references _bodyA and _bodyB
 */
void TorqueActuator::connectToModel(Model& model)
{
	Super::connectToModel(model);

    if (get_bodyA().empty() || get_bodyB().empty())
        throw OpenSim::Exception(
            "TorqueActuator::connectToModel(): body name properties "
            "were not set.");

    // Look up the bodies by name in the Model, and record pointers to the
    // corresponding body objects.
	_bodyA = model.updBodySet().get(get_bodyA());
	_bodyB = model.updBodySet().get(get_bodyB());
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
void TorqueActuator::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	int documentVersion = versionNumber;
	bool converting=false;
	if ( documentVersion < XMLDocument::getLatestVersion()){
		if (documentVersion<10905){
			// This used to be called "Force" back then
			XMLDocument::renameChildNode(aNode, "body_A", "bodyB"); // body_B -> body
			XMLDocument::renameChildNode(aNode, "body_B", "bodyA"); // direction_A -> direction
			XMLDocument::renameChildNode(aNode, "direction_A", "axis"); // direction_A -> direction
			converting = true;
		}
	}
	Super::updateFromXMLNode(aNode, versionNumber);
	if (converting) upd_axis() *= -1.0;
}	

