// PointActuator.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2009, Stanford University. All rights reserved. 
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

#include "PointActuator.h"

using namespace OpenSim;
using namespace std;
using SimTK::Vec3;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTORS
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

//_____________________________________________________________________________
/**
 * Also serves as default constructor.
 */
PointActuator::PointActuator(const string& bodyName)
{
	setNull();
    constructProperties();

	if (!bodyName.empty())
	    setProperty_body(bodyName);
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
// Set the data members of this actuator to their null values.
void PointActuator::setNull()
{
    // no data members
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void PointActuator::constructProperties()
{
	constructProperty_body();
	constructProperty_point(Vec3(0)); // origin
	constructProperty_point_is_global(false);
	constructProperty_direction(Vec3(1,0,0)); // arbitrary
	constructProperty_force_is_global(false);
	constructProperty_optimal_force(1.0);
}

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
// Set the generalized Body to which the Body actuator is applied, and set
// the body name property to match.
void PointActuator::setBody(Body* aBody)
{
	_body = aBody;
	if(aBody)
		setProperty_body(aBody->getName());
}
//_____________________________________________________________________________
// Get the generalized Body to which the Body actuator is applied.
Body* PointActuator::getBody() const
{
	return _body;
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
//Set the optimal force of the force.

void PointActuator::setOptimalForce(double aOptimalForce)
{
	setProperty_optimal_force(aOptimalForce);
}
//_____________________________________________________________________________
// Get the optimal force of the force.
double PointActuator::getOptimalForce() const
{
	return getProperty_optimal_force();
}
//_____________________________________________________________________________
// Get the stress of the force.
double PointActuator::getStress( const SimTK::State& s) const
{
	return std::abs(getForce(s) / getOptimalForce()); 
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double PointActuator::computeActuation( const SimTK::State& s ) const
{
	if(_model==NULL) return 0;

	// FORCE
	return getControl(s) * getOptimalForce();
}



//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void PointActuator::computeForce(const SimTK::State& s, 
							     SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							     SimTK::Vector& generalizedForces) const
{
	const SimbodyEngine& engine = getModel().getSimbodyEngine();

	if( !_model || !_body ) return;

    double force;

    if( isForceOverriden(s) ) {
       force = computeOverrideForce(s);
    } else {
       force = computeActuation(s);
    }
    setForce(s,  force );

	
	Vec3 forceVec = force*SimTK::UnitVec3(getProperty_direction());
	Vec3 lpoint = getProperty_point();
	if (!getProperty_force_is_global())
		engine.transform(s, *_body, forceVec, 
                         engine.getGroundBody(), forceVec);
	if (getProperty_point_is_global())
        engine.transformPosition(s, engine.getGroundBody(), lpoint, 
                                 *_body, lpoint);
	applyForceToPoint(s, *_body, lpoint, forceVec, bodyForces);

	// get the velocity of the actuator in ground
	Vec3 velocity(0);
	engine.getVelocity(s, *_body, lpoint, velocity);

	// the speed of the point is the "speed" of the actuator used to compute 
    // power
	setSpeed(s, velocity.norm());
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body reference _body
 */
void PointActuator::setup(Model& model)
{
	Super::setup(model);

	string errorMessage;

	const string& bodyName = getProperty_body();

	if (!model.updBodySet().contains(bodyName)) {
		errorMessage = "PointActuator: Unknown body (" + bodyName 
                        + ") specified in Actuator " + getName();
		throw OpenSim::Exception(errorMessage);
	}

    _body = &model.updBodySet().get(bodyName);
}


//=============================================================================
// XML
//=============================================================================
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
void PointActuator::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	int documentVersion = versionNumber;
	bool converting=false;
	if ( documentVersion < XMLDocument::getLatestVersion()){
		if (documentVersion<10905){
			// This used to be called "Force" back then
			XMLDocument::renameChildNode(aNode, "body_B", "body"); // body_B -> body
			XMLDocument::renameChildNode(aNode, "point_B", "point"); // point_B -> point
			XMLDocument::renameChildNode(aNode, "direction_A", "direction"); // direction_A -> direction
			setProperty_force_is_global(true);
			converting = true;
		}
	}
	Super::updateFromXMLNode(aNode, versionNumber);
	if (converting) updProperty_direction(0) *= -1.0;

	if (_model && !getProperty_body().empty()) {
        const std::string& bodyName = getProperty_body();
		_body = &_model->updBodySet().get(bodyName);
    }
}	

