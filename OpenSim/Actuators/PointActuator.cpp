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
#include "PointActuator.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PointActuator::~PointActuator()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PointActuator::PointActuator( string aBodyName) :
	Actuator(),
	_body(NULL)
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	setPropertyValue("body", aBodyName);

	if (_model) {
		_body = &_model->updBodySet().get(aBodyName);
	} 
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
PointActuator::PointActuator(const PointActuator &anActuator) :
	Actuator(anActuator),
	_body(NULL)
{
	setNull();
	copyData(anActuator);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PointActuator::
setNull()
{
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointActuator::
setupProperties()
{
    // Allow this to be filled in later.
	addOptionalProperty<string>("body",
		"");

	const SimTK::Vec3 origin(0.0);
	addProperty<SimTK::Vec3>("point",
		"",
		origin);
	addProperty<bool>("point_is_global",
		"",
		false);
	addProperty<SimTK::Vec3>("direction",
		"",
		origin);
	addProperty<bool>("force_is_global",
		"",
		false);
	addProperty<double>("optimal_force",
		"",
		1.0);
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void PointActuator::
copyData(const PointActuator &aPointActuator)
{
	// MEMBER VARIABLES
	setPropertyValue("body", aPointActuator.getProperty<string>("body"));

	setPropertyValue("point", aPointActuator.getPropertyValue<SimTK::Vec3>("point"));
	setPropertyValue("point_is_global", aPointActuator.getPropertyValue<bool>("point_is_global"));
	setPropertyValue("direction", aPointActuator.getPropertyValue<SimTK::Vec3>("direction"));
	setPropertyValue("force_is_global", aPointActuator.getPropertyValue<bool>("force_is_global"));

	setOptimalForce(aPointActuator.getOptimalForce());
	setBody(aPointActuator.getBody());
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  aBodyID ID (or number, or index) of the generalized Body.
 */
PointActuator& PointActuator::
operator=(const PointActuator &aPointActuator)
{
	// BASE CLASS
	Actuator::operator =(aPointActuator);

	copyData(aPointActuator);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// BodyID
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized Body to which the Body actuator is applied.
 *
 * @param aBody Pointer to the generalized Body.
 */
void PointActuator::setBody(Body* aBody)
{
	_body = aBody;
	if(aBody)
		setPropertyValue("body", aBody->getName());
}
//_____________________________________________________________________________
/**
 * Get the generalized Body to which the Body actuator
 * is applied.
 *
 * @return Pointer to the Body
 */
Body* PointActuator::getBody() const
{
	return _body;
}

//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the force.
 *
 * @param aOptimalForce Optimal force.
 */
void PointActuator::setOptimalForce(double aOptimalForce)
{
	setPropertyValue("optimal_force", aOptimalForce);
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double PointActuator::getOptimalForce() const
{
	return getPropertyValue<double>("optimal_force");
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PointActuator::getStress( const SimTK::State& s) const
{
	return fabs(getForce(s)/getPropertyValue<double>("optimal_force")); 
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double  PointActuator::computeActuation( const SimTK::State& s ) const
{
	if(_model==NULL) return 0;

	// FORCE
	return( getControl(s) * getPropertyValue<double>("optimal_force")) ;
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

	if(_model==NULL || _body == NULL) return;

    double force;

    if( isForceOverriden(s) ) {
       force = computeOverrideForce(s);
    } else {
       force = computeActuation(s);
    }
    setForce(s,  force );

	
	SimTK::Vec3 forceVec = force*SimTK::UnitVec3(getPropertyValue<SimTK::Vec3>("direction"));
	SimTK::Vec3 lpoint = getPropertyValue<SimTK::Vec3>("point");
	if (!getPropertyValue<bool>("force_is_global"))
		engine.transform(s, *_body, forceVec, engine.getGroundBody(), forceVec);
	if (getPropertyValue<bool>("point_is_global"))
			engine.transformPosition(s, engine.getGroundBody(), lpoint, *_body, lpoint);
	applyForceToPoint(s, *_body, lpoint, forceVec, bodyForces);

	// get the velocity of the actuator in ground
	SimTK::Vec3 velocity(0);
	engine.getVelocity(s, *_body, lpoint, velocity);

	// the speed of the point is the "speed" of the actuator used to compute power
	setSpeed(s, velocity.norm());
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body reference _body
 */
void PointActuator::setup(Model& aModel)
{
	string errorMessage;

	const string &bodyName = getPropertyValue<string>("body");

	Actuator::setup(aModel);

	if (!aModel.updBodySet().contains(bodyName)) {
		errorMessage = "PointActuator: Unknown body (" + bodyName + ") specified in Actuator " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	if (_model) {
		_body = &_model->updBodySet().get(bodyName);
	}
}


//=============================================================================
// CHECK
//=============================================================================
//_____________________________________________________________________________
/**
 * Check that this point actuator actuator is valid.
 *
 * @return True if valid, false if invalid.
 */
bool PointActuator::check() const
{
	// BodyID
	if( _body != NULL) {
		printf("PointActuator.check: ERROR- %s actuates ",
			getName().c_str());
		printf("an invalid Body (%s).\n", getPropertyValue<string>("body").c_str());
		return(false);
	}
	return(true);
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
			setPropertyValue("force_is_global", true);
			converting = true;
		}
	}
	Actuator::updateFromXMLNode(aNode, versionNumber);
	if (converting) updPropertyValue<SimTK::Vec3>("direction") *= -1.0;
	setBody(_body);
}	

