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
#include <OpenSim/Common/XMLNode.h>
#include "PointActuator.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>

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
	_bodyName(_propBodyName.getValueStr()),
	_point(_propPoint.getValueDblVec()),
	_pointIsGlobal(_propPointIsGlobal.getValueBool()),
	_direction(_propDirection.getValueDblVec()),
	_forceIsGlobal(_propForceIsGlobal.getValueBool()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_body(NULL)
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	_bodyName = aBodyName;

	if (_model) {
		_body = &_model->updBodySet().get(_bodyName);
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
	_bodyName(_propBodyName.getValueStr()),
	_point(_propPoint.getValueDblVec()),
	_pointIsGlobal(_propPointIsGlobal.getValueBool()),
	_direction(_propDirection.getValueDblVec()),
	_forceIsGlobal(_propForceIsGlobal.getValueBool()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_body(NULL)
{
	setNull();
	copyData(anActuator);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* PointActuator::
copy() const
{
	PointActuator *force = new PointActuator(*this);
	return force;
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
	setType("PointActuator");
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PointActuator::
setupProperties()
{
	SimTK::Vec3 origin(0.0);

	_propBodyName.setName("body");
	_propertySet.append( &_propBodyName );

	_propPoint.setName("point");
	_propPoint.setValue(origin);
	_propertySet.append( &_propPoint );

	_propPointIsGlobal.setName("point_is_global");
	_propPointIsGlobal.setValue(false);
	_propertySet.append( &_propPointIsGlobal );

	_propDirection.setName("direction");
	_propDirection.setValue(origin);
	_propertySet.append( &_propDirection );

	_propForceIsGlobal.setName("force_is_global");
	_propForceIsGlobal.setValue(false);
	_propertySet.append( &_propForceIsGlobal );

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void PointActuator::
copyData(const PointActuator &aPointActuator)
{
	// MEMBER VARIABLES
	_bodyName = aPointActuator._bodyName;
	_point = aPointActuator._point; 
	_pointIsGlobal = aPointActuator._pointIsGlobal;
	_direction = aPointActuator._direction;
	_forceIsGlobal = aPointActuator._forceIsGlobal;

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
		_bodyName = aBody->getName();
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
	return(_body);
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
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double PointActuator::getOptimalForce() const
{
	return(_optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PointActuator::getStress( const SimTK::State& s) const
{
	return fabs(getForce(s)/_optimalForce); 
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
	return( getControl(s) * _optimalForce) ;
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


//std::cout << "PointActuator::computeForce t=" << s.getTime() << "  " << getName() <<   " force= " << force << std::endl;

	
	SimTK::Vec3 forceVec = force*SimTK::UnitVec3(_direction);
	SimTK::Vec3 lpoint = _point;
	if (!_forceIsGlobal)
		engine.transform(s, *_body, forceVec, engine.getGroundBody(), forceVec);
	if (_pointIsGlobal)
			engine.transformPosition(s, engine.getGroundBody(), lpoint, *_body, lpoint);
	applyForceToPoint(s, *_body, lpoint, forceVec, bodyForces);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body reference _body
 */
void PointActuator::setup(Model& aModel)
{
	string errorMessage;
	Actuator::setup(aModel);

	if (!aModel.updBodySet().contains(_bodyName)) {
		errorMessage = "PointActuator: Unknown body (" + _bodyName + ") specified in Actuator " + getName();
		throw (Exception(errorMessage.c_str()));
	}

	if (_model) {
		_body = &_model->updBodySet().get(_bodyName);
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
		printf("an invalid Body (%s).\n", _bodyName.c_str());
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
 * This method simply calls Object::updateFromXMLNode() and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void PointActuator::
updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	bool converting=false;
	if ( documentVersion < XMLDocument::getLatestVersion()){
			// Now check if we need to create a correction controller to replace springs
		if (_node!=NULL && documentVersion<10905){
			// This used to be called "Force" back then
			renameChildNode("body_B", "body"); // body_B -> body
			renameChildNode("point_B", "point"); // point_B -> point
			renameChildNode("direction_A", "direction"); // direction_A -> direction
			_forceIsGlobal = true;
			converting = true;
		}
	}
	Actuator::updateFromXMLNode();
	if (converting) _direction *= -1.0;
	setBody(_body);
	setOptimalForce(_optimalForce);
}	

/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> PointActuator::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> PointActuator::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);
	values.append(getForce(state));
	return values;
};
