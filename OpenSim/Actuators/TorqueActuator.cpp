// TorqueActuator.cpp
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
#include "TorqueActuator.h"
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
TorqueActuator::~TorqueActuator()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
TorqueActuator::TorqueActuator( string aBodyNameA, string aBodyNameB) :
	CustomActuator(),
	_bodyNameA(_propBodyNameA.getValueStr()),
	_bodyNameB(_propBodyNameB.getValueStr()),
	_torqueIsGlobal(_propTorqueIsGlobal.getValueBool()),
	_axis(_propAxis.getValueDblVec3()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_bodyA(NULL),
	_bodyB(NULL)
{
	// NULL
	setNull();

	// MEMBER VARIABLES
	_bodyNameA = aBodyNameA;
	_bodyNameB = aBodyNameB;

	if (_model) {
		_bodyA = &_model->updBodySet().get(_bodyNameA);
		_bodyB = &_model->updBodySet().get(_bodyNameB);
	} 
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
TorqueActuator::TorqueActuator(const TorqueActuator &anActuator) :
	CustomActuator(anActuator),
	_bodyNameA(_propBodyNameA.getValueStr()),
	_bodyNameB(_propBodyNameB.getValueStr()),
	_torqueIsGlobal(_propTorqueIsGlobal.getValueBool()),
	_axis(_propAxis.getValueDblVec3()),
	_optimalForce(_propOptimalForce.getValueDbl()),
	_bodyA(NULL),
	_bodyB(NULL)
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
Object* TorqueActuator::
copy() const
{
	TorqueActuator *force = new TorqueActuator(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void TorqueActuator::
setNull()
{
	setType("TorqueActuator");
	setupProperties();

	setNumStateVariables( 0);
}

void TorqueActuator::initStateCache(SimTK::State& s, SimTK::SubsystemIndex subsystemIndex, Model& model )
{
    Actuator::initStateCache(s, subsystemIndex, model);

}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void TorqueActuator::
setupProperties()
{
	SimTK::Vec3 z(0.0, 0.0, 1.0 );

	_propBodyNameA.setName("bodyA");
	_propertySet.append( &_propBodyNameA );

	_propBodyNameB.setName("bodyB");
	_propertySet.append( &_propBodyNameB );

	_propTorqueIsGlobal.setName("torque_is_global");
	_propTorqueIsGlobal.setValue(true);
	_propertySet.append( &_propTorqueIsGlobal );

	_propAxis.setName("axis");
	_propAxis.setValue(z);
	_propertySet.append( &_propAxis );

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void TorqueActuator::
copyData(const TorqueActuator &aTorqueActuator)
{
	// MEMBER VARIABLES
	_bodyNameA = aTorqueActuator._bodyNameA;
	_bodyNameB = aTorqueActuator._bodyNameB;
	_torqueIsGlobal = aTorqueActuator._torqueIsGlobal;
	_axis = aTorqueActuator._axis;

	setOptimalForce(aTorqueActuator.getOptimalForce());
	setBodyA(aTorqueActuator.getBodyA());
	setBodyB(aTorqueActuator.getBodyB());
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
TorqueActuator& TorqueActuator::
operator=(const TorqueActuator &aTorqueActuator)
{
	// BASE CLASS
	Actuator::operator =(aTorqueActuator);

	copyData(aTorqueActuator);

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
void TorqueActuator::setBodyA(Body* aBody)
{
	_bodyA = aBody;
	if(aBody)
		_bodyNameA = aBody->getName();
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
		_bodyNameB = aBody->getName();
}
//_____________________________________________________________________________
/**
 * Get the generalized Body to which the Body actuator
 * is applied.
 *
 * @return Pointer to the Body
 */
Body* TorqueActuator::getBodyA() const
{
	return(_bodyA);
}
//_____________________________________________________________________________
/**
 * Get the generalized Body to which the equal and opposite Body actuation
 * is applied.
 *
 * @return Pointer to the Body
 */
Body* TorqueActuator::getBodyB() const
{
	return(_bodyB);
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
void TorqueActuator::setOptimalForce(double aOptimalForce)
{
	_optimalForce = aOptimalForce;
}
//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double TorqueActuator::getOptimalForce() const
{
	return(_optimalForce);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double TorqueActuator::getStress( const SimTK::State& s) const
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
double TorqueActuator::computeActuation( const SimTK::State& s ) const
{
	if(_model==NULL) return 0;

	// FORCE
	return ( getControl(s) * _optimalForce );
}



//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force to BodyA and BodyB.
 */
void TorqueActuator::computeForce(const SimTK::State& s, 
							      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							      SimTK::Vector& generalizedForces) const
{
	if(_model==NULL) return;
	const SimbodyEngine& engine = getModel().getSimbodyEngine();

	
    double force;

    if( isControlled() && getController().getIsEnabled() ) {
       force = computeActuation(s);
       setForce(s,  force);
    } else {
       force = getForce(s);
    }

//std::cout << "TorqueActuator::computeForce t=" << s.getTime() << "  " << getName() <<   " force= " << force <<  std::endl;

	if(_bodyA ==NULL)
		return;
	// if bodyB is not specified, use the ground body by default
	if(_bodyB == NULL)
		*_bodyB = getModel().getGroundBody();

    setForce(s, force );
	SimTK::Vec3 torque = force*SimTK::UnitVec3(_axis);
	
	if (!_torqueIsGlobal)
		engine.transform(s, *_bodyA, torque, engine.getGroundBody(), torque);
	
	applyTorque(s, *_bodyA, torque, bodyForces);

	if(_bodyB !=NULL)
		applyTorque(s, *_bodyB, -torque, bodyForces);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Body references _bodyA and _bodyB
 */
void TorqueActuator::setup(Model& aModel)
{
	CustomActuator::setup( aModel);

	if (_model) {
		_bodyA = &_model->updBodySet().get(_bodyNameA);
		_bodyB = &_model->updBodySet().get(_bodyNameB);
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
bool TorqueActuator::check() const
{
	if(!Actuator::check()) return(false);

	// BodyID
	if( _bodyA != NULL) {
		printf("TorqueActuator.check: ERROR- %s actuates ",
			getName().c_str());
		printf("an invalid Body (%s).\n", _bodyNameA.c_str());
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
void TorqueActuator::
updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	bool converting=false;
	if ( documentVersion < XMLDocument::getLatestVersion()){
		// Now check if we need to create a correction controller to replace springs
		if (_node!=NULL && documentVersion<10905){
			// This used to be called "Force" back then
			renameChildNode("body_A", "bodyB"); // body_B -> body
			renameChildNode("body_B", "bodyA"); // direction_A -> direction
			renameChildNode("direction_A", "axis"); // direction_A -> direction
			converting = true;
		}
	}
	Actuator::updateFromXMLNode();
	if (converting) _axis *= -1.0;
	setBodyA(_bodyA);
	setBodyB(_bodyB);
	setOptimalForce(_optimalForce);
}	

/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> TorqueActuator::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> TorqueActuator::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);
	values.append(getForce(state));
	return values;
};

