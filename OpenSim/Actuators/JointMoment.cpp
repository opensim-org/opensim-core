// JointMoment.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include "JointMoment.h"
#include <OpenSim/Simulation/Model/Muscle.h>




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
JointMoment::~JointMoment()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
JointMoment::
JointMoment(int aQID,int aNX,int aNY,int aNYP) :
	GeneralizedForceAtv(aQID,aNX,aNY,aNYP),
	_optimalNegForce(_propOptimalNegForce.getValueDbl())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct the actuator from an XML Element.
 *
 * @param aElement XML element.
 * @param aNX Number of controls.
 * @param aNY Number of states.
 * @param aNYP Number of pseudostates.
 */
JointMoment::
JointMoment(DOMElement *aElement,int aNX,int aNY,int aNYP) :
	GeneralizedForceAtv(aElement,aNX,aNY,aNYP),
	_optimalNegForce(_propOptimalNegForce.getValueDbl())
{
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to be copied.
 */
JointMoment::
JointMoment(const JointMoment &aActuator) :
	GeneralizedForceAtv(aActuator),
	_optimalNegForce(_propOptimalNegForce.getValueDbl())
{
	setNull();
	copyData(aActuator);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* JointMoment::
copy() const
{
	Actuator *act = new JointMoment(*this);
	return(act);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using the contructor for the DOMElement
 * in order to establish the relationship of the Force object with the
 * XML node.  Then, the assignment operator is used to set all data members
 * of the copy to the values of this object.  Finally, the data members of
 * the copy are updated from the DOMElment using updateObject().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this actuator.
 */
Object* JointMoment::
copy(DOMElement *aElement) const
{
	// ESTABLISH RELATIONSHIP WITH XML NODE
	JointMoment *act = new
		JointMoment(aElement,getNX(),getNY(),getNYP());

	// ASSIGNMENT OPERATOR
	*act = *this;

	// UPDATE BASED ON NODE
	act->updateFromXMLNode();

	return(act);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void JointMoment::
setNull()
{
	setType("JointMoment");
	setupProperties();

	// APPLIES FORCE
	_appliesForce = true;

	// MEMBER VARIABLES
	_a = 0.0;
}
//_____________________________________________________________________________
/**
 * Set up the serializable member variables.  This involves generating
 * properties and connecting local variables to those properties.
 */
void JointMoment::
setupProperties()
{
	_propOptimalNegForce.setName("optimal_negative_force");
	_propOptimalNegForce.setValue(1.0);
	_propertySet.append( &_propOptimalNegForce );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void JointMoment::
copyData(const JointMoment &aActuator)
{
	setOptimalNegativeForce(aActuator.getOptimalNegativeForce());
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
 * @return  Reference to the altered object.
 */
JointMoment& JointMoment::
operator=(const JointMoment &aActuator)
{
	// BASE CLASS
	GeneralizedForceAtv::operator =(aActuator);

	// DATA
	copyData(aActuator);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// OPTIMAL NEGATIVE FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal negative force.  This property is used when the activation
 * level of the actuator is negative, allowing for a different flexion and
 * extension stregths of the actuator.
 *
 * Note that this property should always be positive.  If a negative value
 * is sent in through the argument list of this method, it's absolute value
 * is taken.
 *
 * @param aOptimalNegForce Optinal negative force of the actuator.
 */
void JointMoment::
setOptimalNegativeForce(double aOptimalNegForce)
{
	_optimalNegForce = fabs(aOptimalNegForce);

}
//_____________________________________________________________________________
/**
 * Get the optimal negative force.  This property is used when the activation
 * level of the actuator is negative, allowing for a different flexion and
 * extension stregths of the actuator.
 *
 * This property should always be positive.
 *
 * @return Optinal negative force of the actuator.
 */
double JointMoment::
getOptimalNegativeForce() const
{
	return(_optimalNegForce);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
void JointMoment::
computeActuation()
{
	if(_model==NULL) return;

	// SPEED
	_speed = _model->getSpeed(_qID);

	// FORCE
	double force;
	if(_a<0.0) {
		force = _a * _optimalNegForce;
	} else {
		force = _a * _optimalForce;
	}

	setForce(force);
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
void JointMoment::
updateFromXMLNode()
{
	GeneralizedForceAtv::updateFromXMLNode();
	setOptimalNegativeForce(_optimalNegForce);
}	

