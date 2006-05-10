// GeneralizedForceAtv.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include "GeneralizedForceAtv.h"
#include <OpenSim/Simulation/Model/Muscle.h>




using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================
const string GeneralizedForceAtv::X_NAME = "excitation";
const string GeneralizedForceAtv::Y_NAME = "activation";


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
GeneralizedForceAtv::~GeneralizedForceAtv()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
GeneralizedForceAtv::
GeneralizedForceAtv(int aQID,int aNX,int aNY,int aNYP) :
	GeneralizedForce(aQID,aNX,aNY,aNYP),
	_riseTime(_propRiseTime.getValueDbl()),
	_fallTime(_propFallTime.getValueDbl())
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
GeneralizedForceAtv::
GeneralizedForceAtv(DOMElement *aElement,int aNX,int aNY,int aNYP) :
	GeneralizedForce(aElement,aNX,aNY,aNYP),
	_riseTime(_propRiseTime.getValueDbl()),
	_fallTime(_propFallTime.getValueDbl())
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
GeneralizedForceAtv::
GeneralizedForceAtv(const GeneralizedForceAtv &aActuator) :
	GeneralizedForce(aActuator),
	_riseTime(_propRiseTime.getValueDbl()),
	_fallTime(_propFallTime.getValueDbl())
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
Object* GeneralizedForceAtv::
copy() const
{
	Actuator *act = new GeneralizedForceAtv(*this);
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
Object* GeneralizedForceAtv::
copy(DOMElement *aElement) const
{
	// ESTABLISH RELATIONSHIP WITH XML NODE
	GeneralizedForceAtv *act = new
		GeneralizedForceAtv(aElement,getNX(),getNY(),getNYP());

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
void GeneralizedForceAtv::
setNull()
{
	setType("GeneralizedForceAtv");
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
void GeneralizedForceAtv::
setupProperties()
{
	_propRiseTime.setName("rise_time");
	_propRiseTime.setValue(0.010);
	_propertySet.append( &_propRiseTime );

	_propFallTime.setName("fall_time");
	_propFallTime.setValue(0.050);
	_propertySet.append( &_propFallTime );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void GeneralizedForceAtv::
copyData(const GeneralizedForceAtv &aActuator)
{
	// STATES
	aActuator.getStates(&_a);

	// PROPERTIES
	setRiseTime(aActuator.getRiseTime());
	setFallTime(aActuator.getFallTime());
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
GeneralizedForceAtv& GeneralizedForceAtv::
operator=(const GeneralizedForceAtv &aActuator)
{
	// BASE CLASS
	GeneralizedForce::operator =(aActuator);

	// DATA
	copyData(aActuator);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// STATES
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the number of states.
 *
 * @return Number of states (1 = activation).
 */
int GeneralizedForceAtv::
getNY() const
{
	return(1);
}
//_____________________________________________________________________________
/**
 * Get the name of a state.\n
 * Valid indices: 0
 *
 * @param aIndex Index of the state whose name is desired.
 * @throws Exception If aIndex is not valid.
 */
const string& GeneralizedForceAtv::
getStateName(int aIndex) const
{
	switch(aIndex) {
	case(0):
		return(Y_NAME);
	default:
		string msg = "GeneralizedForceAtv.setState: ERR- index out of bounds.\n";
		msg += "Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has 1 states (index 0 is valid).";
		throw( Exception(msg,__FILE__,__LINE__) );
	}
}
//_____________________________________________________________________________
/**
 * Get the index of a state of a specified name.\n
 * Valid names: activation
 *
 * @param aName Name of the state whose index is desired.
 * @return Index of the specified state.
 * @throws Exception If aName is not valid.
 */
int GeneralizedForceAtv::
getStateIndex(const string &aName) const
{
	if(aName==Y_NAME) {
		return(0);
	} else {
		string msg = "GeneralizedForceAtv.getStateIndex: ERR- Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has no state by the name";
		msg += aName;
		msg += ".\n Valid names are ";
		msg += Y_NAME + "."; 
		throw( Exception(msg,__FILE__,__LINE__) );
	}
	return(0);
}
//_____________________________________________________________________________
/**
 * Set the current values of the states.  The states are the
 * coordinates of contact PointA.
 *
 * @param aY Array of states.  aY should have a size of 1.
 */
void GeneralizedForceAtv::
setStates(const double aY[])
{
	_a = aY[0];
}
//_____________________________________________________________________________
/**
 * Set the value of a state at a specified index.\n
 * Valid indices:  0
 *
 * @param aIndex Index of the state to be set.
 * @param aValue Value to which to set the state.
 * @throws Exception If aIndex is not valid.
 */
void GeneralizedForceAtv::
setState(int aIndex,double aValue)
{
	if(aIndex!=0) {
		string msg = "GeneralizedForceAtv.setState: ERR- index out of bounds.\n";
		msg += "Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has 3 states (indices 0-2 are valid).";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	_a = aValue;
}
//_____________________________________________________________________________
/**
 * Set the value of a state of a specified name.\n
 * Valid names: actvation
 *
 * @param aName Name of the state to be set.
 * @param aValue Value to which to set the state.
 * @throws Exception If aName is not valid.
 */
void GeneralizedForceAtv::
setState(const string &aName,double aValue)
{
	int index = getStateIndex(aName);
	setState(index,aValue);
}
//_____________________________________________________________________________
/**
 * Get the current values of the states.
 *
 * @param rY Array of states.  The size of rYP should be 3.
 */
void GeneralizedForceAtv::
getStates(double rY[]) const
{
	rY[0] = _a;
}
//_____________________________________________________________________________
/**
 * Get the value of a state at a specified index.\n
 *	Valid Indices: 0
 *
 * @param aIndex Index of the desired state.
 * @return Value of the desired state.
 * @throws Exception If aIndex is not valid.
 */
double GeneralizedForceAtv::
getState(int aIndex) const
{
	if(aIndex!=0) {
		string msg = "GeneralizedForceAtv.setState: ERR- index out of bounds.\n";
		msg += "Actuator ";
		msg += getName();
		msg += " of type ";
		msg += getType();
		msg += " has 1 states (index 0 is valid).";
		throw( Exception(msg,__FILE__,__LINE__) );
	}

	return(_a);
}
//_____________________________________________________________________________
/**
 * Get the value of a state of a specified name.\n
 * Valid names: activation
 *
 * @param aName Name of the desired state.
 * @return Value of the desired state.
 * @throws Exception If aName is not valid.
 */
double GeneralizedForceAtv::
getState(const string &aName) const
{
	int index = getStateIndex(aName);
	return(getState(index));
}

//-----------------------------------------------------------------------------
// RISE TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the rise time of the actuator.
 *
 * @param aRiseTime Rise time of activation dynamics.
 */
void GeneralizedForceAtv::
setRiseTime(double aRiseTime)
{
	_riseTime = aRiseTime;
}
//_____________________________________________________________________________
/**
 * Get the rise time of the actuator.
 *
 * @return Rise time of activation dynamics.
 */
double GeneralizedForceAtv::
getRiseTime() const
{
	return(_riseTime);
}

//-----------------------------------------------------------------------------
// FALL TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the fall time of the actuator.
 *
 * @param aFallTime Fall time of activation dynamics.
 */
void GeneralizedForceAtv::
setFallTime(double aFallTime)
{
	_fallTime = aFallTime;
}
//_____________________________________________________________________________
/**
 * Get the fall time of the actuator.
 *
 * @return Fall time of activation dynamics.
 */
double GeneralizedForceAtv::
getFallTime() const
{
	return(_fallTime);
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the activation level of this actuator equal to the neural excitation.
 */
void GeneralizedForceAtv::
promoteControlsToStates(const double aX[],double aDT)
{
	if(aX==NULL) return;
	if(aDT<=0) {
		_a = aX[0];
	} else {
		_a = Muscle::EstimateActivation(_riseTime,_fallTime,_a,aX[0],aDT);
	}
}

//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
void GeneralizedForceAtv::
computeActuation()
{
	if(_model==NULL) return;

	// SPEED
	_speed = _model->getSpeed(_qID);

	// FORCE
	double force = _a * _optimalForce;
	setForce(force);
}

//_____________________________________________________________________________
/**
 * Compute the time derivatives of the states for this actuator.
 *
 * @param rDYDT Time derivatives of the states-- should have a length of at
 * least the value returned by getNY().
 * @see getNY()
 */
void GeneralizedForceAtv::
computeStateDerivatives(double rDYDT[])
{
	if(rDYDT==NULL) return;
	rDYDT[0] = Muscle::DADT(_riseTime,_fallTime,getControl(0),_a);
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
void GeneralizedForceAtv::
updateFromXMLNode()
{
	GeneralizedForce::updateFromXMLNode();
	setRiseTime(_riseTime);
	setFallTime(_fallTime);
}	

