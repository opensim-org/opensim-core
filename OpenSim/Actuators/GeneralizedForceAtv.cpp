// GeneralizedForceAtv.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/PropertyDbl.h>
#include "GeneralizedForceAtv.h"
#include "Muscle.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>



using namespace OpenSim;
using namespace std;


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
GeneralizedForceAtv(string aQName) :
	GeneralizedForce(aQName),
	_riseTime(_propRiseTime.getValueDbl()),
	_fallTime(_propFallTime.getValueDbl())
{
	setNull();
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
	AbstractActuator *act = new GeneralizedForceAtv(*this);
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

	setNumControls(1); setNumStates(1); setNumPseudoStates(0);
	bindControl(0, _excitation, "excitation");
	bindState(0, _a, "activation");

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
	if (_q)
	{
		AbstractSpeed *speed =  _model->getDynamicsEngine().getSpeedSet()->get(_q->getName());
		if (speed)
			_speed = speed->getValue();
	}

	// FORCE
	double force = _a * _optimalForce;
	setForce(force);
}

//_____________________________________________________________________________
/**
 * Compute the time derivatives of the states for this actuator.
 *
 * @param rDYDT Time derivatives of the states-- should have a length of at
 * least the value returned by getNumStates().
 * @see getNumStates()
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

