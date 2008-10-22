// rdCMC_Joint.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include "rdCMC_Task.h"
#include "rdCMC_Joint.h"

using namespace std;
using namespace OpenSim;

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
rdCMC_Joint::~rdCMC_Joint()
{
}

//_____________________________________________________________________________
/**
 * Construct a task for a specified generalized coordinate.
 *
 * @param aQID ID of the generalized coordinate to be tracked.
 * @todo Insted of an integer id, the name of the coordinate
 * should be used.
 */
rdCMC_Joint::rdCMC_Joint(const string &aCoordinateName) :
	_coordinateName(_propCoordinateName.getValueStr()),
   	_limit(_propLimit.getValueDbl())
{
	setNull();
	setCoordinateName(aCoordinateName);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Joint task to be copied.
 */
rdCMC_Joint::rdCMC_Joint(const rdCMC_Joint &aTask) :
	rdCMC_Task(aTask),
	_coordinateName(_propCoordinateName.getValueStr()),
	_limit(_propLimit.getValueDbl())
{
	setNull();
	copyData(aTask);
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void rdCMC_Joint::
setNull()
{
	setType("rdCMC_Joint");
	setupProperties();

	_nTrk = 1;
	_q = 0;
	_u = 0;
}
//_____________________________________________________________________________
/**
 * Set up serialized member variables.
 */
void rdCMC_Joint::
setupProperties()
{
	_propCoordinateName.setComment("Name of the coordinate to be tracked.");
	_propCoordinateName.setName("coordinate");
	_propCoordinateName.setValue("");
	_propertySet.append(&_propCoordinateName);

	_propLimit.setComment("Error limit on the tracking accuracy for this "
		"coordinate. If the tracking errors approach this limit, the weighting "
		"for this coordinate is increased. ");
	_propLimit.setName("limit");
	_propLimit.setValue(0);
	_propertySet.append(&_propLimit);
}


//_____________________________________________________________________________
/**
 * Copy only the member data of specified object.
 */
void rdCMC_Joint::
copyData(const rdCMC_Joint &aTask)
{
	setCoordinateName(aTask.getCoordinateName());
	_limit = aTask._limit;
}

//_____________________________________________________________________________
/**
 * Copy this track object and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this track object.
 */
Object* rdCMC_Joint::
copy() const
{
	rdCMC_Joint *object = new rdCMC_Joint(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Update work variables
 */
void rdCMC_Joint::
updateWorkVariables()
{
	_u = 0;
	_q = 0;
	if(_model) {
		std::string speedName = AbstractSpeed::getSpeedName(_coordinateName);
		_u = _model->getDynamicsEngine().getSpeedSet()->get(speedName);
		if(_u) {
			_q = _u->getCoordinate();
			if(!_q) throw Exception("rdCMC_Joint.updateWorkVariables: ERROR- joint task '" + getName() 
											+ "' references invalid coordinate '" + _coordinateName + "'",__FILE__,__LINE__);
		} else throw Exception("rdCMC_Joint.updateWorkVariables: ERROR- joint task '" + getName() 
									  + "' references invalid speed '" + speedName + "'",__FILE__,__LINE__);
	}
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
 * @param aTask Object to be copied.
 * @return  Reference to the altered object.
 */
rdCMC_Joint& rdCMC_Joint::
operator=(const rdCMC_Joint &aTask)
{
	// BASE CLASS
	rdCMC_Task::operator =(aTask);

	// DATA
	copyData(aTask);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Initializes pointers to the Coordinate and Speed in the model given the 
 * coordinate name assigned to this task.
 *
 * @param aModel Model.
 */
void rdCMC_Joint::
setModel(Model *aModel)
{
	rdCMC_Task::setModel(aModel);
	updateWorkVariables();
}
//-----------------------------------------------------------------------------
// Coordinate Name
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the generalized coordinate that is to be tracked.
 *
 * @param aName Name of the tracked generalized coordinate.
 */
void rdCMC_Joint::
setCoordinateName(const string &aName)
{
	_coordinateName = aName;
	updateWorkVariables();
}
//_____________________________________________________________________________
/**
 * Get the generalized coordinate that is to be tracked.
 *
 * @return Name of the tracked generalized coordinate.
 */
string rdCMC_Joint::
getCoordinateName() const
{
	return(_coordinateName);
}
//_____________________________________________________________________________
/**
 * Get the limit.
 *
 * @return limit valid of the tracked generalized coordinate.
 */
double rdCMC_Joint::
getLimit() const
{
	return _limit;
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the position and velocity errors.
 * This method assumes the states have been set for the model.
 * 
 * @param aT Current time in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void rdCMC_Joint::
computeErrors(double aT)
{
	// COMPUTE ERRORS
	//std::cout<<_coordinateName<<std::endl;
	//std::cout<<"_pTrk[0]->evaluate(0,aT) = "<<_pTrk[0]->evaluate(0,aT)<<std::endl;
	//std::cout<<"_q->getValue() = "<<_q->getValue()<<std::endl;
	_pErr[0] = _pTrk[0]->evaluate(0,aT) - _q->getValue();
	if(_vTrk[0]==NULL) {
		_vErr[0] = _pTrk[0]->evaluate(1,aT) - _u->getValue();
	} else {
		_vErr[0] = _vTrk[0]->evaluate(0,aT) - _u->getValue();
	}
}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aT Time at which the desired accelerations are to be computed in
 * real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void rdCMC_Joint::
computeDesiredAccelerations(double aT)
{
	_aDes=rdMath::getNAN();

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(aT);

	// DESIRED ACCELERATION
	double p = (_kp)[0]*_pErr[0];
	double v = (_kv)[0]*_vErr[0];
	double a;
	if(_aTrk[0]==NULL) {
		a = (_ka)[0]*_pTrk[0]->evaluate(2,aT);
	} else {
		a = (_ka)[0]*_aTrk[0]->evaluate(0,aT);
	}
	_aDes[0] = a + v + p;

	// PRINT
	//printf("rdCMC_Joint.computeDesiredAcceleration:\n");
	//printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
	//	_pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 * This method assumes that the states have been set for the model.
 *
 * @param aTI Initial time of the controlled interval in real time units.
 * @param aTF Final time of the controlled interval in real time units.
 * @see Model::set()
 * @see Model::setStates()
 */
void rdCMC_Joint::
computeDesiredAccelerations(double aTI,double aTF)
{
	_aDes=rdMath::getNAN();

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(aTI);

	// DESIRED ACCELERATION
	double p = (_kp)[0]*_pErr[0];
	double v = (_kv)[0]*_vErr[0];
	double a;
	if(_aTrk[0]==NULL) {
		a = (_ka)[0]*_pTrk[0]->evaluate(2,aTF);
	} else {
		a = (_ka)[0]*_aTrk[0]->evaluate(0,aTF);
	}
	_aDes[0] = a + v + p;

	// PRINT
	//printf("rdCMC_Joint.computeDesiredAcceleration:\n");
	//printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
	//	_pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}
//_____________________________________________________________________________
/**
 * Compute the acceleration of the appropriate generalized coordinate.
 * For the computed accelerations to be correct,
 * Model::computeAccelerations() must have already been called.
 *
 * For joints (i.e., generalized coordinates), the acceleration is
 * not computed.  It has already been computed and is simply retrieved
 * from the model.
 *
 * @see Model::computeAccelerations()
 * @see suTrackObject::getAcceleration()
 */
void rdCMC_Joint::
computeAccelerations()
{
	_a=rdMath::getNAN();

	// CHECK
	if(_model==NULL) return;

	// ACCELERATION
	_a[0] = _u->getAcceleration();
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
 * @param aDeep If true, update this object and all its child objects
 * (that is, member variables that are Object's); if false, update only
 * the member variables that are not Object's.
 */
void rdCMC_Joint::
updateFromXMLNode()
{
	rdCMC_Task::updateFromXMLNode();
	setCoordinateName(_coordinateName);
}
