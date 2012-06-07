// CMC_Joint.cpp
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
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "CMC_Task.h"
#include "CMC_Joint.h"

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
CMC_Joint::~CMC_Joint()
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
CMC_Joint::CMC_Joint(const string &aCoordinateName) :
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
CMC_Joint::CMC_Joint(const CMC_Joint &aTask) :
	CMC_Task(aTask),
	_coordinateName(_propCoordinateName.getValueStr()),
	_limit(_propLimit.getValueDbl())
{
	setNull();
	*this = aTask;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void CMC_Joint::
setNull()
{
	setupProperties();

	_nTrk = 1;
	_q = 0;
}
//_____________________________________________________________________________
/**
 * Set up serialized member variables.
 */
void CMC_Joint::
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
void CMC_Joint::
copyData(const CMC_Joint &aTask)
{
	setCoordinateName(aTask.getCoordinateName());
	_limit = aTask._limit;
}


//_____________________________________________________________________________
/**
 * Update work variables
 */
void CMC_Joint::
updateWorkVariables()
{
	_q = 0;
	if(_model) {
        try {
            _q = &_model->updCoordinateSet().get(_coordinateName);
        }
        catch (const Exception&) {
            throw Exception("CMC_Joint.updateWorkVariables: ERROR- joint task '" + getName() 
									+ "' references invalid coordinate '" + _coordinateName + "'",__FILE__,__LINE__);
        }
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
CMC_Joint& CMC_Joint::
operator=(const CMC_Joint &aTask)
{
	// BASE CLASS
	CMC_Task::operator =(aTask);

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
void CMC_Joint::
setModel(Model& aModel)
{
	CMC_Task::setModel(aModel);
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
void CMC_Joint::
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
string CMC_Joint::
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
double CMC_Joint::
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
void CMC_Joint::
computeErrors(const SimTK::State& s, double aT)
{
	// COMPUTE ERRORS
	//std::cout<<_coordinateName<<std::endl;
	//std::cout<<"_pTrk[0]->evaluate(0,aT) = "<<_pTrk[0]->evaluate(0,aT)<<std::endl;
	//std::cout<<"_q->getValue() = "<<_q->getValue()<<std::endl;
	_pErr[0] = _pTrk[0]->calcValue(SimTK::Vector(1,aT)) - _q->getValue(s);
	if(_vTrk[0]==NULL) {
		std::vector<int> derivComponents(1);
		derivComponents[0]=0;
		_vErr[0] = _pTrk[0]->calcDerivative(derivComponents,SimTK::Vector(1,aT)) - _q->getSpeedValue(s);
	} else {
		_vErr[0] = _vTrk[0]->calcValue(SimTK::Vector(1,aT)) - _q->getSpeedValue(s);
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
void CMC_Joint::
computeDesiredAccelerations(const SimTK::State& s, double aT)
{
	_aDes=SimTK::NaN;

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(s, aT);

	// DESIRED ACCELERATION
	double p = (_kp)[0]*_pErr[0];
	double v = (_kv)[0]*_vErr[0];
	double a;
	if(_aTrk[0]==NULL) {
		std::vector<int> derivComponents(2);
		derivComponents[0]=0;
		derivComponents[1]=0;
		a = (_ka)[0]*_pTrk[0]->calcDerivative(derivComponents,SimTK::Vector(1,aT));
	} else {
		a = (_ka)[0]*_aTrk[0]->calcValue(SimTK::Vector(1,aT));
	}
	_aDes[0] = a + v + p;

	// PRINT
	//printf("CMC_Joint.computeDesiredAcceleration:\n");
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
void CMC_Joint::
computeDesiredAccelerations(const SimTK::State& s, double aTI,double aTF)
{
	double a;
	_aDes=SimTK::NaN;

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(s, aTI);

	// DESIRED ACCELERATION
	double p = (_kp)[0]*_pErr[0];
	double v = (_kv)[0]*_vErr[0];
	
	if(_aTrk[0]==NULL) {
		std::vector<int> derivComponents(2);
		derivComponents[0]=0;
		derivComponents[1]=0;
		a = (_ka)[0]*_pTrk[0]->calcDerivative(derivComponents,SimTK::Vector(1,aTF));
	} else {
		a = (_ka)[0]*_aTrk[0]->calcValue(SimTK::Vector(1,aTF));
	}
	_aDes[0] = a + v + p;

	// PRINT
	//printf("CMC_Joint.computeDesiredAcceleration:\n");
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
void CMC_Joint::
computeAccelerations(const SimTK::State& s )
{
	_a=SimTK::NaN;

	// CHECK
	if(_model==NULL) return;

	// ACCELERATION
	_a[0] = _q->getAccelerationValue(s);
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
void CMC_Joint::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	CMC_Task::updateFromXMLNode(aNode, versionNumber);
	setCoordinateName(_coordinateName);
}
