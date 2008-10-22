// rdCMC_Point.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*Contributors: Frank C. Anderson, Jeffrey A. Reinbolt
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
#include <OpenSim/Simulation/Model/BodySet.h>
#include "rdCMC_Task.h"
#include "rdCMC_Point.h"

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
rdCMC_Point::~rdCMC_Point()
{
}
//_____________________________________________________________________________
/**
 * Construct a task for a specified point.
 *
 */
rdCMC_Point::rdCMC_Point(const SimTK::Vec3 &aPoint) :
	_point(_propPoint.getValueDblVec3())
{
	setNull();
	setPoint(aPoint);
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aTask Point task to be copied.
 */
rdCMC_Point::rdCMC_Point(const rdCMC_Point &aTask) :
	rdCMC_Task(aTask),
	_point(_propPoint.getValueDblVec3())
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
void rdCMC_Point::
setNull()
{
	setType("rdCMC_Point");
	setupProperties();

	_nTrk = 3;
	_p = 0;
	_v = 0;
}
//_____________________________________________________________________________
/**
 * Set up serialized member variables.
 */
void rdCMC_Point::
setupProperties()
{
	_propPoint.setComment("Point in body frame with respect to which an objective is tracked.");
	_propPoint.setName("point");
	_propPoint.setValue(SimTK::Vec3(0));
	_propertySet.append(&_propPoint);
}

//_____________________________________________________________________________
/**
 * Copy only the member data of specified object.
 */
void rdCMC_Point::
copyData(const rdCMC_Point &aTask)
{
	setPoint(aTask.getPoint());
}

//_____________________________________________________________________________
/**
 * Copy this track object and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this track object.
 */
Object* rdCMC_Point::
copy() const
{
	rdCMC_Point *object = new rdCMC_Point(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Update work variables
 */
void rdCMC_Point::
updateWorkVariables()
{
	_p = 0;
	_v = 0;
	if(_model) {

		BodySet *bs = _model->getDynamicsEngine().getBodySet();

		if(_wrtBodyName == "center_of_mass") {

			SimTK::Vec3 pVec,vVec,com;
			double Mass = 0.0;
			double rP[3] = { 0.0, 0.0, 0.0 };
			for(int i=0;i<bs->getSize();i++) {
				AbstractBody *body = bs->get(i);
				body->getMassCenter(com);
				_model->getDynamicsEngine().getPosition(*body,com,pVec);
				if(pVec[0] != pVec[0]) throw Exception("rdCMC_Point.computeAccelerations: ERROR- point task '" + getName() 
											+ "' references invalid acceleration components",__FILE__,__LINE__);
				_model->getDynamicsEngine().getVelocity(*body,com,vVec);
				if(vVec[0] != vVec[0]) throw Exception("rdCMC_Point.computeAccelerations: ERROR- point task '" + getName() 
											+ "' references invalid acceleration components",__FILE__,__LINE__);
				// ADD TO WHOLE BODY MASS
				Mass += body->getMass();
				_p += body->getMass() * pVec;
				_v += body->getMass() * vVec;
			}

			//COMPUTE COM OF WHOLE BODY
			_p /= Mass;
			_v /= Mass;

		} else {

			_wrtBody =  bs->get(_wrtBodyName);

			_model->getDynamicsEngine().getPosition(*_wrtBody,_point,_p);
			if(_p[0] != _p[0]) throw Exception("rdCMC_Point.updateWorkVariables: ERROR- point task '" + getName() 
												+ "' references invalid position components",__FILE__,__LINE__);
			_model->getDynamicsEngine().getVelocity(*_wrtBody,_point,_v);
			if(_v[0] != _v[0]) throw Exception("rdCMC_Point.updateWorkVariables: ERROR- point task '" + getName() 
												+ "' references invalid velocity components",__FILE__,__LINE__);

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
rdCMC_Point& rdCMC_Point::
operator=(const rdCMC_Point &aTask)
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
void rdCMC_Point::
setModel(Model *aModel)
{
	rdCMC_Task::setModel(aModel);
	updateWorkVariables();
}
//-----------------------------------------------------------------------------
// Point
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the point that is to be tracked.
 *
 * @param aPoint Components of the tracked point.
 */
void rdCMC_Point::
setPoint(const SimTK::Vec3 &aPoint)
{
	_point = aPoint;
	updateWorkVariables();
}
//_____________________________________________________________________________
/**
 * Get the point that is to be tracked.
 *
 * @return Components of the tracked point.
 */
SimTK::Vec3 rdCMC_Point::
getPoint() const
{
	return(_point);
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
void rdCMC_Point::
computeErrors(double aT)
{
	updateWorkVariables();
	// COMPUTE ERRORS
	//std::cout<<getName()<<std::endl;
	//std::cout<<"_pTrk[0]->evaluate(0,aT) = "<<_pTrk[0]->evaluate(0,aT)<<std::endl;
	//std::cout<<"_pTrk[1]->evaluate(0,aT) = "<<_pTrk[1]->evaluate(0,aT)<<std::endl;
	//std::cout<<"_pTrk[2]->evaluate(0,aT) = "<<_pTrk[2]->evaluate(0,aT)<<std::endl;
	//std::cout<<"_inertialPTrk[0] = "<<_inertialPTrk[0]<<std::endl;
	//std::cout<<"_inertialPTrk[1] = "<<_inertialPTrk[1]<<std::endl;
	//std::cout<<"_inertialPTrk[2] = "<<_inertialPTrk[2]<<std::endl;
	//std::cout<<"_p = "<<_p<<std::endl;

	BodySet *bs = _model->getDynamicsEngine().getBodySet();

	_inertialPTrk = 0;
	_inertialVTrk = 0;
	if(_expressBodyName == "ground") {

		for(int i=0;i<3;i++) {
			_inertialPTrk[i] = _pTrk[i]->evaluate(0,aT);
			if(_vTrk[i]==NULL) {
				_inertialVTrk[i] = _pTrk[i]->evaluate(1,aT);
			} else {
				_inertialVTrk[i] = _vTrk[i]->evaluate(0,aT);
			}
		}

	} else {

		_expressBody =  bs->get(_expressBodyName);

		SimTK::Vec3 pVec,vVec,origin;

		for(int i=0;i<3;i++) {
			pVec(i) = _pTrk[i]->evaluate(0,aT);
		}
		_model->getDynamicsEngine().getPosition(*_expressBody,pVec,_inertialPTrk);
		if(_vTrk[0]==NULL) {
			_model->getDynamicsEngine().getVelocity(*_expressBody,pVec,_inertialVTrk);
		} else {
			for(int i=0;i<3;i++) {
				vVec(i) = _vTrk[i]->evaluate(0,aT);
			}
			_model->getDynamicsEngine().getVelocity(*_expressBody,origin,_inertialVTrk); // get velocity of _expressBody origin in inertial frame
			_inertialVTrk += vVec; // _vTrk is velocity in _expressBody, so it is simply added to velocity of _expressBody origin in inertial frame
		}

	}

	_pErr[0] = 0.0;
	_vErr[0] = 0.0;
	for(int j=0; j<3; j++) {
		_pErr[0] += _inertialPTrk[j]*_r0[j] - _p[j]*_r0[j];
		_vErr[0] += _inertialVTrk[j]*_r0[j] - _v[j]*_r0[j];
	}
	_pErr[1] = 0.0;
	_vErr[1] = 0.0;
	for(int j=0; j<3; j++) {
		_pErr[1] += _inertialPTrk[j]*_r1[j] - _p[j]*_r1[j];
		_vErr[1] += _inertialVTrk[j]*_r1[j] - _v[j]*_r1[j];
	}
	_pErr[2] = 0.0;
	_vErr[2] = 0.0;
	for(int j=0; j<3; j++) {
		_pErr[2] += _inertialPTrk[j]*_r2[j] - _p[j]*_r2[j];
		_vErr[2] += _inertialVTrk[j]*_r2[j] - _v[j]*_r2[j];
	}

}
//_____________________________________________________________________________
/**
 * Compute the desired accelerations.
 */
void rdCMC_Point::
computeDesiredAccelerations(double aT)
{
	_aDes=rdMath::getNAN();

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(aT);

	// DESIRED ACCELERATION
	double p;
	double v;
	double a;
	for(int i=0; i<3; i++) {
		p = (_kp)[0]*_pErr[i];
		v = (_kv)[0]*_vErr[i];
		if(_aTrk[i]==NULL) {
			a = (_ka)[0]*_pTrk[i]->evaluate(2,aT);
		} else {
			a = (_ka)[0]*_aTrk[i]->evaluate(0,aT);
		}
		_aDes[i] = a + v + p;
	}

	// PRINT
	//printf("rdCMC_Point.computeDesiredAcceleration:\n");
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
void rdCMC_Point::
computeDesiredAccelerations(double aTI,double aTF)
{
	_aDes=rdMath::getNAN();

	// CHECK
	if(_model==NULL) return;
	if(_pTrk[0]==NULL) return;

	// COMPUTE ERRORS
	computeErrors(aTI);

	// DESIRED ACCELERATION
	double p;
	double v;
	double a;
	for(int i=0; i<3; i++) {
		p = (_kp)[0]*_pErr[i];
		v = (_kv)[0]*_vErr[i];
		if(_aTrk[i]==NULL) {
			a = (_ka)[0]*_pTrk[i]->evaluate(2,aTF);
		} else {
			a = (_ka)[0]*_aTrk[i]->evaluate(0,aTF);
		}
		_aDes[i] = a + v + p;
	}

	// PRINT
	//printf("rdCMC_Point.computeDesiredAcceleration:\n");
	//printf("%s:  t=%lf aDes=%lf a=%lf vErr=%lf pErr=%lf\n",getName(),t,_aDes[0],
	//	_pTrk[0]->evaluate(2,t),_vErr[0],_pErr[0]);
}

//_____________________________________________________________________________
/**
 * Compute the acceleration of the appropriate point.
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
void rdCMC_Point::
computeAccelerations()
{
	// CHECK
	if(_model==NULL) return;

	// ACCELERATION
	_a = 0;
	BodySet *bs = _model->getDynamicsEngine().getBodySet();
	if(_wrtBodyName == "center_of_mass") {

		SimTK::Vec3 pVec,vVec,aVec,com;
		double Mass = 0.0;
		for(int i=0;i<bs->getSize();i++) {
			AbstractBody *body = bs->get(i);
			body->getMassCenter(com);
			_model->getDynamicsEngine().getAcceleration(*body,com,aVec);
			if(aVec[0] != aVec[0]) throw Exception("rdCMC_Point.computeAccelerations: ERROR- point task '" + getName() 
											+ "' references invalid acceleration components",__FILE__,__LINE__);
			// ADD TO WHOLE BODY MASS
			Mass += body->getMass();
			_a += body->getMass() * aVec;
		}

		//COMPUTE COM ACCELERATION OF WHOLE BODY
		_a /= Mass;

	} else {

		_wrtBody =  bs->get(_wrtBodyName);

		_model->getDynamicsEngine().getAcceleration(*_wrtBody,_point,_a);
		if(_a[0] != _a[0]) throw Exception("rdCMC_Point.computeAccelerations: ERROR- point task '" + getName() 
											+ "' references invalid acceleration components",__FILE__,__LINE__);
	}
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
void rdCMC_Point::
updateFromXMLNode()
{
	rdCMC_Task::updateFromXMLNode();
	setPoint(_point);
}
