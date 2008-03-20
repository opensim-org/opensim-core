// BodyIndPowers.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Frank C. Anderson, Saryn R. Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
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
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include "BodyIndPowers.h"


//=============================================================================
// DEFINES
//=============================================================================


using namespace std;
using namespace OpenSim;
using SimTK::Vec3;
using SimTK::Mat33;

#define MAXLEN 2048


//=============================================================================
// CONSTANTS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
BodyIndPowers::~BodyIndPowers()
{
	// STORAGE
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced power instance for performing an induced
 * power analysis on the bodies of a model.
 *
 * @param aModel Model on which the analyses are to be performed.
 */
BodyIndPowers::BodyIndPowers(Model *aModel) :
	BodyIndAcc(aModel)
{
	setName("BodyInducedPowers");

	// MEMBERS
	_powerStore = NULL;

	// DESCRIPTION
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced powers instance from a set of force
 * decomposition files.
 *
 * Note that the induced accelerations are not read in from file.  The
 * induced accelerations are recomputed based on the force decomposition.
 *
 * @param aModel Model on which the analyses were performed.
 * @param aStates Set of model states.
 * @param aBaseName Base name for the force decompositon files.
 * @param aDir Directory in which the results reside.
 * @param aExtension File extension.
 */
BodyIndPowers::BodyIndPowers(Model *aModel,Storage *aStates,Storage *aControls,
	char *aBaseName,char *aDir,char *aExtension) :
	BodyIndAcc(aModel,aStates,aControls,aBaseName,aDir,aExtension)
{
	printf("BodyIndAcc: constructing from file.\n");
	printf("baseName = %s  aDir = %s  aExtension= %s\n",
		aBaseName,aDir,aExtension);

	// NAME
	setName("BodyInducedPowers");

		// DESCRIPTION
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStoragePointers();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body powers files.
 */
void BodyIndPowers::
constructDescription()
{
	char descrip[1024];
	char tmp[MAXLEN];

	strcpy(descrip,"\nThis file contains the induced powers");
	sprintf(tmp," of the body segments in model %s.\n",
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nInduced powers are given about");
	strcat(descrip," the body-local axes.\n");
	strcat(descrip,"\nUnits are S.I. units (seconds, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");

	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body powers files.
 */
void BodyIndPowers::
constructColumnLabels()
{
	// GET STATE NAMES
	Array<string> labels;
	labels.append("time");
	BodySet *bs = _model->getDynamicsEngine().getBodySet();
	for(int i=0; i<bs->getSize(); i++) labels.append(bs->get(i)->getName());
	labels.append("Total");
	setColumnLabels(labels);

}

//_____________________________________________________________________________
/**
 * Allocate storage pointers for the induced powers of the bodies.
 */
void BodyIndPowers::
allocateStoragePointers()
{
	_powerStore = new Storage*[_nc];
	int c;
	for(c=0;c<_nc;c++) {
		_powerStore[c] = NULL;
	}
}
//_____________________________________________________________________________
/**
 * Allocate storage.
 */
void BodyIndPowers::
allocateStorage()
{
	// ALLOCATE STORAGE POINTERS
	allocateStoragePointers();

	// ALLOCATE STORAGE
	int c;
	for(c=0;c<_nc;c++) {
		_powerStore[c] = new Storage(500,"BodyInducedPowers");
		_powerStore[c]->setCapacityIncrement(100);
		_powerStore[c]->setDescription(getDescription());
		_powerStore[c]->setColumnLabels(getColumnLabels());
	}
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void BodyIndPowers::
deleteStorage()
{
	// INDUCED POWERS
	int c;
	if(_powerStore!=NULL) {
		for(c=0;c<_nc;c++) {
			if(_powerStore[c]!=NULL) { delete _powerStore[c];  _powerStore[c]=NULL; }
		}
		delete []_powerStore;
	}
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// STORAGE CAPACITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the capacity increments of all storage instances.
 *
 * @param aIncrement Increment by which storage capacities will be increased
 * when storage capcities run out.
 */
void BodyIndPowers::
setStorageCapacityIncrements(int aIncrement)
{
	// BASE CLASS
	IndAcc::setStorageCapacityIncrements(aIncrement);

	// THIS CLASS
	int c;
	for(c=0;c<_nc;c++) {
		_powerStore[c]->setCapacityIncrement(aIncrement);
	}
}


//=============================================================================
// OPERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the induced powers of the bodies of a model from a
 * force decomposition and a set of states.
 *
 * @return 0 on success, -1 on error.
 */
int BodyIndPowers::
computeBodyPowers()
{
	if(_yStore==NULL) return(-1);

	// CHECK FOR TIME CORRESPONDENCE
	if(!getUseNullDecomposition()) {
		double ti = _feStore[0]->getFirstTime();
		double tf = _feStore[0]->getLastTime();
		if((ti!=_yStore->getFirstTime())||(tf!=_yStore->getLastTime())) {
			printf("BodyIndPowers.computeBodyPowers: WARN-\n");
			printf("\tTime range for states is %lf to %lf\n",
				_yStore->getFirstTime(),_yStore->getLastTime());
			printf("\tTime range for force decomposition is %lf to %lf\n",
				ti,tf);
		}
	}

	// NUMBERS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int nb = _model->getNumBodies();
	int ny = _model->getNumStates();
	int np = _model->getNumContacts();

	// GRAVITY
	SimTK::Vec3 g, grav;
	SimTK::Vec3 g0(0.0, 0.0, 0.0);
	_model->getGravity(g);
	printf("gravity = %lf %lf %lf\n",g[0],g[1],g[2]);

	// LOOP OVER TIME
	int i,J,c;
	SimTK::Vec3 pointB;
	double t;
	double *y = new double[ny];
	double *fe = new double[3*np];
	double *dqdt = new double[nq];
	double *dudt = new double[nu];
	int npwr = nb + 1;
	double *indPower = new double[npwr];
	SimTK::Vec3 com(0.0, 0.0, 0.0);
	SimTK::Vec3 acc,angAcc;
	double *vel = new double[nb*3];
	double *angVel = new double[nb*3];
	StateVector *yVec;
	int bodyIndex;
	BodySet *bs = NULL;
	AbstractBody *body, *bodyB;

	for(i=0;i<_yStore->getSize();i++) {

		// GET STATES
		yVec = _yStore->getStateVector(i);
		t = yVec->getTime();
		_yStore->getDataAtTime(t,ny,y);

		// SET STATES
		_model->setTime(t);
		_model->setStates(y);
		
		// GET VELOCITIES
		// Need to store them for each body for power calculation 
		//	before we set them to zero for acceleration calculation
		bodyIndex = 0;
		bs = _model->getDynamicsEngine().getBodySet();
		int j=0;
		for(j=0; j<bs->getSize(); j++)
		{
			body = bs->get(j);
			int k = Mtx::ComputeIndex(bodyIndex++,3,0);
			_model->getDynamicsEngine().getVelocity(*body,com,Vec3::updAs(&vel[k]));
			_model->getDynamicsEngine().getAngularVelocityBodyLocal(*body,Vec3::updAs(&angVel[k]));
		}

		// LOOP OVER INDEPENDENT COMPONENTS
		int ai=0;	// index for looping over actuators
		ActuatorSet* as = _model->getActuatorSet();
		AbstractActuator* act = as->get(ai);
		for(c=0;c<_nic;c++) {

			// SET GRAVITY
			if(c!=getGravityIndex()) {
				_model->setGravity(g0);
			} else {
				_model->setGravity(g);
			}

			// GET ELEMENT FORCES
			if(!getUseNullDecomposition()) {
				_feStore[c]->getDataAtTime(t,3*np,fe);
			}

			// GET STATES
			_yStore->getDataAtTime(t,ny,y);

			// SET TIME AND STATES
			if(c!=getVelocityIndex()) {
				for(j=0;j<nu;j++) y[nq+j] = 0.0;
			}
			_model->setTime(t);
			_model->setStates(y);

			// APPLY ACTUATOR FORCE
			if(c<_model->getNumActuators()) {
				act->apply();
			}

			// APPLY ELEMENT FORCES
			for(j=0;j<_model->getNumContacts();j++) {
				J = Mtx::ComputeIndex(j,3,0);
				bodyB = _model->getContactSet()->getContactBodyB(j);
				_model->getContactSet()->getContactPointB(j,pointB);
				_model->getDynamicsEngine().applyForce(*bodyB,pointB,Vec3::getAs(&fe[J]));
			}

			// COMPUTE THE ACCELERATIONS
			_model->getDynamicsEngine().computeDerivatives(dqdt,dudt);
		
			// COMPUTE THE BODY POWERS
			bodyIndex = 0;
			int bsi = 0;
			BodySet* bs =_model->getDynamicsEngine().getBodySet();
			
			for(bsi=0; bsi < bs->getSize(); bsi++)
			{
				body = bs->get(bsi);
				// GET ACCELERATIONS
				_model->getDynamicsEngine().getAcceleration(*body,com,acc);
				_model->getDynamicsEngine().getAngularAccelerationBodyLocal(*body,angAcc);
	
				// COMPUTE POWER
				double gravPower, linPower, angPower, power;
				Mat33 inertia;
				SimTK::Vec3 result;
				
				int k = Mtx::ComputeIndex(bodyIndex,3,0);

				// GRAVITY
				_model->getGravity(grav);
				gravPower = -body->getMass() * Mtx::DotProduct(3,grav,Vec3::getAs(&vel[k]));

				// LINEAR KINETIC
				linPower = body->getMass() * Mtx::DotProduct(3,Vec3::getAs(&vel[k]),acc);

				// ANGULAR KINETIC
				body->getInertia(inertia);
				Mtx::Multiply(3,3,1,&inertia[0][0],&angAcc[0],&result[0]);
				angPower =  Mtx::DotProduct(3,Vec3::getAs(&angVel[k]),result);

				// TOTAL
				power = gravPower + linPower + angPower;

				// FILL ARRAY
				indPower[bodyIndex] = power;

			}

			// SUM POWERS
			for(indPower[npwr-1]=0.0,bodyIndex=0;bodyIndex<nb;bodyIndex++) {
				indPower[npwr-1] += indPower[bodyIndex];
			}

			// STORE THE BODY ACCELERATIONS
			if(_powerStore[c]==NULL) {
				_powerStore[c] = new
					Storage(1000,"BodyInducedPowers");
				_powerStore[c]->setDescription(getDescription());
				_powerStore[c]->setColumnLabels(getColumnLabels());
			}
			_powerStore[c]->append(t,npwr,indPower);
			ai++;
			act = as->get(ai);
		}
	}

	// RESET GRAVITY
	_model->setGravity(g);

	// CLEANUP
	if(y!=NULL) { delete[] y;  y=NULL; }
	if(fe!=NULL) { delete[] fe;  fe=NULL; }
	if(dqdt!=NULL) { delete[] dqdt;  dqdt=NULL; }
	if(dudt!=NULL) { delete[] dudt;  dudt=NULL; }
	if(indPower!=NULL) { delete[] indPower;  indPower=NULL; }
	if(vel!=NULL) { delete[] vel;  vel=NULL; }
	if(angVel!=NULL) { delete[] angVel;  angVel=NULL; }

	return(0);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Sum the power results.
 */
void BodyIndPowers::
sumPowerResults()
{
	if(_powerStore[0]==NULL) return;

	// SUM ACROSS ACTUATORS
	int c;
	int cAct = getAllActuatorsIndex();
	if(_powerStore[cAct]!=NULL) delete _powerStore[cAct];
	_powerStore[cAct] = new Storage(*_powerStore[0]);
	for(c=1;c<=getLastActuatorIndex();c++) {
		if(_powerStore[c]!=NULL) _powerStore[cAct]->add(_powerStore[c]);
	}

	// SUMM ALL COMPONENTS
	int cAll = getAllIndex();
	if(_powerStore[cAll]!=NULL) delete _powerStore[cAll];
	_powerStore[cAll] = new Storage(*_powerStore[cAct]);
	_powerStore[cAll]->add(_powerStore[_cGrav]);
	_powerStore[cAll]->add(_powerStore[_cVel]);
	_powerStore[cAll]->add(_powerStore[_cIner]);
}


//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print results.
 * 
 * The file names are constructed as
 * aDir + "/" + aBaseName + "_" + ComponentName + aExtension
 *
 * @param aDir Directory in which the results reside.
 * @param aBaseName Base file name.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int BodyIndPowers::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// SUM RESULTS
	sumPowerResults();

	// COMPONENTS
	for(int c=0;c<_nc;c++) {
		Storage::printResult(_powerStore[c],aBaseName+"_"+getName()+"_"+_cNames[c],aDir,aDT,aExtension);
	}

	return(0);
}
