// BodyIndPowers.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Frank C. Anderson, Saryn R. Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Simulation/Simm/AbstractModel.h>
#include <OpenSim/Simulation/Simm/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Simm/BodySet.h>
#include <OpenSim/Simulation/Simm/ActuatorSet.h>
#include "BodyIndPowers.h"


//=============================================================================
// DEFINES
//=============================================================================


using namespace std;
using namespace OpenSim;
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
 * @param aModel AbstractModel on which the analyses are to be performed.
 */
BodyIndPowers::BodyIndPowers(AbstractModel *aModel) :
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
 * @param aModel AbstractModel on which the analyses were performed.
 * @param aStates Set of model states.
 * @param aBaseName Base name for the force decompositon files.
 * @param aDir Directory in which the results reside.
 * @param aExtension File extension.
 */
BodyIndPowers::BodyIndPowers(AbstractModel *aModel,Storage *aStates,Storage *aControls,
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
	string labels = "time";
	BodySet *bs = _model->getDynamicsEngine().getBodySet();
	int i=0;

	for(i=0; i<bs->getSize(); i++)
	{
		AbstractBody *body=bs->get(i);
		labels += "\t";
		labels += body->getName();
	}

	labels += "\tTotal\n";

	setColumnLabels(labels.c_str());

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
	double g0[] = { 0.0, 0.0, 0.0 };
	double g[3],grav[3];
	_model->getGravity(g);
	printf("gravity = %lf %lf %lf\n",g[0],g[1],g[2]);

	// LOOP OVER TIME
	int i,J,c;
	double pointB[3];
	double t;
	double *y = new double[ny];
	double *fe = new double[3*np];
	double *dqdt = new double[nq];
	double *dudt = new double[nu];
	int npwr = nb + 1;
	double *indPower = new double[npwr];
	double com[] = { 0.0, 0.0, 0.0 };
	double acc[3],angAcc[3];
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
			_model->getDynamicsEngine().getVelocity(*body,com,&vel[k]);
			_model->getDynamicsEngine().getAngularVelocityBodyLocal(*body,&angVel[k]);
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
				_model->getDynamicsEngine().applyForce(*bodyB,pointB,&fe[J]);
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
				double inertia[3][3];
				double result[3];
				
				int k = Mtx::ComputeIndex(bodyIndex,3,0);

				// GRAVITY
				_model->getGravity(grav);
				gravPower = -body->getMass() * Mtx::DotProduct(3,grav,&vel[k]);

				// LINEAR KINETIC
				linPower = body->getMass() * Mtx::DotProduct(3,&vel[k],acc);

				// ANGULAR KINETIC
				body->getInertia(inertia);
				Mtx::Multiply(3,3,1,&inertia[0][0],angAcc,result);
				angPower =  Mtx::DotProduct(3,&angVel[k],result);

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
printResults(const char *aBaseName,const char *aDir,double aDT,
				 const char *aExtension)
{
	char baseName[NAME_LENGTH];
	if(aBaseName==NULL) {
		strcpy(baseName,"null");
	} else {
		strncpy(baseName,aBaseName,NAME_LENGTH);
		baseName[NAME_LENGTH-1] = 0;
	}

	// CONSTRUCT PATH
	char path[NAME_LENGTH];
	if(aDir==NULL) {
		strcpy(path,".");
	} else {
		strcpy(path,aDir);
	}

	// SUM RESULTS
	sumPowerResults();

	// COMPONENTS
	int c;
	char name[2048];
	for(c=0;c<_nc;c++) {

		// POWERS
		if(aExtension==NULL) {
			sprintf(name,"%s/%s_%s_%s",path,baseName,getName().c_str(),_cNames[c]);
		} else {
			sprintf(name,"%s/%s_%s_%s%s",path,baseName,getName().c_str(),_cNames[c],
				aExtension);
		}
		if(aDT<=0.0) {
			if(_powerStore[c]!=NULL) _powerStore[c]->print(name);
		} else {
			if(_powerStore[c]!=NULL) _powerStore[c]->print(name,aDT);
		}

	}

	return(0);
}
