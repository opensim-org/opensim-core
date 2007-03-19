// BodyIndAcc.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Simulation/Model/AbstractModel.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include "BodyIndAcc.h"




using namespace OpenSim;
using namespace std;


//=============================================================================
// DEFINES
//=============================================================================
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
BodyIndAcc::~BodyIndAcc()
{
	// STORAGE
	deleteBodyStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced acceleration instance for performing an induced
 * acceleration analysis on the bodies of a model.
 *
 * @param aModel Model on which the analyses are to be performed.
 */
BodyIndAcc::BodyIndAcc(AbstractModel *aModel) :
	IndAcc(aModel)
{
	setName("BodyIndAcc");

	// MEMBERS
	_aeBodyStore = NULL;

	// DESCRIPTION
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateBodyStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced acceleration instance from a set of force
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
 * @todo	verify induced accelerations are correct
 * @todo	check that code is correct for generalized force case
 */
BodyIndAcc::BodyIndAcc(AbstractModel *aModel,Storage *aStates,Storage *aControls,
	char *aBaseName,char *aDir,char *aExtension) :
	IndAcc(aModel,aStates,aControls,aBaseName,aDir,aExtension)
{
	printf("BodyIndAcc: constructing induced acceleration analysis from file.\n");
	printf("baseName = %s  aDir = %s  aExtension= %s\n",
		aBaseName,aDir,aExtension);

	// DESCRIPTION
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateBodyStoragePointers();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void BodyIndAcc::
constructDescription()
{
	char descrip[1024];
	char tmp[MAXLEN];

	strcpy(descrip,"\nThis file contains the induced accelerations");
	sprintf(tmp," of the body segments in model %s.\n",
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nInduced angular accelerations are given about");
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
 * Construct column labels for the body kinematics files.
 */
void BodyIndAcc::
constructColumnLabels()
{
	string labels = "time";
	BodySet *bs = _model->getDynamicsEngine().getBodySet();
	int i=0;

	for(i=0; i<bs->getSize(); i++)
	{
		AbstractBody *body = bs->get(i);
		labels += "\t" + body->getName() + "_X";
		labels += "\t" + body->getName() + "_Y";
		labels += "\t" + body->getName() + "_Z";
		labels += "\t" + body->getName() + "_Ox";
		labels += "\t" + body->getName() + "_Oy";
		labels += "\t" + body->getName() + "_Oz";
	}

	labels += "\n";

	setColumnLabels(labels.c_str());

}

//_____________________________________________________________________________
/**
 * Allocate storage pointers for the induced accelerations of the bodies.
 */
void BodyIndAcc::
allocateBodyStoragePointers()
{
	_aeBodyStore = new Storage*[_nc];
	int c;
	for(c=0;c<_nc;c++) {
		_aeBodyStore[c] = NULL;
	}
}
//_____________________________________________________________________________
/**
 * Allocate storage.
 */
void BodyIndAcc::
allocateBodyStorage()
{
	// ALLOCATE STORAGE POINTERS
	allocateBodyStoragePointers();

	// ALLOCATE STORAGE
	int c;
	for(c=0;c<_nc;c++) {
		_aeBodyStore[c] = new Storage(500,"BodyInducedAccelerations");
		_aeBodyStore[c]->setCapacityIncrement(100);
		_aeBodyStore[c]->setDescription(getDescription());
		_aeBodyStore[c]->setColumnLabels(getColumnLabels());
	}
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void BodyIndAcc::
deleteBodyStorage()
{
	// INDUCED ACCELERATIONS
	int c;
	if(_aeBodyStore!=NULL) {
		for(c=0;c<_nc;c++) {
			if(_aeBodyStore[c]!=NULL) { delete _aeBodyStore[c];  _aeBodyStore[c]=NULL; }
		}
		delete []_aeBodyStore;
	}
}


//=============================================================================
// GET AND SET
//=============================================================================
/**
 * Returns a pointer to an _aeBodyStore.
 *
 * @param index The index of the aeBodyStore to return
 */

Storage* BodyIndAcc::
getBodyStore(int index)
{
	if(index < 0 || index >= _nc)
		return(NULL);
	else
		return(_aeBodyStore[index]);
}

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
void BodyIndAcc::
setStorageCapacityIncrements(int aIncrement)
{
	// BASE CLASS
	IndAcc::setStorageCapacityIncrements(aIncrement);

	// THIS CLASS
	int c;
	for(c=0;c<_nc;c++) {
		_aeBodyStore[c]->setCapacityIncrement(aIncrement);
	}
}


//=============================================================================
// OPERATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the induced accelerations of the bodies of a model from a
 * force decomposition and a set of states.
 *
 * @return 0 on success, -1 on error.
 */
int BodyIndAcc::
computeBodyAccelerations()
{
	if(_yStore==NULL) return(-1);

	// CHECK FOR TIME CORRESPONDENCE
	if(!getUseNullDecomposition()) {
		double ti = _feStore[0]->getFirstTime();
		double tf = _feStore[0]->getLastTime();
		if((ti!=_yStore->getFirstTime())||(tf!=_yStore->getLastTime())) {
			printf("BodyIndAcc.computeAccelerations: WARN-\n");
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
	int nx = _model->getNumControls();
	int np = _model->getNumContacts();

	// GRAVITY
	double g[3];
	double g0[] = { 0.0, 0.0, 0.0 };
	_model->getGravity(g);
	printf("gravity = %lf %lf %lf\n",g[0],g[1],g[2]);

	// LOOP OVER TIME
	int i,j,c,I,J;
	AbstractBody *bodyB;
	double pointB[3];
	double t;
	StateVector *yVec;
	double *y = new double[ny];
	double *x = new double[nx];
	double *fe = new double[3*np];
	double *dqdt = new double[nq];
	double *dudt = new double[nu];
	double *indAcc = new double[6*nb];
	double *initVel = new double[6*nb];
	double *initPos = new double[6*nb];
	double acc[3],angAcc[3], vel[3], angVel[3], pos[3], angPos[3];
	double dirCos[3][3];

	AbstractDynamicsEngine &engine = _model->getDynamicsEngine();
	BodySet *bs = engine.getBodySet();
	int bsi=0;	// BodySet index;
	AbstractBody *body;
	int bodyIndex;

	for(i=0;i<_yStore->getSize();i++) {

		// LOOP OVER INDEPENDENT COMPONENTS
		ActuatorSet* as = _model->getActuatorSet();
		int asi=0;
		AbstractActuator* act = as->get(asi);
		for(c=0;c<_nic;c++) {

			// SET GRAVITY
			if(c!=getGravityIndex()) {
				_model->setGravity(g0);
			} else {
				_model->setGravity(g);
			}

			// GET STATES
			yVec = _yStore->getStateVector(i);
			t = yVec->getTime();
			if(i==0) _ti = t;
			if(i==(_yStore->getSize() - 1)) _tf = t;
			_yStore->getDataAtTime(t,ny,y);

			// GET CONTROLS
			if(_xStore!=NULL)
				_xStore->getDataAtTime(t,nx,x);

			// GET CONTACT POINT FORCES
			if(!getUseNullDecomposition()) {
				_feStore[c]->getDataAtTime(t,3*np,fe);
			}

			// SET
			if(c!=getVelocityIndex()) {
				for(j=0;j<nu;j++) y[nq+j] = 0.0;
			}
			_model->set(t,x,y);

			// COMPUTE ACTUATION
			_model->getActuatorSet()->computeActuation();

			// NEED TO RECORD THE INITIAL BODY VELOCITIES AND POSITIONS
				if(c==getVelocityIndex() && t==_ti){
					_model->getActuatorSet()->apply();
					engine.computeDerivatives(dqdt,dudt);

					bsi=0;
					bodyIndex = 0;
					for(bsi=0; bsi < bs->getSize(); bsi++)
					{
						body = bs->get(bsi);

						double com[3];
						body->getMassCenter(com);

						engine.getVelocity(*body,com,vel);
						engine.getAngularVelocityBodyLocal(*body,angVel);

						engine.getPosition(*body,com,pos);	
						engine.getDirectionCosines(*body, dirCos);
						engine.convertDirectionCosinesToAngles(dirCos,&angPos[0],&angPos[1],
								&angPos[2]);
		
						// DEGREES?
						if(getInDegrees()) {
							for(j=0;j<3;j++) {
								angVel[j] *= rdMath::RTD;
								angPos[j] *= rdMath::RTD;	
							}
						}
	
						// FILL ARRAY
						I = Mtx::ComputeIndex(bodyIndex,6,0);
						memcpy(&initVel[I],vel,3*sizeof(double));
						memcpy(&initVel[I+3],angVel,3*sizeof(double));
					
						memcpy(&initPos[I],pos,3*sizeof(double));
						memcpy(&initPos[I+3],angPos,3*sizeof(double));
						bodyIndex++;
					}

					// RESET STATES
					_model->set(t,x,y);

					// RECOMPUTE ACTUATION
					_model->getActuatorSet()->computeActuation();		
				}

			// APPLY ACTUATOR FORCE
			if(c<_model->getNumActuators()) {
				act->apply();
			}

			// APPLY ELEMENT FORCES
			if(!getUseNullDecomposition()) {
				for(j=0;j<np;j++) {
					J = Mtx::ComputeIndex(j,3,0);
					bodyB = _model->getContactSet()->getContactBodyB(j);
					_model->getContactSet()->getContactPointB(j,pointB);
					engine.applyForce(*bodyB,pointB,&fe[J]);
				}
			}

			// COMPUTE THE ACCELERATIONS
			engine.computeDerivatives(dqdt,dudt);

			// COMPUTE THE BODY ACCELERATIONS
			bsi=0;
			bodyIndex = 0;
			for(bsi=0; bsi < bs->getSize(); bsi++)
			{
				body = bs->get(bsi);
				// COMPUTE
				double com[3];
				body->getMassCenter(com);
				engine.getAcceleration(*body,com,acc);
				engine.getAngularAccelerationBodyLocal(*body,angAcc);

				// DEGREES?
				if(getInDegrees()) {
					for(j=0;j<3;j++) {
						angAcc[j] *= rdMath::RTD;
					}
				}

				// FILL ARRAY
				I = Mtx::ComputeIndex(bodyIndex,6,0);
				memcpy(&indAcc[I],acc,3*sizeof(double));
				memcpy(&indAcc[I+3],angAcc,3*sizeof(double));
				bodyIndex++;
			}
	
			// STORE THE BODY ACCELERATIONS
			if(_aeBodyStore[c]==NULL) {
				_aeBodyStore[c] = new
					Storage(1000,"BodyInducedAccelerations");
				_aeBodyStore[c]->setDescription(getDescription());
				_aeBodyStore[c]->setColumnLabels(getColumnLabels());
			}
			_aeBodyStore[c]->append(t,6*nb,indAcc);

			asi++;
			act = as->get(asi);
		}
	}

	// COMPUTE THE INDUCED VELOCITIES AND POSITIONS
	for(c=0;c<_nic;c++) {
		
		// INTEGRATE TO GET VELOCITIES
		_velStore[c] = _aeBodyStore[c]->integrate(_ti,_tf);

		// INTEGRATE TO GET POSITIONS
		_posStore[c] = _velStore[c]->integrate(_ti,_tf);
	}

	// INDUCED POSITION DUE TO INITIAL POSITION AND INITIAL VELOCITY
	
		//STORE INITIAL VELOCITIES
		_iVelStore = new Storage(*_yStore, false);
		_iVelStore->append(_ti,6*nb,&initVel[0]);

		// "INTEGRATE" INITIAL VELOCITES TO YIELD INITIAL POSITIONS
		_iPosStore = new Storage(*_yStore, false);
		_iPosStore->append(_ti,6*nb,&initPos[0]);
		_iPosStore->append(_tf,6*nb,&initVel[0]);
		_iPosStore->getStateVector(1)->multiply(_tf -_ti);
		_iPosStore->getStateVector(1)->add(nu,_iPosStore->getStateVector(0)->getData().get());

	// RESET GRAVITY
	_model->setGravity(g);

	// CLEANUP
	if(y!=NULL) { delete[] y;  y=NULL; }
	if(dqdt!=NULL) { delete[] dqdt;  dqdt=NULL; }
	if(dudt!=NULL) { delete[] dudt;  dudt=NULL; }
	if(indAcc!=NULL) { delete[] indAcc;  indAcc=NULL; }

	return(0);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Sum the accelerations results.
 */
void BodyIndAcc::
sumBodyAccelerationResults()
{
	if(_aeBodyStore[0]==NULL) return;

	// SUM ACROSS ACTUATORS
	int c;
	int cAct = getAllActuatorsIndex();
	if(_aeBodyStore[cAct]!=NULL) delete _aeBodyStore[cAct];
	_aeBodyStore[cAct] = new Storage(*_aeBodyStore[0]);
	for(c=1;c<=getLastActuatorIndex();c++) {
		if(_aeBodyStore[c]!=NULL) _aeBodyStore[cAct]->add(_aeBodyStore[c]);
	}
			
		// INTEGRATE TO GET VELOCITIES
		_velStore[cAct] = _aeBodyStore[cAct]->integrate(_ti,_tf);

		// INTEGRATE TO GET POSITIONS
		_posStore[cAct] = _velStore[cAct]->integrate(_ti,_tf);

	// SUM ALL COMPONENTS
	int cAll = getAllIndex();
	if(_aeBodyStore[cAll]!=NULL) delete _aeBodyStore[cAll];
	_aeBodyStore[cAll] = new Storage(*_aeBodyStore[cAct]);
	_aeBodyStore[cAll]->add(_aeBodyStore[_cGrav]);
	_aeBodyStore[cAll]->add(_aeBodyStore[_cVel]);
	_aeBodyStore[cAll]->add(_aeBodyStore[_cIner]);

		// INTEGRATE TO GET VELOCITIES
		_velStore[cAll] = _aeBodyStore[cAll]->integrate(_ti,_tf);

		// INTEGRATE TO GET POSITIONS
		_posStore[cAll] = _velStore[cAll]->integrate(_ti,_tf);

		// ADD INITIAL VELOCITIES
		StateVector *vVec;
		vVec = _iVelStore->getStateVector(0);
		_velStore[cAll]->add(vVec);

		// ADD POSITION DUE TO INITIAL VELOCITIES AND POSITIONS
		_posStore[cAll]->add(_iPosStore);
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
int BodyIndAcc::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// SUM RESULTS
	sumBodyAccelerationResults();

	// COMPONENTS
	for(int c=0;c<_nc;c++) {
		// ACCELERATIONS
		Storage::printResult(_aeBodyStore[c],"abody_"+aBaseName+"_"+_cNames[c],aDir,aDT,aExtension);
		// VELOCITIES
		Storage::printResult(_velStore[c],"vbody_"+aBaseName+"_"+_cNames[c],aDir,aDT,aExtension);
		// POSITIONS
		Storage::printResult(_posStore[c],"pbody_"+aBaseName+"_"+_cNames[c],aDir,aDT,aExtension);
	}

	//INITIAL VELOCITY
	Storage::printResult(_iVelStore,aBaseName+"_initVelBody",aDir,aDT,aExtension);
	//INDUCED POSTION DUE TO INITIAL VELOCITY AND POSITION
	Storage::printResult(_iPosStore,"p_"+aBaseName+"_initVelPos",aDir,(aDT<=0)?0.005:aDT,aExtension);

	return(0);
}
