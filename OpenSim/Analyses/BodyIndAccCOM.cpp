// BodyIndAccCOM.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn R. Goldberg 
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/Mtx.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include "BodyIndAccCOM.h"


//=============================================================================
// DEFINES
//=============================================================================


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
BodyIndAccCOM::~BodyIndAccCOM()
{
	// STORAGE
	if(_aeCOMBodyStore!=NULL){
		delete _aeCOMBodyStore;
		_aeCOMBodyStore=NULL;
	}
	if(_veCOMBodyStore!=NULL){
		delete _veCOMBodyStore;
		_veCOMBodyStore=NULL;
	}
	if(_peCOMBodyStore!=NULL){
		delete _peCOMBodyStore;
		_peCOMBodyStore=NULL;
	}
	if(_posStore!=NULL){
		delete _posStore;
		_posStore=NULL;
	}
	if(_velStore!=NULL){
		delete _velStore;
		_velStore=NULL;
	}
}
//_____________________________________________________________________________
/**
 * Construct an induced acceleration COM instance for performing an induced
 * acceleration COM analysis on the bodies of a model.
 *
 * @param aModel AbstractModel on which the analyses are to be performed.
 * @param aN Number of bodies that will be used to calculate COM
 * @param aBodyList Array containing aN body numbers corresponding to
 * 					  those used in calculating COM
 */
BodyIndAccCOM::BodyIndAccCOM(AbstractModel *aModel, int aN,
	 AbstractBody* aBodyList[]) : BodyIndAcc(aModel)
{
	setName("BodyIndAccCOM");

	// MEMBERS
	_aeCOMBodyStore = NULL;
	_veCOMBodyStore = NULL;
	_peCOMBodyStore = NULL;
	_posStore = NULL;
	_velStore = NULL;
	_aN = aN;
	setBodyList(aBodyList);
	
	// DESCRIPTION
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateBodyStorage();
}
//_____________________________________________________________________________
/**
 * Construct an induced acceleration COM instance from a set of force
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
 * @param aN Number of bodies that will be used to calculate COM
 * @param aBodyList Array containing aN body numbers corresponding to
 * 					  those used in calculating COM
 * @todo	verify induced accelerations are correct
 * @todo	check that code is correct for generalized force case
 * @todo add initial velocity and ind pos due to init vel and pos to all
 */
BodyIndAccCOM::BodyIndAccCOM(AbstractModel *aModel,Storage *aStates,
	Storage *aControls,char *aBaseName,char *aDir,char *aExtension,
	int aN,AbstractBody* aBodyList[]) :
	BodyIndAcc(aModel,aStates,aControls,aBaseName,aDir,aExtension)
{
	printf("BodyIndAccCOM: constructing COM induced acceleration analysis from file.\n");
	printf("baseName = %s  aDir = %s  aExtension= %s\n",
		aBaseName,aDir,aExtension);

	// MEMBERS
	_aeCOMBodyStore = NULL;
	_veCOMBodyStore = NULL;
	_peCOMBodyStore = NULL;
	_posStore = NULL;
	_velStore = NULL;
	_aN = aN;
	setBodyList(aBodyList);

	// DESCRIPTION
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateBodyStorage();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a the list of body numbers to use in COM calculation.
 * 
 * If the default value of aN=0 is set, a list is constructed which contains
 * all of the body numbers for the given model.
 */
void BodyIndAccCOM::
setBodyList(AbstractBody* aBodyList[])
{
	int i;	

	if (_aN==0){
		_aN = _model->getNumBodies();
		_aBodyList = new AbstractBody*[_aN];		
		i = 0;
		BodySet *bs = _model->getDynamicsEngine().getBodySet();

		for(i=0; i<bs->getSize(); i++)
		{
			AbstractBody *body=bs->get(i);
			_aBodyList[i++] = body;
		}
	} else {
		_aBodyList = new AbstractBody*[_aN];		
		for(i=0;i<_aN;i++)
			_aBodyList[i] = aBodyList[i];
	}
}

//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void BodyIndAccCOM::
constructDescription()
{
	char descrip[1024];
	char tmp[MAXLEN];
	char tmp2[MAXLEN];
	int i;

	strcpy(descrip,"\nThis file contains the induced accelerations");
	sprintf(tmp," of the COM of a given set of body segments in %s.\n",
		_model->getName().c_str());
	strcat(descrip,tmp);
	strcat(descrip,"\nThe bodies in the COM calculation were: \n");
	for(i=0;i<_aN;i++){
		sprintf(tmp2,"%s ", _aBodyList[i]->getName().c_str());
		strcat(descrip,tmp2);
	}
	strcat(descrip,"\n");
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
 * Construct column labels for the COM acceleration files.
 */
void BodyIndAccCOM::
constructColumnLabels()
{
	char labels[MAXLEN];

	// GET STATE NAMES
	int i;
	char name[MAXLEN];
	if(getComponentName(0)==NULL) {
		setColumnLabels(NULL);
	} else {
		strcpy(labels,"time");
		for(i=0;i<getNumComponents();i++) {
			sprintf(name,"\t%s_X",getComponentName(i));
			strcat(labels,name);
			sprintf(name,"\t%s_Y",getComponentName(i));
			strcat(labels,name);
			sprintf(name,"\t%s_Z",getComponentName(i));
			strcat(labels,name);
		}
		strcat(labels,"\n");
	}

	setColumnLabels(labels);
}


//_____________________________________________________________________________
/**
 * Allocate storage.
 */
void BodyIndAccCOM::
allocateBodyStorage()
{
	_aeCOMBodyStore = new Storage(500,"BodyCOMInducedAccelerations");
	_aeCOMBodyStore->setCapacityIncrement(100);
	_aeCOMBodyStore->setDescription(getDescription());
	_aeCOMBodyStore->setColumnLabels(getColumnLabels());

	_peCOMBodyStore = new Storage(500,"BodyCOMInducedPositions");
	_veCOMBodyStore = new Storage(500,"BodyCOMInducedVelocities");
	
	_posStore = new Storage(500,"BodyCOMPosition");
	_velStore = new Storage(500,"BodyCOMVelocity");
	
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________


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
void BodyIndAccCOM::
setStorageCapacityIncrements(int aIncrement)
{
	// BASE CLASS
	IndAcc::setStorageCapacityIncrements(aIncrement);

	// THIS CLASS
		_aeCOMBodyStore->setCapacityIncrement(aIncrement);
		_veCOMBodyStore->setCapacityIncrement(aIncrement);
		_peCOMBodyStore->setCapacityIncrement(aIncrement);
		_posStore->setCapacityIncrement(aIncrement);
		_velStore->setCapacityIncrement(aIncrement);	
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
int BodyIndAccCOM::
computeBodyCOMAccelerations()
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

	// COMPUTE THE ACCELERATIONS
	computeBodyAccelerations();
	sumBodyAccelerationResults();

	// COMPUTE THE COM ACCELERATIONS

	int I,J,c,nc,t,body, i;
	double bodyMass =0.0;
	double totMass = 0.0;
	double time = 0.0;
	Storage *dataStorage = NULL;
	StateVector *stateVec = NULL;
	StateVector *stateStoreVec = NULL;
	double *dataVec = NULL;
	double *statedataVec = NULL;
	double velVec[3];
	double posVec[3];
	double ivelVec[3];
	double iposVec[3];
	double b_velVec[3];
	double b_posVec[3];

	nc = 	getNumComponents();
	double *accVec = new double[nc*3];

	// LOOP OVER TIME STEPS
	for(t=0;t<_yStore->getSize();t++) {

		// INITIALIZE DATA VARIABLES
		for(int m=0;m<nc*3;m++)
			accVec[m] = 0.0;
		for(int n=0;n<3;n++){
			posVec[n] = 0.0;
			velVec[n] = 0.0;
		}
		totMass = 0.0;
		stateStoreVec = _yStore->getStateVector(t);
		statedataVec = stateStoreVec->getData().get();

		// LOOP OVER COMPONENTS
		for(c=0;c<nc;c++){
			//printf("\tComputing for component %s\n", getComponentName(c));
			dataStorage = getBodyStore(c);
			stateVec = dataStorage->getStateVector(t);
			dataVec = stateVec->getData().get();
			time = stateVec->getTime();	
			if(t==0) _ti = time;
			if(t==(_yStore->getSize() - 1)) _tf = time;

			for(body=0;body<_aN;body++) {
				bodyMass = _aBodyList[body]->getMass();
				if (c==0){ // only need to do this once for each time step
					double posCOM[3];
					_aBodyList[body]->getMassCenter(posCOM);
					_model->setStates(statedataVec);
					_model->getDynamicsEngine().getPosition(*_aBodyList[body],posCOM,b_posVec);
					_model->getDynamicsEngine().getVelocity(*_aBodyList[body],posCOM,b_velVec);
				}
				// COMPUTE
				for(int k=0;k<3;k++){	
					I = Mtx::ComputeIndex(body/*_aBodyList[body]*/,6,k); // TODOAUG
					J = Mtx::ComputeIndex(c,3,k);
					accVec[J] += bodyMass*dataVec[I];
					if (c==0){ // only need to do this once for each time step
						posVec[k] += bodyMass*b_posVec[k];
						velVec[k] += bodyMass*b_velVec[k];
					}		
				}
				if(c==0) // only need to do this for one body loop		
					totMass += bodyMass;
			}			
		}
		// STORE THE BODY ACCELERATIONS
		if(_aeCOMBodyStore==NULL) {
			_aeCOMBodyStore = new
				Storage(1000,"COMBodyInducedAccelerations");
			_aeCOMBodyStore->setDescription(getDescription());
			_aeCOMBodyStore->setColumnLabels(getColumnLabels());
		}			
		_aeCOMBodyStore->append(time,nc*3,accVec);
		_posStore->append(time,3,posVec);
		_velStore->append(time,3,velVec);
		if(t==0){
			for(i=0;i<3;i++){
				ivelVec[i] = velVec[i];
				iposVec[i] = posVec[i];
			}
		}

	}	
	_aeCOMBodyStore->divide(totMass);
	_posStore->divide(totMass);
	_velStore->divide(totMass);

	// COMPUTE THE INDUCED VELOCITIES AND POSITIONS
		
		// INTEGRATE TO GET VELOCITIES
		_veCOMBodyStore = _aeCOMBodyStore->integrate(_ti,_tf);

		// INTEGRATE TO GET POSITIONS
		_peCOMBodyStore = _veCOMBodyStore->integrate(_ti,_tf);

	// INDUCED POSITION DUE TO INITIAL POSITION AND INITIAL VELOCITY

		//STORE INITIAL VELOCITIES
		_iVelStore = new Storage(*_velStore, false);
		_iVelStore->append(_ti,3,ivelVec);
		_iVelStore->divide(totMass);

		// ADD INITIAL VELOCITY OF POINT TO ALL COLLUMN
		//NOTE: this funtionality is waiting for a collumn add feature

		// "INTEGRATE" INITIAL VELOCITES TO YIELD INITIAL POSITIONS
		_iPosStore = new Storage(*_posStore, false);
		_iPosStore->append(_ti,3,iposVec);
		_iPosStore->append(_tf,3,ivelVec);
		_iPosStore->getStateVector(1)->multiply(_tf -_ti);
		_iPosStore->getStateVector(1)->add(3,
			_iPosStore->getStateVector(0)->getData().get());
		_iPosStore->divide(totMass);

		// ADD INDUCED POSITION DUE TO INITIAL VELOCITY AND POSITION
		//NOTE: this funtionality is waiting for a collumn add feature


	return(0);
}


//=============================================================================
// UTILITY
//=============================================================================


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
int BodyIndAccCOM::
printResults(const std::string &aBaseName,const std::string &aDir,double aDT,
				 const std::string &aExtension)
{
	// COM INDUCED ACCELERATIONS
	Storage::printResult(_aeCOMBodyStore,"aCOMbody_%s"+aBaseName,aDir,aDT,aExtension);
	// COM INDUCED VELOCITIES
	Storage::printResult(_veCOMBodyStore,"vCOMbody_%s"+aBaseName,aDir,aDT,aExtension);
	// COM INDUCED POSITIONS
	Storage::printResult(_peCOMBodyStore,"pCOMbody_%s"+aBaseName,aDir,aDT,aExtension);
	// COM VELOCITIES
	Storage::printResult(_velStore,"COMbodyVelocity_"+aBaseName,aDir,aDT,aExtension);
	// COM POSITIONS
	Storage::printResult(_posStore,"COMbodyPosition_"+aBaseName,aDir,aDT,aExtension);
	//INITIAL VELOCITY
	Storage::printResult(_iVelStore,aBaseName+"_initVel",aDir,aDT,aExtension);
	//INDUCED POSTION DUE TO INITIAL VELOCITY AND POSITION
	Storage::printResult(_iPosStore,"p_"+aBaseName+"_initVelPos",aDir,(aDT<=0)?0.005:aDT,aExtension);

	return(0);
}
