// MuscleAnalysis.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Katherine Holzbaur, Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "MuscleAnalysis.h"
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>


using namespace OpenSim;
using namespace std;


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
MuscleAnalysis::~MuscleAnalysis()
{
}
//_____________________________________________________________________________
/**
 * Construct a MuscleAnalysis object for recording the MuscleAnalysis of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the MuscleAnalysis are to be recorded.
 */
MuscleAnalysis::MuscleAnalysis(Model *aModel) :
	Analysis(aModel)
{
	// NULL
	setNull();

	// CHECK MODEL
	if(_model==NULL) return;

	// STORAGE
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
MuscleAnalysis::MuscleAnalysis(const std::string &aFileName):
Analysis(aFileName, false)
{
	setNull();
	updateFromXMLNode();
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
MuscleAnalysis::MuscleAnalysis(const MuscleAnalysis &aMuscleAnalysis):
Analysis(aMuscleAnalysis)
{
	setNull();
	*this = aMuscleAnalysis;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* MuscleAnalysis::copy() const
{
	MuscleAnalysis *object = new MuscleAnalysis(*this);
	return(object);

}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void MuscleAnalysis::
setNull()
{
	setType("MuscleAnalysis");
	setName("MuscleAnalysis");
	setupProperties();
	constructDescription();

	// STORAGE
	_pennationAngleStore = NULL;
	_lengthStore = NULL;
	_fiberLengthStore = NULL;
	_normalizedFiberLengthStore = NULL;
	_tendonLengthStore = NULL;
	_forceStore = NULL;
	_fiberForceStore = NULL;
	_activeFiberForceStore = NULL;
	_passiveFiberForceStore = NULL;
	_activeFiberForceAlongTendonStore = NULL;
	_passiveFiberForceAlongTendonStore = NULL;

	// DEFAULT VALUES
	_muscleListProp.getValueStrArray().setSize(1);
	_muscleListProp.getValueStrArray().get(0) = "all";
	_coordinateListProp.getValueStrArray().setSize(1);
	_coordinateListProp.getValueStrArray().get(0) = "all";
}
//_____________________________________________________________________________
/**
 * Set up the properties.
 */
void MuscleAnalysis::
setupProperties()
{
	_muscleListProp.setComment("List of muscles for which to perform the analysis."
		" Use 'all' to perform the analysis for all muscles.");
	_muscleListProp.setName("muscle_list");
	_propertySet.append( &_muscleListProp );

	_coordinateListProp.setComment("List of generalized coordinates for which to "
		"compute moment arms. Use 'all' to compute for all coordinates.");
	_coordinateListProp.setName("moment_arm_coordinate_list");
	_propertySet.append( &_coordinateListProp );
}
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the MuscleAnalysis files.
 */
void MuscleAnalysis::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis analysis gathers basic information about muscles ");
	strcat(descrip,"during a simulation (e.g., forces, tendon lenghts, moment arms, etc).");

	strcat(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
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
 * Allocate storage for the muscle variables.
 */
void MuscleAnalysis::
allocateStorageObjects()
{
	if(_model==NULL) return;

	// CLEAR EXISTING WORK ARRAYS
	_storageList.setMemoryOwner(true);
	_storageList.setSize(0);
	_momentArmStorageArray.setMemoryOwner(true);
	_momentArmStorageArray.setSize(0);
	_muscleArray.setMemoryOwner(false);
	_muscleArray.setSize(0);

	// FOR MOMENT ARMS AND MOMEMTS
	CoordinateSet *qSet = _model->getDynamicsEngine().getCoordinateSet();
	int nq = qSet->getSize();
	Storage *store;
	for(int i=0;i<nq;i++) {
		AbstractCoordinate *q = qSet->get(i);
		string name = "MomentArm_" + q->getName();
		store = new Storage(1000,name);
		store->setDescription(getDescription());
		_storageList.append(store);
	}
	for(int i=0;i<nq;i++) {
		AbstractCoordinate *q = qSet->get(i);
		string name = "Moment_" + q->getName();
		store = new Storage(1000,name);
		store->setDescription(getDescription());
		_storageList.append(store);
	}

	// EVERYTHING ELSE
	_storageList.setMemoryOwner(false);
	_pennationAngleStore = new Storage(1000,"PennationAngle");
	_pennationAngleStore->setDescription(getDescription());
	_storageList.append(_pennationAngleStore );

	_lengthStore = new Storage(1000,"Length");
	_lengthStore->setDescription(getDescription());
	_storageList.append(_lengthStore );

	_fiberLengthStore = new Storage(1000,"FiberLength");
	_fiberLengthStore->setDescription(getDescription());
	_storageList.append(_fiberLengthStore );

	_normalizedFiberLengthStore = new Storage(1000,"NormalizedFiberLength");
	_normalizedFiberLengthStore->setDescription(getDescription());
	_storageList.append(_normalizedFiberLengthStore );

	_tendonLengthStore = new Storage(1000,"TendonLength");
	_tendonLengthStore->setDescription(getDescription());
	_storageList.append(_tendonLengthStore );

	_forceStore = new Storage(1000,"Force");
	_forceStore->setDescription(getDescription());
	_storageList.append(_forceStore );

	_fiberForceStore = new Storage(1000,"FiberForce");
	_fiberForceStore->setDescription(getDescription());
	_storageList.append(_fiberForceStore );

	_activeFiberForceStore = new Storage(1000,"ActiveFiberForce");
	_activeFiberForceStore->setDescription(getDescription());
	_storageList.append(_activeFiberForceStore );

	_passiveFiberForceStore = new Storage(1000,"PassiveFiberForce");
	_passiveFiberForceStore->setDescription(getDescription());
	_storageList.append(_passiveFiberForceStore );

	_activeFiberForceAlongTendonStore = new Storage(1000,"ActiveFiberForceAlongTendon");
	_activeFiberForceAlongTendonStore->setDescription(getDescription());
	_storageList.append(_activeFiberForceAlongTendonStore );

	_passiveFiberForceAlongTendonStore = new Storage(1000,"PassiveFiberForceAlongTendon");
	_passiveFiberForceAlongTendonStore->setDescription(getDescription());
	_storageList.append(_passiveFiberForceAlongTendonStore );

	// UPDATE ALL STORAGE OBJECTS
	updateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Update storage objects.  This is necessary if the modle, mucle, or
 * coordinate list is changed.
 */
void MuscleAnalysis::
updateStorageObjects()
{
	if(_model==NULL) return;

	// POPULATE MUSCLE LIST FOR "all"
	ActuatorSet *actSet = _model->getActuatorSet();
	_muscleList = _muscleListProp.getValueStrArray();
	int nm = _muscleList.getSize();
	if((nm==1) && (_muscleList.get(0)=="all")) {
		_muscleList.setSize(0);
		int na = actSet->getSize();
		for(int i=0;i<na;i++) {
			AbstractActuator *act = actSet->get(i);
			_muscleList.append(act->getName());
		}
	}
	// POPULATE ACTIVE MUSCLE ARRAY
	Array<string> tmpMuscleList("");
	nm = _muscleList.getSize();
	_muscleArray.setSize(0);
	for(int i=0; i<nm; i++) {
		AbstractMuscle *mus = dynamic_cast<AbstractMuscle*>( actSet->get(_muscleList[i]) );
		if(mus!=NULL) {
			_muscleArray.append(mus);
			tmpMuscleList.append(mus->getName());
		}
	}
	_muscleList = tmpMuscleList;

	// POPULATE COORDINATE LIST FOR "all"
	CoordinateSet *qSet = _model->getDynamicsEngine().getCoordinateSet();
	_coordinateList = _coordinateListProp.getValueStrArray();
	int nq = qSet->getSize();
	int nActiveQ = _coordinateList.getSize();
	if((nActiveQ==1) && (_coordinateList.get(0)=="all")) {
		_coordinateList.setSize(0);
		for(int i=0;i<nq;i++) {
			AbstractCoordinate *q = qSet->get(i);
			_coordinateList.append(q->getName());
		}
	}
	// POPULATE ACTIVE MOMENT ARM ARRAY
	Array<string> tmpCoordinateList("");  // For making sure the coordinates in the list really exist.
	_momentArmStorageArray.setSize(0);
	nActiveQ = _coordinateList.getSize();
	for(int i=0; i<nActiveQ; i++) {
		string name = _coordinateList[i];
		for(int j=0; j<nq; j++) {
			AbstractCoordinate *q = qSet->get(j);
			if(name == q->getName()) {
				StorageCoordinatePair *pair = new StorageCoordinatePair();
				pair->q = q;
				pair->momentArmStore = _storageList[j];
				pair->momentStore = _storageList[j+nq];
				_momentArmStorageArray.append(pair);
				tmpCoordinateList.append(q->getName());
			}
		}
	}
	_coordinateList = tmpCoordinateList;
	cout<<"Number of active moment arm storage array = "<<_momentArmStorageArray.getSize()<<endl;

	// CONSTRUCT AND SET COLUMN LABELS
	constructColumnLabels();
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->setColumnLabels(getColumnLabels());
	}
}
//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the column labels for the MuscleAnalysis storage files.
 */
void MuscleAnalysis::
constructColumnLabels()
{
	if(!_model) return;
	int size = _muscleList.getSize();
	Array<string> labels("",size+1);
	labels[0] = "time";
	for(int i=0; i<size; i++) {
		labels[i+1] = _muscleList[i];
	}
	setColumnLabels(labels);
}


//=============================================================================
// OPERATORS
//=============================================================================
MuscleAnalysis& MuscleAnalysis::operator=(const MuscleAnalysis &aAnalysis)
{
	// BASE CLASS
	Analysis::operator=(aAnalysis);

	// MEMBER VARIABLES
	_muscleListProp = aAnalysis._muscleListProp;
	_coordinateListProp = aAnalysis._coordinateListProp;
	allocateStorageObjects();

	return (*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void MuscleAnalysis::setModel(Model *aModel)
{
	Analysis::setModel(aModel);
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of muscles to analyze.
 *
 * @param aMuscles is the array of names of muscles to analyze.
 */
void MuscleAnalysis::
setMuscles(Array<std::string>& aMuscles)
{
	int size = aMuscles.getSize();
	_muscleListProp.getValueStrArray().setSize(aMuscles.getSize());
	for(int i=0; i<size; i++){
		_muscleListProp.getValueStrArray().get(i) = aMuscles.get(i);
	}
	updateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Set the list of coordinates.
 *
 * @param aCoordinates Array of coordinates about which to compute moment arms.
 */
void MuscleAnalysis::
setCoordinates(Array<std::string>& aCoordinates)
{
	int size = aCoordinates.getSize();
	_coordinateListProp.getValueStrArray().setSize(size);
	for(int i=0; i<size; i++){
		_coordinateListProp.getValueStrArray().get(i) = aCoordinates[i];
	}
	updateStorageObjects();
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
void MuscleAnalysis::
setStorageCapacityIncrements(int aIncrement)
{
	if(!_model) return;
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->setCapacityIncrement(aIncrement);
	}
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the MuscleAnalysis quantities.
 */
int MuscleAnalysis::
record(double aT,double *aX,double *aY)
{
	if(_model==NULL) return(-1);

	// MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
	// COMPUTE DERIVATIVES
	// ----------------------------------
	// SET
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// ACTUATION
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// TIME NORMALIZATION
	double tReal = aT * _model->getTimeNormConstant();
	// ----------------------------------

	// LOOP THROUGH MUSCLES
	int nm = _muscleArray.getSize();
	Array<double> penang(0.0,nm);
	Array<double> len(0.0,nm),tlen(0.0,nm);
	Array<double> fiblen(0.0,nm),normfiblen(0.0,nm);
	Array<double> force(0.0,nm),fibforce(0.0,nm);
	Array<double> actfibforce(0.0,nm),passfibforce(0.0,nm);
	Array<double> actfibforcealongten(0.0,nm),passfibforcealongten(0.0,nm);
	for(int i=0; i<nm; i++) {
		penang[i] = _muscleArray[i]->getPennationAngle();
		len[i] = _muscleArray[i]->getLength();
		tlen[i] = _muscleArray[i]->getTendonLength();
		fiblen[i] = _muscleArray[i]->getFiberLength();
		normfiblen[i] = _muscleArray[i]->getNormalizedFiberLength();
		force[i] = _muscleArray[i]->getForce();
		fibforce[i] = _muscleArray[i]->getFiberForce();
		actfibforce[i] = _muscleArray[i]->getActiveFiberForce();
		passfibforce[i] = _muscleArray[i]->getPassiveFiberForce();
		actfibforcealongten[i] = _muscleArray[i]->getActiveFiberForceAlongTendon();
		passfibforcealongten[i] = _muscleArray[i]->getPassiveFiberForceAlongTendon();
	}
	// APPEND TO STORAGE
	_pennationAngleStore->append(tReal,penang.getSize(),&penang[0]);
	_lengthStore->append(tReal,len.getSize(),&len[0]);
	_fiberLengthStore->append(tReal,fiblen.getSize(),&fiblen[0]);
	_normalizedFiberLengthStore->append(tReal,normfiblen.getSize(),&normfiblen[0]);
	_tendonLengthStore->append(tReal,tlen.getSize(),&tlen[0]);
	_forceStore->append(tReal,force.getSize(),&force[0]);
	_fiberForceStore->append(tReal,fibforce.getSize(),&fibforce[0]);
	_activeFiberForceStore->append(tReal,actfibforce.getSize(),&actfibforce[0]);
	_passiveFiberForceStore->append(tReal,passfibforce.getSize(),&passfibforce[0]);
	_activeFiberForceAlongTendonStore->append(tReal,actfibforcealongten.getSize(),&actfibforcealongten[0]);
	_passiveFiberForceAlongTendonStore->append(tReal,passfibforcealongten.getSize(),&passfibforcealongten[0]);

	// LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
	AbstractCoordinate *q = NULL;
	Storage *maStore=NULL, *mStore=NULL;
	int nq = _momentArmStorageArray.getSize();
	Array<double> ma(0.0,nm),m(0.0,nm);
	for(int i=0; i<nq; i++) {

		q = _momentArmStorageArray[i]->q;
		maStore = _momentArmStorageArray[i]->momentArmStore;
		mStore = _momentArmStorageArray[i]->momentStore;

		// LOOP OVER MUSCLES
		for(int j=0; j<nm; j++) {
			ma[j] = _muscleArray[j]->computeMomentArm(*q);
			m[j] = ma[j] * force[j];
		}
		maStore->append(aT,nm,&ma[0]);
		mStore->append(aT,nm,&m[0]);
	}

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * Model::integBeginCallback() and has the same argument list.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that will be attempted.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	Storage *store;
	int size = _storageList.getSize();
	for(int i=0;i<size;i++) {
		store = _storageList[i];
		if(store==NULL) continue;
		store->purge();
	}

	// RECORD
	int status = 0;
	if(_storageList.get(0)->getSize() <= 0) status = record(aT,aX,aY);

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * Model::integStepCallback(), which has the same argument list.
 *
 * This method should be overriden in derived classes.  It is
 * included here so that the derived class will not have to implement it if
 * it is not necessary.
 *
 * @param aXPrev Controls at the beginining of the current time step.
 * @param aYPrev States at the beginning of the current time step.
 * @param aYPPrev Pseudo states at the beginning of the current time step.
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
step(double *aXPrev,double *aYPrev,double *aYPPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	int status = record(aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if (!proceed()) return 0;
	record(aT,aX,aY);
	return(0);
}

//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * Model::integEndCallback() and has the same argument list.
 *
 * This method should be overriden in the child class.  It is
 * included here so that the child class will not have to implement it if it
 * is not necessary.
 *
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just completed.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("MuscleAnalysis.printResults: Off- not printing.\n");
		return(0);
	}

	std::string prefix = aBaseName + "_" + getName() + "_";
	Storage::printResult(_pennationAngleStore,prefix+"PennationAngle",aDir,aDT,aExtension);
	Storage::printResult(_lengthStore,prefix+"Length",aDir,aDT,aExtension);
	Storage::printResult(_fiberLengthStore,prefix+"FiberLength",aDir,aDT,aExtension);
	Storage::printResult(_normalizedFiberLengthStore,prefix+"NormalizedFiberLength",aDir,aDT,aExtension);
	Storage::printResult(_tendonLengthStore,prefix+"TendonLength",aDir,aDT,aExtension);
	Storage::printResult(_forceStore,prefix+"Force",aDir,aDT,aExtension);
	Storage::printResult(_fiberForceStore,prefix+"FiberForce",aDir,aDT,aExtension);
	Storage::printResult(_activeFiberForceStore,prefix+"ActiveFiberForce",aDir,aDT,aExtension);
	Storage::printResult(_passiveFiberForceStore,prefix+"PassiveFiberForce",aDir,aDT,aExtension);
	Storage::printResult(_activeFiberForceAlongTendonStore,prefix+"ActiveFiberForceAlongTendon",aDir,aDT,aExtension);
	Storage::printResult(_passiveFiberForceAlongTendonStore,prefix+"PassiveFiberForceAlongTendon",aDir,aDT,aExtension);

	int size = _momentArmStorageArray.getSize();
	for(int i=0;i<size;i++) {
		string fileName = prefix + _momentArmStorageArray.get(i)->momentArmStore->getName();
		Storage::printResult(_momentArmStorageArray.get(i)->momentArmStore,fileName,aDir,aDT,aExtension);
		fileName = prefix + _momentArmStorageArray.get(i)->momentStore->getName();
		Storage::printResult(_momentArmStorageArray.get(i)->momentStore,fileName,aDir,aDT,aExtension);
	}

	return(0);
}

