// MomentArmAnalysis.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "MomentArmAnalysis.h"
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
MomentArmAnalysis::~MomentArmAnalysis()
{
}
//_____________________________________________________________________________
/**
 * Construct a MomentArmAnalysis object for recording the MomentArmAnalysis of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the MomentArmAnalysis are to be recorded.
 */
MomentArmAnalysis::MomentArmAnalysis(Model *aModel) :
	Analysis(aModel)
{
	// NULL
	setNull();

	// CHECK MODEL
	if(_model==NULL) return;

	// STORAGE OBJECTS
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
MomentArmAnalysis::MomentArmAnalysis(const std::string &aFileName):
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
MomentArmAnalysis::MomentArmAnalysis(const MomentArmAnalysis &aMomentArmAnalysis):
	Analysis(aMomentArmAnalysis)
{
	setNull();
	*this = aMomentArmAnalysis;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* MomentArmAnalysis::copy() const
{
	MomentArmAnalysis *object = new MomentArmAnalysis(*this);
	return(object);

}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void MomentArmAnalysis::
setNull()
{
	// TYPE
	setType("MomentArmAnalysis");
	// NAME
	setName("MomentArmAnalysis");

	// PROPERTIES
	setupProperties();

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
void MomentArmAnalysis::
setupProperties()
{
	_muscleListProp.setComment("List of muscles for which to perform the analysis."
		" Use 'all' to perform the analysis for all muscles.");
	_muscleListProp.setName("muscle_list");
	_propertySet.append( &_muscleListProp );

	_coordinateListProp.setComment("List of generalized coordinates for which to "
		"perform the analysis. Use 'all' to perform the analysis for all coordinates.");
	_coordinateListProp.setName("coordinate_list");
	_propertySet.append( &_coordinateListProp );
}



//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
MomentArmAnalysis& MomentArmAnalysis::operator=(const MomentArmAnalysis &aMomentArmAnalysis)
{
	// BASE CLASS
	Analysis::operator=(aMomentArmAnalysis);

	// MEMBER VARIABLES
	_muscleListProp = aMomentArmAnalysis._muscleListProp;
	_coordinateListProp = aMomentArmAnalysis._coordinateListProp;
	allocateStorageObjects();

	return (*this);
}

//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void MomentArmAnalysis::setModel(Model *aModel)
{
	Analysis::setModel(aModel);
	allocateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Allocate storage for the moment arm data.
 */
void MomentArmAnalysis::
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

	// CREATE NEW STORAGES
	CoordinateSet *qSet = _model->getDynamicsEngine().getCoordinateSet();
	int nq = qSet->getSize();
	Storage *store;
	for(int i=0;i<nq;i++) {
		AbstractCoordinate *q = qSet->get(i);
		string name = getName() + "_" + q->getName();
		store = new Storage(1000,name);
		_storageList.append(store);
	}

	// UPDATE THE COLUMN LABELS ECT.
	updateStorageObjects();
}
//_____________________________________________________________________________
/**
 * Update storage objects.  This is necessary if the mucle or coordinate list
 * is changed.
 */
void MomentArmAnalysis::
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
	Array<string> tmpCoordinateList("");
	_momentArmStorageArray.setSize(0);
	nActiveQ = _coordinateList.getSize();
	for(int i=0; i<nActiveQ; i++) {
		string name = _coordinateList[i];
		for(int j=0; j<nq; j++) {
			AbstractCoordinate *q = qSet->get(j);
			if(name == q->getName()) {
				StorageCoordinatePair *pair = new StorageCoordinatePair();
				pair->q = q;
				pair->store = _storageList[j];
				_momentArmStorageArray.append(pair);
				tmpCoordinateList.append(q->getName());
			}
		}
	}
	_coordinateList = tmpCoordinateList;
	cout<<"Number of active moment arm storage array = "<<_momentArmStorageArray.getSize()<<endl;


	// CONSTRUCT AND SET COLUMN LABELS
	constructColumnLabels();
	string name;
	Storage *store;
	for(int i=0;i<nq;i++) {
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
 * Construct the column labels for the MomentArmAnalysis storage files.
 */
void MomentArmAnalysis::
constructColumnLabels()
{
	if(_model) {
		int size = _muscleList.getSize();
		Array<string> labels("",size+1);
		labels[0] = "time";
		for(int i=0; i<size; i++) {
			labels[i+1] = _muscleList[i];
		}
		setColumnLabels(labels);
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
void MomentArmAnalysis::
setStorageCapacityIncrements(int aIncrement)
{
	int size = _momentArmStorageArray.getSize();
	for(int i=0;i<size;i++) {
		_momentArmStorageArray[i]->store->setCapacityIncrement(aIncrement);
	}
}
//_____________________________________________________________________________
/**
 * Set the list of muscles to analyze.
 *
 * @param aMuscles is the array of names of muscles to analyze.
 */
void MomentArmAnalysis::
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
void MomentArmAnalysis::
setCoordinates(Array<std::string>& aCoordinates)
{
	int size = aCoordinates.getSize();
	_coordinateListProp.getValueStrArray().setSize(size);
	for(int i=0; i<size; i++){
		_coordinateListProp.getValueStrArray().get(i) = aCoordinates[i];
	}
	updateStorageObjects();
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the MomentArmAnalysis quantities.
 */
int MomentArmAnalysis::
record(double aT,double *aX,double *aY)
{
	if(_model==NULL) return(-1);

	// MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
	// COMPUTE DERIVATIVES
	// ----------------------------------
	// Set
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// Actuation
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// Time normalization
	double tReal = aT * _model->getTimeNormConstant();
	//-----------------------------------

	// LOOP OVER ACTIVE MOMENT ARM STORAGE OBJECTS
	AbstractCoordinate *q = NULL;
	Storage *store = NULL;
	int nq = _momentArmStorageArray.getSize();
	int nm = _muscleArray.getSize();
	Array<double> ma(0.0,nm);
	for(int i=0; i<nq; i++) {

		q = _momentArmStorageArray[i]->q;
		store = _momentArmStorageArray[i]->store;

		// LOOP OVER MUSCLES
		for(int j=0; j<nm; j++) {
			ma[j] = _muscleArray[j]->computeMomentArm(*q);
		}
		store->append(aT,nm,&ma[0]);
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
int MomentArmAnalysis::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	int size = _momentArmStorageArray.getSize();
	for(int i=0;i<size;i++) {
		_momentArmStorageArray.get(i)->store->reset(aT);
	}

	// RECORD
	int status = 0;
	if(_momentArmStorageArray.get(0)->store->getSize() <=0 ) status = record(aT,aX,aY);

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
int MomentArmAnalysis::
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
int MomentArmAnalysis::
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
int MomentArmAnalysis::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("MomentArmAnalysis.printResults: Off- not printing.\n");
		return(0);
	}

	int size = _momentArmStorageArray.getSize();
	for(int i=0;i<size;i++) {

		string fileName = aBaseName + "_" + _momentArmStorageArray.get(i)->store->getName();
		cout<<"fileName = "<<fileName<<endl;
		Storage::printResult(_momentArmStorageArray.get(i)->store,fileName,aDir,aDT,aExtension);
	}

	return(0);
}

