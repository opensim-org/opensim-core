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
#include "MuscleAnalysis.h"




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
	deleteStorage();
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

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocateStorage();

	// CHECK MODEL
	if(_model==NULL) return;

	// LABELS
	constructColumnLabels();
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
Analysis(aFileName)
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// DESCRIPTION
	constructDescription();

	// STORAGE
	allocateStorage();
}
// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
MuscleAnalysis::MuscleAnalysis(const MuscleAnalysis &aMuscleAnalysis):
Analysis(aMuscleAnalysis)
{
	setNull();
	// COPY TYPE AND NAME
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
	// TYPE
	setType("MuscleAnalysis");
	// NAME
	setName("MuscleAnalysis");

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
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
MuscleAnalysis& MuscleAnalysis::operator=(const MuscleAnalysis &aMuscleAnalysis)
{
	// BASE CLASS
	Analysis::operator=(aMuscleAnalysis);

	// STORAGE
	deleteStorage();
	allocateStorage();

	return (*this);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void MuscleAnalysis::setModel(Model *aModel)
{
	Analysis::setModel(aModel);
	constructColumnLabels();
}
//_____________________________________________________________________________
/**
 * Allocate storage for the muscle variables.
 */
void MuscleAnalysis::
allocateStorage()
{
	_pennationAngleStore = new Storage(1000,"PennationAngle");
	_lengthStore = new Storage(1000,"Length");
	_fiberLengthStore = new Storage(1000,"FiberLength");
	_normalizedFiberLengthStore = new Storage(1000,"NormalizedFiberLength");
	_tendonLengthStore = new Storage(1000,"FiberLength");
	_forceStore = new Storage(1000,"Force");
	_fiberForceStore = new Storage(1000,"FiberForce");
	_activeFiberForceStore = new Storage(1000,"ActiveFiberForce");
	_passiveFiberForceStore = new Storage(1000,"PassiveFiberForce");
	_activeFiberForceAlongTendonStore = new Storage(1000,"ActiveFiberAlongTendonForce");
	_passiveFiberForceAlongTendonStore = new Storage(1000,"PassiveFiberAlongTendonForce");

	_lengthStore->setDescription(getDescription());

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

	strcpy(descrip,"\nThis file contains the muscle-tendon lengths ");
	strcat(descrip,"developed\nby the muscle actuators of a model ");
	strcat(descrip,"during a simulation.\n");

	strcat(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
	if(getInDegrees()) {
		strcat(descrip,"\nAngles are in degrees.");
	} else {
		strcat(descrip,"\nAngles are in radians.");
	}
	strcat(descrip,"\n\n");

	setDescription(descrip);
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
	// Make labels
	if (_model) {
		ActuatorSet *ai = _model->getActuatorSet();
		AbstractActuator *act=0;
		int size = ai->getSize();
		Array<string> labels("",size+1);
	
		labels[0] = "time";
		for(int i=0; i<size; i++) {
			act = ai->get(i);
			labels[i+1] = act->getName();
		}
		setColumnLabels(labels);
	}

	// Set labels on storages.
	_pennationAngleStore->setColumnLabels(getColumnLabels());
	_lengthStore->setColumnLabels(getColumnLabels());
	_fiberLengthStore->setColumnLabels(getColumnLabels());
	_normalizedFiberLengthStore->setColumnLabels(getColumnLabels());
	_tendonLengthStore->setColumnLabels(getColumnLabels());
	_forceStore->setColumnLabels(getColumnLabels());
	_fiberForceStore->setColumnLabels(getColumnLabels());
	_activeFiberForceStore->setColumnLabels(getColumnLabels());
	_passiveFiberForceStore->setColumnLabels(getColumnLabels());
	_activeFiberForceAlongTendonStore->setColumnLabels(getColumnLabels());
	_passiveFiberForceAlongTendonStore->setColumnLabels(getColumnLabels());
}



//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void MuscleAnalysis::
deleteStorage()
{
	if(_pennationAngleStore!=NULL) {
		delete _pennationAngleStore;
		_pennationAngleStore=NULL;
	}
	if(_lengthStore!=NULL) {
		delete _lengthStore;
		_lengthStore=NULL;
	}
	if(_fiberLengthStore!=NULL) {
		delete _fiberLengthStore;
		_fiberLengthStore=NULL;
	}
	if(_normalizedFiberLengthStore!=NULL) {
		delete _normalizedFiberLengthStore;
		_normalizedFiberLengthStore=NULL;
	}
	if(_tendonLengthStore!=NULL) {
		delete _tendonLengthStore;
		_tendonLengthStore=NULL;
	}
	if(_forceStore!=NULL) {
		delete _forceStore;
		_forceStore=NULL;
	}
	if(_fiberForceStore!=NULL) {
		delete _fiberForceStore;
		_fiberForceStore=NULL;
	}
	if(_activeFiberForceStore!=NULL) {
		delete _activeFiberForceStore;
		_activeFiberForceStore=NULL;
	}
	if(_passiveFiberForceStore!=NULL) {
		delete _passiveFiberForceStore;
		_passiveFiberForceStore=NULL;
	}
	if(_activeFiberForceAlongTendonStore!=NULL) {
		delete _activeFiberForceAlongTendonStore;
		_activeFiberForceAlongTendonStore=NULL;
	}
	if(_passiveFiberForceAlongTendonStore!=NULL) {
		delete _passiveFiberForceAlongTendonStore;
		_passiveFiberForceAlongTendonStore=NULL;
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
void MuscleAnalysis::
setStorageCapacityIncrements(int aIncrement)
{
	_pennationAngleStore->setCapacityIncrement(aIncrement);
	_lengthStore->setCapacityIncrement(aIncrement);
	_fiberLengthStore->setCapacityIncrement(aIncrement);
	_normalizedFiberLengthStore->setCapacityIncrement(aIncrement);
	_tendonLengthStore->setCapacityIncrement(aIncrement);
	_forceStore->setCapacityIncrement(aIncrement);
	_fiberForceStore->setCapacityIncrement(aIncrement);
	_activeFiberForceStore->setCapacityIncrement(aIncrement);
	_passiveFiberForceStore->setCapacityIncrement(aIncrement);
	_activeFiberForceAlongTendonStore->setCapacityIncrement(aIncrement);
	_passiveFiberForceAlongTendonStore->setCapacityIncrement(aIncrement);
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
	_model->getActuatorSet()->computeActuation();

	// TIME NORMALIZATION
	double tReal = aT * _model->getTimeNormConstant();

	// LOOP THROUGH MUSCLES
	ActuatorSet *as = _model->getActuatorSet();
	int size = as->getSize();
	Array<double> penang(0.0,size);
	Array<double> len(0.0,size),tlen(0.0,size);
	Array<double> fiblen(0.0,size),normfiblen(0.0,size);
	Array<double> force(0.0,size),fibforce(0.0,size);
	Array<double> actfibforce(0.0,size),passfibforce(0.0,size);
	Array<double> actfibforcealongten(0.0,size),passfibforcealongten(0.0,size);
	for(int i=0; i<size; i++) {
		//Check if actuator is actually a muscle actuator
		//if it is not, go on to next actuator in list
		AbstractMuscle *mus = dynamic_cast<AbstractMuscle*>( as->get(i) );
		if (mus==NULL) continue;
		penang[i] = mus->getPennationAngle();
		len[i] = mus->getLength();
		tlen[i] = mus->getTendonLength();
		fiblen[i] = mus->getFiberLength();
		normfiblen[i] = mus->getNormalizedFiberLength();
		force[i] = mus->getForce();
		fibforce[i] = mus->getFiberForce();
		actfibforce[i] = mus->getActiveFiberForce();
		passfibforce[i] = mus->getPassiveFiberForce();
		actfibforcealongten[i] = mus->getActiveFiberForceAlongTendon();
		passfibforcealongten[i] = mus->getPassiveFiberForceAlongTendon();
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
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	// SET COLUMN LABELS
	constructColumnLabels();

	// RESET STORAGE
	_pennationAngleStore->reset(aT);
	_lengthStore->reset(aT);
	_fiberLengthStore->reset(aT);
	_normalizedFiberLengthStore->reset(aT);
	_tendonLengthStore->reset(aT);
	_forceStore->reset(aT);
	_fiberForceStore->reset(aT);
	_activeFiberForceStore->reset(aT);
	_passiveFiberForceStore->reset(aT);
	_activeFiberForceAlongTendonStore->reset(aT);
	_passiveFiberForceAlongTendonStore->reset(aT);

	// RECORD
	int status = 0;
	if(_lengthStore->getSize()<=0) status = record(aT,aX,aY);

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
 * @param aStep Step number of the integration.
 * @param aDT Size of the time step that was just taken.
 * @param aT Current time in the integration.
 * @param aX Current control values.
 * @param aY Current states.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
step(double *aXPrev,double *aYPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,
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
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int MuscleAnalysis::
end(int aStep,double aDT,double aT,double *aX,double *aY,
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

	std::string prefix=aBaseName+"_"+getName()+"_";
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

	return(0);
}

