// Actuation.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/AbstractModel.h>
#include <OpenSim/Simulation/Model/AbstractActuator.h>
#include "Actuation.h"




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
Actuation::~Actuation()
{
	if(_fsp!=NULL) { delete[] _fsp;  _fsp=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an Actuation object for recording the Actuation of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the Actuation are to be recorded.
 */
Actuation::Actuation(AbstractModel *aModel) :
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

	// NUMBER OF ACTUATORS
	_na = _model->getNumActuators();
	if(_na<=0) return;

	// WORK ARRAY
	_fsp = new double[_na];

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
Actuation::Actuation(const std::string &aFileName):
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
Actuation::Actuation(const Actuation &aActuation):
Analysis(aActuation)
{
	setNull();
	// COPY TYPE AND NAME
	*this = aActuation;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* Actuation::copy() const
{
	Actuation *object = new Actuation(*this);
	return(object);

}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void Actuation::
setNull()
{
	// TYPE
	setType("Actuation");
	// NAME
	setName("Actuation");

	_na = 0;
	_fsp = NULL;
	_forceStore = NULL;
	_speedStore = NULL;
	_powerStore = NULL;
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
Actuation& Actuation::operator=(const Actuation &aActuation)
{
	// BASE CLASS
	Analysis::operator=(aActuation);

	// Deallocate _fsp if already allocated
	if(_fsp!=NULL) { delete[] _fsp;  _fsp=NULL; }

	// STORAGE
	deleteStorage();
	allocateStorage();

	// CHECK MODEL
	if(_model!=NULL) {
		_na = _model->getNumActuators();
		_fsp = new double[_na];
		constructColumnLabels();
	}

	return (*this);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analysis.
 */
void Actuation::setModel(AbstractModel *aModel)
{
	// BASE CLASS
	Analysis::setModel(aModel);

	if (_model)
		_na = _model->getNumActuators();
	else
		_na = 0;

	if(_na<=0) return;

	// WORK ARRAY
	if(_fsp!=NULL) delete[] _fsp;
	_fsp = new double[_na];

	// UPDATE LABELS
	constructColumnLabels();
}
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void Actuation::
allocateStorage()
{
	// ACCELERATIONS
	_forceStore = new Storage(1000,"ActuatorForces");
	_forceStore->setDescription(getDescription());

	// VELOCITIES
	_speedStore = new Storage(1000,"ActuatorSpeeds");
	_speedStore->setDescription(getDescription());

	// POSITIONS
	_powerStore = new Storage(1000,"ActuatorPowers");
	_powerStore->setDescription(getDescription());
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the Actuation files.
 */
void Actuation::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nThis file contains either the forces, speeds, or ");
	strcat(descrip,"powers developed\nby the actuators of a model ");
	strcat(descrip,"during a simulation.\n");

	strcat(descrip,"\nAn actuator force is a generalized force, meaning that");
	strcat(descrip," it can be either a force (N) or a torque (Nm).\n");

	strcat(descrip,"\nAn actuator speed is the rate at which an actuator ");
	strcat(descrip,"shortens. Depending on the actuator,\na speed can be ");
	strcat(descrip,"either a translational speed (m/s) or an angular speed ");
	strcat(descrip,"(deg/s or rad/s).\n");

	strcat(descrip,"\nAn actuator power (Watts) is the rate at which an ");
	strcat(descrip,"actuator does work.  Positive power means\nthat the ");
	strcat(descrip,"actuator is delivering energy to the model; negative ");
	strcat(descrip,"power means that the actuator\nis absorbing energy ");
	strcat(descrip,"from the model.\n");

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
 * Construct the column labels for the actuation storage files.
 */
void Actuation::
constructColumnLabels()
{
	if (_model)
	{
		// ASSIGN
		string labels = "time";
		ActuatorSet *ai = _model->getActuatorSet();
		AbstractActuator *act=0;
	
		int i;
		for (i=0; i < ai->getSize(); i++)
		{
			act = ai->get(i);
			labels += "\t";
			labels += act->getName();
		}

		labels += "\n";

		setColumnLabels(labels.c_str());

		if (_forceStore)
			_forceStore->setColumnLabels(getColumnLabels());
		if (_speedStore)
			_speedStore->setColumnLabels(getColumnLabels());
		if (_powerStore)
			_powerStore->setColumnLabels(getColumnLabels());
	}
	_forceStore->setColumnLabels(getColumnLabels());
	_speedStore->setColumnLabels(getColumnLabels());
	_powerStore->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void Actuation::
deleteStorage()
{
	if(_forceStore!=NULL) { delete _forceStore;  _forceStore=NULL; }
	if(_speedStore!=NULL) { delete _speedStore;  _speedStore=NULL; }
	if(_powerStore!=NULL) { delete _powerStore;  _powerStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the force storage.
 *
 * @return Force storage.
 */
Storage* Actuation::
getForceStorage() const
{
	return(_forceStore);
}
//_____________________________________________________________________________
/**
 * Get the speed storage.
 *
 * @return Speed storage.
 */
Storage* Actuation::
getSpeedStorage() const
{
	return(_speedStore);
}
//_____________________________________________________________________________
/**
 * Get the power storage.
 *
 * @return Power storage.
 */
Storage* Actuation::
getPowerStorage() const
{
	return(_powerStore);
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
void Actuation::
setStorageCapacityIncrements(int aIncrement)
{
	_forceStore->setCapacityIncrement(aIncrement);
	_speedStore->setCapacityIncrement(aIncrement);
	_powerStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the actuation quantities.
 */
int Actuation::
record(double aT,double *aX,double *aY)
{
	if(_model==NULL) return(-1);

	// MAKE SURE ALL ACTUATION QUANTITIES ARE VALID
	_model->getActuatorSet()->computeActuation();

	// NUMBER OF ACTUATORS
	int na = _model->getNumActuators();
	if(na!=_na) {
		printf("Actuation.record: WARN- number of actuators has changed!\n");
		_na = na;
		if(_na<=0) return(-1);

		// REALLOCATE WORK ARRAY
		if(_fsp!=NULL) delete[] _fsp;
		_fsp = new double[_na];
	}

	// TIME NORMALIZATION
	double tReal = aT * _model->getTimeNormConstant();

	// FORCE
	ActuatorSet *as = _model->getActuatorSet();
	for(int i=0; i<as->getSize(); i++)
		_fsp[i] = as->get(i)->getForce();
	_forceStore->append(tReal,na,_fsp);

	// SPEED
	for(int i=0; i<as->getSize(); i++)
		_fsp[i] = as->get(i)->getSpeed();
	_speedStore->append(tReal,na,_fsp);

	// POWER
	for(int i=0; i<as->getSize(); i++)
		_fsp[i] = as->get(i)->getPower();
	_powerStore->append(tReal,na,_fsp);


	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the beginning of an analysis so that any
 * necessary initializations may be performed.
 *
 * This method is meant to be called at the begining of an integration in
 * AbstractModel::integBeginCallback() and has the same argument list.
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
int Actuation::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_forceStore->reset(aT);
	_speedStore->reset(aT);
	_powerStore->reset(aT);

	// RECORD
	int status = 0;
	if(_forceStore->getSize()<=0) {
		status = record(aT,aX,aY);
	}

	return(status);
}
//_____________________________________________________________________________
/**
 * This method is called to perform the analysis.  It can be called during
 * the execution of a forward integrations or after the integration by
 * feeding it the necessary data.
 *
 * When called during an integration, this method is meant to be called in
 * AbstractModel::integStepCallback(), which has the same argument list.
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
int Actuation::
step(double *aXPrev,double *aYPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	record(aT,aX,aY);

	return(0);
}
//_____________________________________________________________________________
/**
 * This method is called at the end of an analysis so that any
 * necessary finalizations may be performed.
 *
 * This method is meant to be called at the end of an integration in
 * AbstractModel::integEndCallback() and has the same argument list.
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
int Actuation::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if (!proceed()) return 0;

	record(aT,aX,aY);

	return(0);
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
 * @param aDT Desired time interval between adjacent storage vectors.  Linear
 * interpolation is used to print the data out at the desired interval.
 * @param aExtension File extension.
 *
 * @return 0 on success, -1 on error.
 */
int Actuation::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("Actuation.printResults: Off- not printing.\n");
		return(0);
	}

	std::string prefix=aBaseName+"_"+getName()+"_";
	Storage::printResult(_forceStore, prefix+"force", aDir, aDT, aExtension);
	Storage::printResult(_speedStore, prefix+"speed", aDir, aDT, aExtension);
	Storage::printResult(_powerStore, prefix+"power", aDir, aDT, aExtension);

	return(0);
}


