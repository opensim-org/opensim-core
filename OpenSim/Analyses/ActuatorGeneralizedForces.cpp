// ActuatorGeneralizedForces.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn Goldberg
//
//	Note that the computeGeneralizedForces routine used in this code implements
// the SDFast routine sdcomptrq, which does not take into account contraint forces.
// If a model includes constraint forces, you will need to write a function that
// calls sdfulltrq, which is passed the Lagrange multipliers associated with the
// constraints and can take them into account.  See page R-25 of the SDFast
// manual.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/Storage.h>
#include "ActuatorGeneralizedForces.h"




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
ActuatorGeneralizedForces::~ActuatorGeneralizedForces()
{
	if(_dqdt!=NULL) { delete[] _dqdt;  _dqdt=NULL; }
	if(_dudt!=NULL) { delete[] _dudt;  _dudt=NULL; }

	if(_actuatorGenForces!=NULL) {
		delete[] _actuatorGenForces;  _actuatorGenForces=NULL;
	}
	if(_actuatorGenForcesStore!=NULL) {
		delete _actuatorGenForcesStore;  _actuatorGenForcesStore=NULL;
	}

}
//_____________________________________________________________________________
/**
 * Construct an GeneralizedForces object for recording the joint torques due
 * an actuator or set of actuators during a simulation.
 *
 * @param aModel Model for which the joint torques are to be recorded.
 * @param aNA Number of actuators in set 
 * @param aActuatorList An array containing the actuator numbers
 */
ActuatorGeneralizedForces::ActuatorGeneralizedForces(Model *aModel) : 
Analysis(aModel),
_actuatorNames(_propActuatorNames.getValueStrArray()),
_actuatorList(0)
{
	setNull();

	/* The rest will be done when a model and actuator set are set
	// ALLOCATE STATE VECTOR
	_dqdt = new double[_model->getNQ()];
	_dudt = new double[_model->getNU()];
	_actuatorList = aActuatorList;
	_actuatorGenForces = new double[_model->getNU()];

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
	*/
}
ActuatorGeneralizedForces::
ActuatorGeneralizedForces(const std::string &aFileName) :
Analysis(aFileName),
_actuatorNames(_propActuatorNames.getValueStrArray()),
_actuatorList(0)
{
	setNull();
	// Serialize from XML
	updateFromXMLNode();

	setName("ActuatorGeneralizedForces");
}

ActuatorGeneralizedForces::
ActuatorGeneralizedForces(DOMElement *aElement) :
Analysis(aElement),
_actuatorNames(_propActuatorNames.getValueStrArray()),
_actuatorList(0)
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
ActuatorGeneralizedForces::
ActuatorGeneralizedForces(const ActuatorGeneralizedForces &aObject) :
Analysis(aObject),
_actuatorNames(_propActuatorNames.getValueStrArray()),
_actuatorList(0)
{
	setNull();

	// COPY TYPE AND NAME
	*this = aObject;
}


Object* ActuatorGeneralizedForces::
copy() const
{

	ActuatorGeneralizedForces *object = new ActuatorGeneralizedForces(*this);
	return(object);
}

Object* ActuatorGeneralizedForces::
copy(DOMElement *aElement) const
{
	ActuatorGeneralizedForces *object = new ActuatorGeneralizedForces(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------

ActuatorGeneralizedForces& 
ActuatorGeneralizedForces::operator=(const ActuatorGeneralizedForces &aActuatorGeneralizedForces)
{
	// BASE CLASS
	Analysis::operator=(aActuatorGeneralizedForces);
	_actuatorNames = aActuatorGeneralizedForces._actuatorNames;
	_actuatorList = aActuatorGeneralizedForces._actuatorList;
	return(*this);
}
//_____________________________________________________________________________
/**
 * SetNull().
 */
void ActuatorGeneralizedForces::
setNull()
{

	// POINTERS
	_dqdt = NULL;
	_dudt = NULL;
	_actuatorList = NULL;
	_actuatorGenForces = NULL;
	_actuatorGenForcesStore = NULL;

	// OTHER VARIABLES

	setName("ActuatorGeneralizedForces");

	setType("ActuatorGeneralizedForces");
	Array<std::string> none("", 1);	
	_propActuatorNames.setName("actuators_list");
	_propActuatorNames.setValue(none);
	_propertySet.append(&_propActuatorNames);

}
//_____________________________________________________________________________
/**
 * Analysis::setModel override
 */
void ActuatorGeneralizedForces::setModel(Model *aModel)
{
	Analysis::setModel(aModel);
	if (_dqdt != 0) delete[] _dqdt;	_dqdt = new double[_model->getNQ()];
	if (_dudt != 0) delete[] _dudt;	_dudt = new double[_model->getNU()];
	if (_actuatorGenForces != 0) delete[] _actuatorGenForces;	_actuatorGenForces = new double[_model->getNU()];

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();

	// Map names to indices
	setActuatorList(_actuatorNames);

}
//_____________________________________________________________________________
/**
 * Select Actuators to be used, passed in by names as read from xml file.
 * setModel() must be called first since the model performs the mapping and validation
 * of names.
 */
void ActuatorGeneralizedForces::
setActuatorList(const Array<std::string>& aActuatorNames)
{
	_actuatorList.setSize(0);
	for(int i=0; i < aActuatorNames.getSize(); i++){
		int actIndex = _model->getActuatorIndex(aActuatorNames.get(i));
		if (actIndex != -1)
			_actuatorList.append(actIndex);

	}
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void ActuatorGeneralizedForces::
allocateStorage()
{
	// ACTUATOR GENERALIZED FORCES
	_actuatorGenForcesStore = new Storage(1000,"Actuator Generalized Forces");
	_actuatorGenForcesStore->setDescription(getDescription());
	_actuatorGenForcesStore->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________

//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the joint torque files.
 */
void ActuatorGeneralizedForces::
constructDescription()
{
	char descrip[1024];

	strcpy(descrip,"\nUnits are S.I. units (second, meters, Newtons, ...)");
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
 * Construct the column labels for the kinematics files.
 */
void ActuatorGeneralizedForces::
constructColumnLabels()
{
	// CHECK FOR NULL
	if(_model->getSpeedName(0).empty()) { setColumnLabels(NULL);  return; }

	// ASSIGN
	int i;
	string labels = "time";
	for(i=0;i<_model->getNU();i++) {
		labels += "\t";
		labels += _model->getSpeedName(i);
	}
	labels += "\n";

	setColumnLabels(labels.c_str());
}


//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the storage for the actuator generalized forces.
 *
 * @return Actuator Generalized Force storage.
 */
Storage* ActuatorGeneralizedForces::
getActuatorGenForcesStorage()
{
	return(_actuatorGenForcesStore);
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
void ActuatorGeneralizedForces::
setStorageCapacityIncrements(int aIncrement)
{
	_actuatorGenForcesStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the joint torques.
 */
int ActuatorGeneralizedForces::
record(double aT,double *aX,double *aY)
{
	// NUMBERS
	int nq = _model->getNQ();
	int nu = _model->getNU();
	int ny = _model->getNY();

	// COMPUTE ACCELERATIONS OF GENERALIZED COORDINATES
	_model->set(aT,aX,aY);
	_model->computeActuation();
	_model->applyActuatorForces();
	_model->computeContact();
	_model->applyContactForces();
	_model->computeAccelerations(_dqdt,_dudt);


	// COMPUTE ACTUATOR GENERALIZED FORCES - to use the compute generalized
	// forces function you actually apply everything BUT the force that you care about
	int i;
	int size = _actuatorList.getSize();
	_model->setStates(aY); //clears forces
	_model->computeActuation();
	for(i=0;i<size;i++) {
		_model->setActuatorForce(_actuatorList[i],0.0);
	}
	_model->applyActuatorForces();
	_model->computeContact();
	_model->applyContactForces();
	_model->computeGeneralizedForces(_dudt,_actuatorGenForces);

	// RECORD RESULTS
	_actuatorGenForcesStore->append(aT*_model->getTimeNormConstant(),nu,_actuatorGenForces);

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
int ActuatorGeneralizedForces::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	int status = record(aT,aX,aY);

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
int ActuatorGeneralizedForces::
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
int ActuatorGeneralizedForces::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);
	cout<<"ActuatorGeneralizedForces.end: Finalizing analysis "<<
		getName()<<".\n";
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
int ActuatorGeneralizedForces::
printResults(const char *aBaseName,const char *aDir,double aDT,
				 const char *aExtension)
{
	if(aBaseName==NULL) return(-1);

	// CONSTRUCT PATH
	char path[2048],name[2048];
	if(aDir==NULL) {
		strcpy(path,".");
	} else {
		strcpy(path,aDir);
	}

	// ACTUATOR GENERALIZED FORCES

	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_actuatorGenForcesStore!=NULL)  _actuatorGenForcesStore->print(name);
	} else {
		if(_actuatorGenForcesStore!=NULL)  _actuatorGenForcesStore->print(name,aDT);
	}
	return(0);
}


