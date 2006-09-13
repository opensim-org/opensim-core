// Kinematics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "Kinematics.h"



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
Kinematics::~Kinematics()
{
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an Kinematics object for recording the kinematics of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the kinematics are to be recorded.
 */
Kinematics::Kinematics(Model *aModel) :
	Analysis(aModel)
{
	setNull();

	if (_model != 0){
		// ALLOCATE STATE VECTOR
		_y = new double[_model->getNY()];
		_dy = new double[_model->getNY()];
	}
	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}
//=============================================================================
// Object Overrides
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
Kinematics::Kinematics(const std::string &aFileName):
Analysis(aFileName)
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}
//_____________________________________________________________________________
/**
 * Construct an object from an DOMElement.
 */
Kinematics::Kinematics(DOMElement *aElement):
Analysis(aElement)
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
Kinematics::Kinematics(const Kinematics &aKinematics):
Analysis(aKinematics)
{
	setNull();
	// COPY TYPE AND NAME
	*this = aKinematics;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* Kinematics::copy() const
{
	Kinematics *object = new Kinematics(*this);
	return(object);

}
//_____________________________________________________________________________
/**
 * Instantiate from DOMElement
 *
 */
Object* Kinematics::copy(DOMElement *aElement) const
{
	Kinematics *object = new Kinematics(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Kinematics::
setNull()
{
	setType("Kinematics");

	setName("Kinematics");

	_y=0;
	_dy=0;

	_pStore=_vStore=_aStore=0;

}
//--------------------------------------------------------------------------
// OPERATORS
//--------------------------------------------------------------------------
Kinematics& Kinematics::operator=(const Kinematics &aKinematics)
{
	// BASE CLASS
	Analysis::operator=(aKinematics);

	// Deallocate _y & _dy if already allocated
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }

	if(_model) {
		_y = new double[_model->getNY()];
		_dy = new double[_model->getNY()];
		constructColumnLabels();
	}
	deleteStorage();
	allocateStorage();
	return (*this);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void Kinematics::
allocateStorage()
{
	// ACCELERATIONS
	_aStore = new Storage(1000,"Accelerations");
	_aStore->setDescription(getDescription());
	_aStore->setColumnLabels(getColumnLabels());
	_storageList.append(_aStore);

	// VELOCITIES
	_vStore = new Storage(1000,"Speeds");
	_vStore->setDescription(getDescription());
	_vStore->setColumnLabels(getColumnLabels());
	_storageList.append(_vStore);

	// POSITIONS
	_pStore = new Storage(1000,"Coordinates");
	_pStore->setDescription(getDescription());
	_pStore->setColumnLabels(getColumnLabels());
	_storageList.append(_pStore);
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void Kinematics::
deleteStorage()
{
	if(_aStore!=NULL) { delete _aStore;  _aStore=NULL; }
	if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
	if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Construct the description for the kinematics files.
 */
void Kinematics::
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
void Kinematics::
constructColumnLabels()
{
	// CHECK FOR NULL
	if (_model ==0) return;

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
	_pStore->setColumnLabels(getColumnLabels());
	_vStore->setColumnLabels(getColumnLabels());
	_aStore->setColumnLabels(getColumnLabels());
}


//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the acceleration storage.
 *
 * @return Acceleration storage.
 */
Storage* Kinematics::
getAccelerationStorage()
{
	return(_aStore);
}
//_____________________________________________________________________________
/**
 * Get the velocity storage.
 *
 * @return Velocity storage.
 */
Storage* Kinematics::
getVelocityStorage()
{
	return(_vStore);
}
//_____________________________________________________________________________
/**
 * Get the position storage.
 *
 * @return Position storage.
 */
Storage* Kinematics::
getPositionStorage()
{
	return(_pStore);
}
//_____________________________________________________________________________
/**
 * Set the model pointer for analyzing kinematics.
 */
void Kinematics::setModel(Model *aModel)
{
	// BASE CLASS
	Analysis::setModel(aModel);

	// DATA MEMBERS
	if(_y!=NULL) { delete[] _y;  _y=NULL; }
	if(_dy!=NULL) { delete[] _dy;  _dy=NULL; }

	if (_model){
		// ALLOCATE STATE VECTOR
		_y = new double[_model->getNY()];
		_dy = new double[_model->getNY()];
	}

	constructColumnLabels();
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
void Kinematics::
setStorageCapacityIncrements(int aIncrement)
{
	_aStore->setCapacityIncrement(aIncrement);
	_vStore->setCapacityIncrement(aIncrement);
	_pStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the kinematics.
 *
 * @return 0 of success, -1 on error.
 */
int Kinematics::
record(double aT,double *aX,double *aY)
{
	// NUMBERS
	int nq = _model->getNQ();
	int nu = _model->getNU();
	int ny = _model->getNY();

	// COMPUTE DERIVATIVES
	// ----------------------------------
	// SET
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// ACTUATION
	_model->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->applyActuatorForces();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// CONTACT
	_model->computeContact();
	_model->getDerivCallbackSet()->computeContact(aT,aX,aY);
	_model->applyContactForces();
	_model->getDerivCallbackSet()->applyContact(aT,aX,aY);

	// ACCELERATIONS
	_model->computeAccelerations(_dy,&_dy[nq]);
	// ----------------------------------


	// CONVERT RESULTS TO ANGLES
	memcpy(_y,aY,ny*sizeof(double));
	_model->convertQuaternionsToAngles(_y,_y);

	// CONVERT TO DEGREES
	if(getInDegrees()) {
		_model->convertRadiansToDegrees(_y,_y);
		_model->convertRadiansToDegrees(&_y[nq],&_y[nq]);
		_model->convertRadiansToDegrees(&_dy[nq],&_dy[nq]);
	}

	// RECORD RESULTS
	_pStore->append(aT,nu,_y);
	_vStore->append(aT,nu,&_y[nq]);
	_aStore->append(aT,nu,&_dy[nq]);

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
int Kinematics::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_pStore->reset(aT);
	_vStore->reset(aT);
	_aStore->reset(aT);

	// RECORD
	int status = 0;
	if(_pStore->getSize()<=0) {
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
int Kinematics::
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
int Kinematics::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
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
int Kinematics::
printResults(const char *aBaseName,const char *aDir,double aDT,
				 const char *aExtension)
{
	if(!getOn()) {
		printf("Kinematics.printResults: Off- not printing.\n");
		return(0);
	}
	if(aBaseName==NULL) return(-1);

	// CONSTRUCT PATH
	char path[2048],name[2048];
	if(aDir==NULL) {
		strcpy(path,".");
	} else {
		strcpy(path,aDir);
	}

	// ACCELERATIONS
	_aStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_dudt",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s_dudt%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_aStore!=NULL)  _aStore->print(name);
	} else {
		if(_aStore!=NULL)  _aStore->print(name,aDT);
	}

	// VELOCITIES
	_vStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_u",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s_u%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_vStore!=NULL)  _vStore->print(name);
	} else {
		if(_vStore!=NULL)  _vStore->print(name,aDT);
	}

	// POSITIONS
	_pStore->scaleTime(_model->getTimeNormConstant());
	if(aExtension==NULL) {
		sprintf(name,"%s/%s_%s_q",path,aBaseName,getName().c_str());
	} else {
		sprintf(name,"%s/%s_%s_q%s",path,aBaseName,getName().c_str(),aExtension);
	}
	if(aDT<=0.0) {
		if(_pStore!=NULL)  _pStore->print(name);
	} else {
		if(_pStore!=NULL)  _pStore->print(name,aDT);
	}

	return(0);
}


ArrayPtrs<Storage>& Kinematics::getStorageList()
{
	return _storageList;
}

