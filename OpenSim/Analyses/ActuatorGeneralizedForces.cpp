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
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/AbstractActuator.h>
#include <OpenSim/Simulation/SIMM/SpeedSet.h>
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
 * @param aModel AbstractModel for which the joint torques are to be recorded.
 * @param aNA Number of actuators in set 
 * @param aActuatorList An array containing the actuator numbers
 */
ActuatorGeneralizedForces::ActuatorGeneralizedForces(AbstractModel *aModel) : 
Analysis(aModel),
_actuatorNames(_propActuatorNames.getValueStrArray()),
_actuatorList(NULL)
{
	setNull();

	/* The rest will be done when a model and actuator set are set
	// ALLOCATE STATE VECTOR
	_dqdt = new double[_model->getNumCoordinates()];
	_dudt = new double[_model->getNumSpeeds()];
	_actuatorList = aActuatorList;
	_actuatorGenForces = new double[_model->getNumSpeeds()];

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
_actuatorList(NULL)
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
_actuatorList(NULL)
{
	setNull();

	// Serialize from XML
	updateFromXMLNode();

}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
ActuatorGeneralizedForces::
ActuatorGeneralizedForces(const ActuatorGeneralizedForces &aObject) :
Analysis(aObject),
_actuatorNames(_propActuatorNames.getValueStrArray()),
_actuatorList(NULL)
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
	_actuatorList = Array<AbstractActuator*>(NULL);  // CLAY- Is this right?  Or, should it simply be...
	//_actuatorList.setSize(0);
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
void ActuatorGeneralizedForces::setModel(AbstractModel *aModel)
{
	Analysis::setModel(aModel);
	if (_dqdt != 0) delete[] _dqdt;	_dqdt = new double[_model->getNumCoordinates()];
	if (_dudt != 0) delete[] _dudt;	_dudt = new double[_model->getNumSpeeds()];
	if (_actuatorGenForces != 0) delete[] _actuatorGenForces;	_actuatorGenForces = new double[_model->getNumSpeeds()];

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
		AbstractActuator *act = _model->getActuatorSet()->get(aActuatorNames.get(i));
		if (act)
			_actuatorList.append(act);

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
	if (!_model || _model->getDynamicsEngine().getNumSpeeds() == 0)
	{
		setColumnLabels(NULL);
		return;
	}

	string labels = "time";
	SpeedSet *ss = _model->getDynamicsEngine().getSpeedSet();

	for(int i=0; i<ss->getSize(); i++)
	{
		AbstractSpeed* speed = ss->get(i);
		labels += "\t" + speed->getName();
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
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int ny = _model->getNumStates();

	// COMPUTE ACCELERATIONS OF GENERALIZED COORDINATES
	_model->set(aT,aX,aY);
	_model->getActuatorSet()->computeActuation();
	_model->getActuatorSet()->apply();
	_model->getContactSet()->computeContact();
	_model->getContactSet()->apply();
	_model->getDynamicsEngine().computeDerivatives(_dqdt,_dudt);


	// COMPUTE ACTUATOR GENERALIZED FORCES - to use the compute generalized
	// forces function you actually apply everything BUT the force that you care about
	int i;
	int size = _actuatorList.getSize();
	_model->setStates(aY); //clears forces
	_model->getActuatorSet()->computeActuation();
	for(i=0;i<size;i++) {
		_actuatorList[i]->setForce(0.0);
	}
	_model->getActuatorSet()->apply();
	_model->getContactSet()->computeContact();
	_model->getContactSet()->apply();
	_model->getDynamicsEngine().computeGeneralizedForces(_dudt,_actuatorGenForces);

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
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	Storage::printResult(_actuatorGenForcesStore,aBaseName+"_"+getName(),aDir,aDT,aExtension);
	return(0);
}


