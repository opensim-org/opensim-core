// GeneralizedForces.cpp
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
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include "GeneralizedForces.h"

//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace std;
using namespace OpenSim;
/**
 * Destructor.
 */
GeneralizedForces::~GeneralizedForces()
{
	if(_dqdt!=NULL) { delete[] _dqdt;  _dqdt=NULL; }
	if(_dudt!=NULL) { delete[] _dudt;  _dudt=NULL; }
	if(_zero_aY!=NULL) { delete[] _zero_aY;  _zero_aY=NULL; }
	if(_gravGenForces!=NULL) { delete[] _gravGenForces;  _gravGenForces=NULL; }
	if(_velGenForces!=NULL) { delete[] _velGenForces;  _velGenForces=NULL; }
	if(_actuatorGenForces!=NULL) { delete[] _actuatorGenForces;  _actuatorGenForces=NULL; }
	if(_contactGenForces!=NULL) { delete[] _contactGenForces;  _contactGenForces=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an GeneralizedForces object for recording the joint torques of
 * a model's generalized coodinates during a simulation.
 *
 * @param aModel Model for which the joint torques are to be recorded.
 */
GeneralizedForces::GeneralizedForces(Model *aModel) :
	Analysis(aModel)
{
	setName("GeneralizedForces");

	if (aModel==0)
		return;

	// ALLOCATE STATE VECTOR
	_dqdt = new double[_model->getNumCoordinates()];
	_dudt = new double[_model->getNumSpeeds()];
	_zero_aY = new double[_model->getNumStates()];
	_gravGenForces = new double[_model->getNumSpeeds()];
	_velGenForces = new double[_model->getNumSpeeds()];
	_actuatorGenForces = new double[_model->getNumSpeeds()];
	_contactGenForces = new double[_model->getNumSpeeds()];

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
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
GeneralizedForces::
GeneralizedForces(const std::string &aFileName):
	Analysis(aFileName, false)
{
	//setNull();

	// Serialize from XML
	updateFromXMLNode();

	/* The rest will be done by setModel().
	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
	*/
}

// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
GeneralizedForces::GeneralizedForces(const GeneralizedForces &aGeneralizedForces):
Analysis(aGeneralizedForces)
{
	//setNull();
	// COPY TYPE AND NAME
	*this = aGeneralizedForces;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* GeneralizedForces::copy() const
{
	GeneralizedForces *object = new GeneralizedForces(*this);
	return(object);

}

//_____________________________________________________________________________
/**
 * Set the model for which the GeneralizedForces analysis is to be computed.
 *
 * @param aModel Model pointer
 */
void GeneralizedForces::
setModel(Model *aModel)
{
	Analysis::setModel(aModel);

	assert(_model);
	// ALLOCATE STATE VECTOR
	if (_dqdt!= 0) delete[] _dqdt;			_dqdt = new double[_model->getNumCoordinates()];
	if (_dudt!= 0) delete[] _dudt;			_dudt = new double[_model->getNumSpeeds()];
	if (_zero_aY!= 0) delete[] _zero_aY;	_zero_aY = new double[_model->getNumStates()];
	if (_gravGenForces!= 0) delete[] _gravGenForces;			_gravGenForces = new double[_model->getNumSpeeds()];
	if (_velGenForces!= 0) delete[] _velGenForces;			_velGenForces = new double[_model->getNumSpeeds()];
	if (_actuatorGenForces!= 0) delete[] _actuatorGenForces;			_actuatorGenForces = new double[_model->getNumSpeeds()];
	if (_contactGenForces!= 0) delete[] _contactGenForces;			_contactGenForces = new double[_model->getNumSpeeds()];

	// CONSTRUCT DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	// STORAGE
	allocateStorage();
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void GeneralizedForces::
allocateStorage()
{
	// GRAVITY GENERALIZED FORCES
	_gravGenForcesStore = new Storage(1000,"Gravity Generalized Forces");
	_gravGenForcesStore->setDescription(getDescription());
	_gravGenForcesStore->setColumnLabels(getColumnLabels());

	// VELOCITY GENERALIZED FORCES
	_velGenForcesStore = new Storage(1000,"Velocity Generalized Forces");
	_velGenForcesStore->setDescription(getDescription());
	_velGenForcesStore->setColumnLabels(getColumnLabels());

	// ACTUATOR GENERALIZED FORCES
	_actuatorGenForcesStore = new Storage(1000,"Actuator Generalized Forces");
	_actuatorGenForcesStore->setDescription(getDescription());
	_actuatorGenForcesStore->setColumnLabels(getColumnLabels());

	// CONTACT GENERALIZED FORCES
	_contactGenForcesStore = new Storage(1000,"Contact Generalized Forces");
	_contactGenForcesStore->setDescription(getDescription());
	_contactGenForcesStore->setColumnLabels(getColumnLabels());

}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void GeneralizedForces::
deleteStorage()
{
	if(_gravGenForcesStore!=NULL) { delete _gravGenForcesStore;  _gravGenForcesStore=NULL; }
	if(_velGenForcesStore!=NULL) { delete _velGenForcesStore;  _velGenForcesStore=NULL; }
	if(_actuatorGenForcesStore!=NULL) { delete _actuatorGenForcesStore;  _actuatorGenForcesStore=NULL; }
	if(_contactGenForcesStore!=NULL) { delete _contactGenForcesStore;  _contactGenForcesStore=NULL; }
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
GeneralizedForces& GeneralizedForces::
operator=(const GeneralizedForces &aGeneralizedForces)
{
	// BASE CLASS
	Analysis::operator=(aGeneralizedForces);
	return(*this);
}

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
void GeneralizedForces::
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
void GeneralizedForces::
constructColumnLabels()
{
	// CHECK FOR NULL
	if (!_model || _model->getDynamicsEngine().getNumSpeeds() == 0)
	{
		setColumnLabels(Array<string>());
		return;
	}

	Array<string> labels;
	labels.append("time");
	SpeedSet *ss = _model->getDynamicsEngine().getSpeedSet();
	for(int i=0; i<ss->getSize(); i++) labels.append(ss->get(i)->getName());
	setColumnLabels(labels);
}


//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the storage for the gravity generalized forces.
 *
 * @return Gravity Generalized Force storage.
 */
Storage* GeneralizedForces::
getGravGenForcesStorage()
{
	return(_gravGenForcesStore);
}

/**
 * Get the storage for the velocity generalized forces.
 *
 * @return Velocity Generalized Force storage.
 */
Storage* GeneralizedForces::
getVelGenForcesStorage()
{
	return(_velGenForcesStore);
}

/**
 * Get the storage for the actuator generalized forces.
 *
 * @return Actuator Generalized Force storage.
 */
Storage* GeneralizedForces::
getActuatorGenForcesStorage()
{
	return(_actuatorGenForcesStore);
}

/**
 * Get the storage for the contact generalized forces.
 *
 * @return Contact Generalized Force storage.
 */
Storage* GeneralizedForces::
getContactGenForcesStorage()
{
	return(_contactGenForcesStore);
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
void GeneralizedForces::
setStorageCapacityIncrements(int aIncrement)
{
	_gravGenForcesStore->setCapacityIncrement(aIncrement);
	_velGenForcesStore->setCapacityIncrement(aIncrement);
	_actuatorGenForcesStore->setCapacityIncrement(aIncrement);
	_contactGenForcesStore->setCapacityIncrement(aIncrement);
}



//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the joint torques.
 */
int GeneralizedForces::
record(double aT,double *aX,double *aY)
{
	// NUMBERS
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int ny = _model->getNumStates();

	double zeroGrav[3] = {0.0, 0.0, 0.0};
	double grav[3];

	// COMPUTE ACCELERATIONS OF GENERALIZED COORDINATES
	// ----------------------------------
	// SET
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// ACTUATION
	_model->getActuatorSet()->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// CONTACT
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(aT,aX,aY);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(aT,aX,aY);

	// ACCELERATIONS
	_model->getDynamicsEngine().computeDerivatives(_dqdt,_dudt);
	// ----------------------------------

	// INITIALIZE ZERO VECTOR
	for(int y=0; y<ny; y++){
		_zero_aY[y] = aY[y];	
	}	
	for(int u=0;u<nu;u++){
		_zero_aY[nq+u] = 0.0;
	}	

	// COMPUTE GENERALIZED FORCES DUE TO GRAVITY                                 
	_model->getGravity(grav);
	_model->setGravity(zeroGrav);
	_model->setStates(aY);
	_model->getActuatorSet()->computeActuation();
	_model->getActuatorSet()->apply(); 
	_model->getContactSet()->computeContact();
	_model->getContactSet()->apply();
	_model->getDynamicsEngine().computeGeneralizedForces(_dudt, _gravGenForces);
	_model->setGravity(grav);
		
	//COMPUTE GENERALIZED FORCES DUE TO VELOCITY EFFECTS
	_model->setStates(_zero_aY);
	_model->getActuatorSet()->computeActuation();
	_model->getActuatorSet()->apply(); 
	_model->getContactSet()->computeContact();
	_model->getContactSet()->apply();
	_model->getDynamicsEngine().computeGeneralizedForces(_dudt, _velGenForces);

	// COMPUTE ACTUATOR GENERALIZED FORCES
	_model->setStates(aY);
	_model->getContactSet()->computeContact();
	_model->getContactSet()->apply();
	_model->getDynamicsEngine().computeGeneralizedForces(_dudt, _actuatorGenForces);

	// COMPUTE CONTACT GENERALIZED FORCES
	_model->setStates(aY);
	_model->getActuatorSet()->computeActuation();
	_model->getActuatorSet()->apply(); 
	_model->getDynamicsEngine().computeGeneralizedForces(_dudt, _contactGenForces);

	// RECORD RESULTS
	_gravGenForcesStore->append(aT*_model->getTimeNormConstant(),nu,_gravGenForces);
	_velGenForcesStore->append(aT*_model->getTimeNormConstant(),nu,_velGenForces);
	_actuatorGenForcesStore->append(aT*_model->getTimeNormConstant(),nu,_actuatorGenForces);
	_contactGenForcesStore->append(aT*_model->getTimeNormConstant(),nu,_contactGenForces);

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
int GeneralizedForces::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
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
int GeneralizedForces::
step(double *aXPrev,double *aYPrev,double *aYPPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
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
int GeneralizedForces::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);
	record(aT,aX,aY);
	printf("rdJointTorques.end: Finalizing analysis %s.\n",getName().c_str());
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
int GeneralizedForces::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// GRAVITY GENERALIZED FORCES
	Storage::printResult(_gravGenForcesStore,aBaseName+"_Grav"+getName(),aDir,aDT,aExtension);

	// VELOCITY GENERALIZED FORCES
	Storage::printResult(_velGenForcesStore,aBaseName+"_Vel"+getName(),aDir,aDT,aExtension);

	// ACTUATOR GENERALIZED FORCES
	Storage::printResult(_actuatorGenForcesStore,aBaseName+"_Actuator"+getName(),aDir,aDT,aExtension);

	// CONTACT GENERALIZED FORCES
	Storage::printResult(_contactGenForcesStore,aBaseName+"_Contact"+getName(),aDir,aDT,aExtension);

	return(0);
}


