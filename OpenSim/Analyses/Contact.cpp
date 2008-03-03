// Contact.cpp
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "Contact.h"



using namespace OpenSim;
using namespace std;
using SimTK::Vec3;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Contact::~Contact()
{
	if(_points!=NULL) { delete[] _points;  _points=NULL; }
	if(_velocities!=NULL) { delete[] _velocities;  _velocities=NULL; }
	if(_forces!=NULL) { delete[] _forces;  _forces=NULL; }
	if(_powers!=NULL) { delete[] _powers;  _powers=NULL; }
	if(_resultantForcePoints!=NULL) {
		delete[] _resultantForcePoints;
		_resultantForcePoints=NULL;
	}
	if(_resultantForcePointGroupAssignements!=NULL) {
		delete[] _resultantForcePointGroupAssignements;
		_resultantForcePointGroupAssignements = NULL;
	}
	if(_totContactForces!=NULL) {delete[] _totContactForces;	_totContactForces=NULL; }
	if(_totContactTorques!=NULL) {delete[] _totContactTorques; _totContactTorques=NULL; }
	deleteStorage();
}
//_____________________________________________________________________________
/**
 * Construct an Contact object for recording the various qunatities
 * associates with contact.
 *
 * @param aModel Model for which the kinematics are to be recorded.
 * @param aResultantForcePointGroups Array containing group assignment for each contact point.
 *  The array must be as long as the number of contact points in the model.
 *  For a contact point to not be included in a resultant force point (RFP) calculation,
 *  assign it a negative group number.  IF all points are assigned negative numbers, no RFP
 *  analysis will be performed.
 *  If no array is given, the RFP is calcualted for all contact points in the
 *  model.
 */
Contact::Contact(Model *aModel,int *aResultantForcePointGroups) :
	Analysis(aModel)
{
	// NULL
	setNull();

	// NAME
	setName("Contact");

	// SET RESULTANT FORCE POINT VARIABLES
	int p, counter;
	_resultantForcePointGroupAssignements = aResultantForcePointGroups;
	if(_resultantForcePointGroupAssignements!=NULL){
		counter = -1;
		for(p=0;p<_model->getNumContacts();p++){
			if(counter < _resultantForcePointGroupAssignements[p]){
				counter = _resultantForcePointGroupAssignements[p];
			}
		}
		_nResultantForcePointGroups = counter + 1;
	} else {
		_nResultantForcePointGroups = 1;
		_resultantForcePointGroupAssignements = new int[_model->getNumContacts()];
		for(p=0;p<_model->getNumContacts();p++)
			_resultantForcePointGroupAssignements[p]=0;
	}

	// ALLOCATE WORK ARRAYS
	int n = 6*_model->getNumContacts();
	_points = new double[n];
	_velocities = new double[n];
	_forces = new double[n];
	_powers = new double[_model->getNumContacts()];
	_resultantForcePoints = new double[3*_nResultantForcePointGroups];
	_totContactForces = new double[3*_nResultantForcePointGroups];
	_totContactTorques = new double[3*_nResultantForcePointGroups];

	// DESCRIPTIONS
	constructDescription();
	constructColumnLabels(_nResultantForcePointGroups);

	// STORAGE
	allocateStorage();
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for member variables.
 */
void Contact::
setNull()
{
	_points = NULL;
	_velocities = NULL;
	_forces = NULL;
	_powers = NULL;
	_resultantForcePoints = NULL;
	_totContactForces = NULL;
	_totContactTorques = NULL;
	_pStore = NULL;
	_vStore = NULL;
	_fStore = NULL;
	_pwrStore = NULL;
	_resultantForcePointsStore = NULL;
	_totFStore = NULL;
	_totTorqueStore = NULL;
	_nResultantForcePointGroups = 0;
}
//_____________________________________________________________________________
/**
 * Construct a description.
 */
void Contact::
constructDescription()
{
	string descrip = "";

	// DESCRIPTION
	descrip = "\nThis file contains the contact quantities generated ";
	descrip += "during a simulation.\n";
	descrip += "All quantities are expressed in the global frame.\n";
	descrip += "The units are S.I. (seconds, Newtons, meters, etc.).";

	// BODY ID'S
	int id = 0;
	char tmp[Object::NAME_LENGTH];
	descrip += "\n\nBody ID's:\n";
	int i = 0;
	BodySet *bs = _model->getDynamicsEngine().getBodySet();

	for(i=0; i<bs->getSize(); i++)
	{
		AbstractBody *body=bs->get(i);
		//if(_model->getGroundID()==0) id = i+1; TODOAUG: what is this for?
		//else id = i;
		sprintf(tmp,"%4d\t%s\n",id,body->getName().c_str());
		descrip += tmp;
		id++;
	}
	descrip += "\n";

	setDescription(descrip);

}
//_____________________________________________________________________________
/**
 * Get a string of column labels.
 */
void Contact::
constructColumnLabels(int nResultantForcePointgroups)
{
	Array<string> labels;
	labels.append("time");

	// VECTOR LABELS
	int p;
	AbstractBody *a, *b;
	char tmp[Object::NAME_LENGTH];
	int np = _model->getNumContacts();
	for(p=0;p<np;p++) {

		// BODY A
		a = _model->getContactSet()->getContactBodyA(p);
		sprintf(tmp,"%d_A_%s_x",p,a->getName().c_str());
		labels.append(tmp);
		sprintf(tmp,"%d_A_%s_y",p,a->getName().c_str());
		labels.append(tmp);
		sprintf(tmp,"%d_A_%s_z",p,a->getName().c_str());
		labels.append(tmp);

		// BODY B
		b = _model->getContactSet()->getContactBodyB(p);
		sprintf(tmp,"%d_B_%s_x",p,b->getName().c_str());
		labels.append(tmp);
		sprintf(tmp,"%d_B_%s_y",p,b->getName().c_str());
		labels.append(tmp);
		sprintf(tmp,"%d_B_%s_z",p,b->getName().c_str());
		labels.append(tmp);
	}
	setColumnLabels(labels);

	// SCALAR LABELS
	_scalarLabels.setSize(0);
	_scalarLabels.append("time");
	for(p=0;p<np;p++) {
		a = _model->getContactSet()->getContactBodyA(p);
		b = _model->getContactSet()->getContactBodyB(p);
		sprintf(tmp,"%d_%s_%s",p,a->getName().c_str(),b->getName().c_str());
		_scalarLabels.append(tmp);
	}

	// RESULTANT FORCE POINT LABELS
	_resultantForcePointLabels.setSize(0);
	_resultantForcePointLabels.append("time");
	for(p=0;p<nResultantForcePointgroups;p++) {
		sprintf(tmp,"resultantForcePoint_Group_%d_x",p);
		_resultantForcePointLabels.append(tmp);
		sprintf(tmp,"resultantForcePoint_Group_%d_y",p);
		_resultantForcePointLabels.append(tmp);
		sprintf(tmp,"resultantForcePoint_Group_%d_z",p);
		_resultantForcePointLabels.append(tmp);
	}
}
//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void Contact::
allocateStorage()
{
	// POINTS
	_pStore = new Storage(1000,"ContactPoints");
	_pStore->setDescription(getDescription());
	_pStore->setColumnLabels(getColumnLabels());

	// VELOCITIES
	_vStore = new Storage(1000,"ContactVelocities");
	_vStore->setDescription(getDescription());
	_vStore->setColumnLabels(getColumnLabels());

	// FORCES
	_fStore = new Storage(1000,"ContactForces");
	_fStore->setDescription(getDescription());
	_fStore->setColumnLabels(getColumnLabels());

	// POWERS
	_pwrStore = new Storage(1000,"ContactPowers");
	_pwrStore->setDescription(getDescription().c_str());
	_pwrStore->setColumnLabels(getScalarColumnLabels());

	// RESULTANT FORCE POINT
	_resultantForcePointsStore = new Storage(1000,"ContactResultantForcePoint");
	_resultantForcePointsStore->setDescription(getDescription());
	_resultantForcePointsStore->setColumnLabels(getResultantForcePointColumnLabels());

	// TOTAL FORCE
	_totFStore = new Storage(1000,"ContactTotalForces");
	_totFStore->setDescription(getDescription());
	_totFStore->setColumnLabels(getResultantForcePointColumnLabels());
		
	// TOTAL TORQUE
	_totTorqueStore = new Storage(1000,"ContactTotalTorques");
	_totTorqueStore->setDescription(getDescription());
	_totTorqueStore->setColumnLabels(getResultantForcePointColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void Contact::
deleteStorage()
{
	if(_pStore!=NULL) { delete _pStore;  _pStore=NULL; }
	if(_vStore!=NULL) { delete _vStore;  _vStore=NULL; }
	if(_fStore!=NULL) { delete _fStore;  _fStore=NULL; }
	if(_pwrStore!=NULL) { delete _pwrStore;  _pwrStore=NULL; }
	if(_resultantForcePointsStore!=NULL) { delete _resultantForcePointsStore;  _resultantForcePointsStore=NULL; }
	if(_totFStore!=NULL) { delete _totFStore;  _totFStore=NULL; }
	if(_totTorqueStore!=NULL) { delete _totTorqueStore;  _totTorqueStore=NULL; }
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// COLUMN LABELS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the column labels for scalar variables such as contact powers.
 *
 * @return Column labels for scalar variables.
 */
const Array<string> &Contact::
getScalarColumnLabels()
{
	return(_scalarLabels);
}
//_____________________________________________________________________________
/**
 * Get the column labels for resultant force point groups.
 *
 * @return Column labels for resultant force point groups.
 */
const Array<string> &Contact::
getResultantForcePointColumnLabels()
{
	return(_resultantForcePointLabels);
}

//-----------------------------------------------------------------------------
// STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the contact points storage.
 *
 * @return Points storage.
 */
Storage* Contact::
getPointsStorage() const
{
	return(_pStore);
}
//_____________________________________________________________________________
/**
 * Get the contact point velocity storage.
 *
 * @return Body-local force storage.
 */
Storage* Contact::
getVelocityStorage() const
{
	return(_vStore);
}
//_____________________________________________________________________________
/**
 * Get the force storage.
 *
 * @return Force storage.
 */
Storage* Contact::
getForceStorage() const
{
	return(_fStore);
}
//_____________________________________________________________________________
/**
 * Get the power storage.
 *
 * @return Power storage.
 */
Storage* Contact::
getPowerStorage() const
{
	return(_pwrStore);
}
//_____________________________________________________________________________
/**
 * Get the resultant force point storage.
 *
 * @return resultant force point storage.
 */
Storage* Contact::
getResultantForcePointStorage() const
{
	return(_resultantForcePointsStore);
}
//_____________________________________________________________________________
/**
 * Get the total force storage.
 *
 * @return totForce storage.
 */
Storage* Contact::
getTotForceStorage() const
{
	return(_totFStore);
}
//_____________________________________________________________________________
/**
 * Get the total torque storage.
 *
 * @return totTorque storage.
 */
Storage* Contact::
getTotTorqueStorage() const
{
	return(_totTorqueStore);
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
void Contact::
setStorageCapacityIncrements(int aIncrement)
{
	_pStore->setCapacityIncrement(aIncrement);
	_vStore->setCapacityIncrement(aIncrement);
	_fStore->setCapacityIncrement(aIncrement);
	_pwrStore->setCapacityIncrement(aIncrement);
	_resultantForcePointsStore->setCapacityIncrement(aIncrement);
	_totFStore->setCapacityIncrement(aIncrement);
	_totTorqueStore->setCapacityIncrement(aIncrement);
}


//=============================================================================
// ANALYSIS
//=============================================================================
//_____________________________________________________________________________
/**
 * Record the contact quantities.
 */
int Contact::
record(double aT,double *aX,double *aY)
{
	double tReal = aT * _model->getTimeNormConstant();

	// SET THE STATE
	_model->set(aT,aX,aY);

	// COMPUTE CONTACT
	_model->getContactSet()->computeContact();

	// POINTS
	int p,I,j;
	int np = _model->getNumContacts();
	int n = 6*np;
	SimTK::Vec3 vec;
	AbstractBody *a, *b;
	for(p=0;p<np;p++) {

		I = Mtx::ComputeIndex(p,6,0);

		// A
		a = _model->getContactSet()->getContactBodyA(p);
		_model->getContactSet()->getContactPointA(p,vec);
		_model->getDynamicsEngine().getPosition(*a,vec,Vec3::updAs(&_points[I]));

		// B
		b = _model->getContactSet()->getContactBodyB(p);
		_model->getContactSet()->getContactPointB(p,vec);
		_model->getDynamicsEngine().getPosition(*b,vec,Vec3::updAs(&_points[I+3]));
	}
	_pStore->append(tReal,n,_points);

	// VELOCITIES
	for(p=0;p<np;p++) {

		I = Mtx::ComputeIndex(p,6,0);

		// A
		a = _model->getContactSet()->getContactBodyA(p);
		_model->getContactSet()->getContactPointA(p,vec);
		_model->getDynamicsEngine().getVelocity(*a,vec,Vec3::updAs(&_velocities[I]));

		// B
		b = _model->getContactSet()->getContactBodyB(p);
		_model->getContactSet()->getContactPointB(p,vec);
		_model->getDynamicsEngine().getVelocity(*b,vec,Vec3::updAs(&_velocities[I+3]));
	}
	_vStore->append(tReal,n,_velocities);

	// FORCES
	for(p=0;p<np;p++) {

		I = Mtx::ComputeIndex(p,6,0);

		// FORCE ON BodyB EXPRESSED IN GROUND FRAME
		a = _model->getContactSet()->getContactBodyA(p);
		_model->getContactSet()->getContactForce(p,vec);
		_model->getDynamicsEngine().transform(*a,vec,_model->getDynamicsEngine().getGroundBody(),vec);
		Vec3::updAs(&_forces[I+3]) = vec;
		// FORCE ON BodyA IN GROUND FRAME
		Vec3::updAs(&_forces[I]) = -vec;
	}
	_fStore->append(tReal,n,_forces);

	// POWERS
	for(p=0;p<np;p++) {
		_powers[p] = _model->getContactSet()->getContactPower(p);
	}
	_pwrStore->append(tReal,_model->getNumContacts(),_powers);

	//RESULTANT FORCE POINT
	int g, index;
	AbstractBody *bodyA, *bodyB;
	double resultantForcePoint[3];
	SimTK::Vec3 totContactForce;
	SimTK::Vec3 totContactTorque;
	SimTK::Vec3 contactForce;
	SimTK::Vec3 contactPosRelCOMLocal;
	SimTK::Vec3 contactPosRelCOMGlobal;
	//double contactPosGlobal[3];
	//double posBodyCOMLocal[3] = {0,0,0};
	//double posBodyCOMGlobal[3];
	SimTK::Vec3 contactTorque;

	for(g=0;g<_nResultantForcePointGroups;g++){
		for(j=0;j<3;j++){
			 totContactForce[j]=0.0;
			 totContactTorque[j]=0.0;
		}
		for(p=0;p<np;p++){
			if(_resultantForcePointGroupAssignements[p]==g){
				bodyA = _model->getContactSet()->getContactBodyA(p);
				bodyB = _model->getContactSet()->getContactBodyB(p);
				_model->getContactSet()->getContactForce(p,contactForce);
				_model->getDynamicsEngine().transform(*bodyA,contactForce,_model->getDynamicsEngine().getGroundBody(),contactForce);
				totContactForce+=contactForce;
				_model->getContactSet()->getContactPointB(p, contactPosRelCOMLocal);

				_model->getDynamicsEngine().transformPosition(*bodyB,contactPosRelCOMLocal,contactPosRelCOMGlobal);

//				_model->getPosition(bodyB,contactPosRelCOMLocal,contactPosGlobal);
//				_model->getPosition(bodyB,posBodyCOMLocal,posBodyCOMGlobal);
//				Mtx::Subtract(1,3,contactPosGlobal,posBodyCOMGlobal,contactPosRelCOMGlobal);

				Mtx::CrossProduct(contactPosRelCOMGlobal,contactForce,contactTorque);
				totContactTorque +=contactTorque;
			}
		}
		if(fabs(totContactForce[1])>rdMath::ZERO) {
			resultantForcePoint[0] = totContactTorque[2]/totContactForce[1];
			resultantForcePoint[1] = rdMath::NAN;
			resultantForcePoint[2] = -totContactTorque[0]/totContactForce[1];
		} else {
			for(j=0;j<3;j++){
				resultantForcePoint[j] = rdMath::NAN;
			}
		}
		for(j=0;j<3;j++){
			index = Mtx::ComputeIndex(g,3,j);
			_resultantForcePoints[index] = resultantForcePoint[j];
			_totContactForces[index] = totContactForce[j];
			_totContactTorques[index] = totContactTorque[j];

		}
	}
	if(_nResultantForcePointGroups>0)
		_resultantForcePointsStore->append(tReal,3*_nResultantForcePointGroups,_resultantForcePoints);
		_totFStore->append(tReal,3*_nResultantForcePointGroups,_totContactForces);
		_totTorqueStore->append(tReal,3*_nResultantForcePointGroups,_totContactTorques);


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
int Contact::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_pStore->reset(aT);
	_vStore->reset(aT);
	_fStore->reset(aT);
	_pwrStore->reset(aT);
	_resultantForcePointsStore->reset(aT);
	_totFStore->reset(aT);
	_totTorqueStore->reset(aT);



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
int Contact::
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
int Contact::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	record(aT,aX,aY);

	return(0);
}


//=============================================================================
// UTILITIES
//=============================================================================
//_____________________________________________________________________________
/**
 * Reset all storage objects.  That is, delete all internally stored
 * data so that new data may be stored.
 */
void Contact::
resetStorage()
{
	if(_pStore!=NULL) _pStore->reset();
	if(_vStore!=NULL) _vStore->reset();
	if(_fStore!=NULL) _fStore->reset();
	if(_pwrStore!=NULL) _pwrStore->reset();
	if(_resultantForcePointsStore!=NULL) _resultantForcePointsStore->reset();
	if(_totFStore!=NULL) _totFStore->reset();
	if(_totTorqueStore!=NULL) _totTorqueStore->reset();

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
int Contact::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	if(!getOn()) {
		printf("Contact.printResults: Off- not printing.\n");
		return(-1);
	}

	// POINTS
	Storage::printResult(_pStore,aBaseName+"_"+getName()+"_points",aDir,aDT,aExtension);

	// VELOCITIES
	Storage::printResult(_vStore,aBaseName+"_"+getName()+"_velocities",aDir,aDT,aExtension);

	// FORCES
	Storage::printResult(_fStore,aBaseName+"_"+getName()+"_forces",aDir,aDT,aExtension);

	// POWERS
	Storage::printResult(_pwrStore,aBaseName+"_"+getName()+"_powers",aDir,aDT,aExtension);

	// RESULTANT FORCE POINT
	Storage::printResult(_resultantForcePointsStore,aBaseName+"_"+getName()+"_resultantForcePoints",aDir,aDT,aExtension);

	// TOTAL CONTACT FORCES
	Storage::printResult(_totFStore,aBaseName+"_"+getName()+"_totForce",aDir,aDT,aExtension);

	// TOTAL CONTACT TORQUES
	Storage::printResult(_totTorqueStore,aBaseName+"_"+getName()+"_totTorque",aDir,aDT,aExtension);

	return(0);
}


