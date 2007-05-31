// InverseDynamics.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Eran Guendelman
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <SimTKmath.h>
#include <SimTKlapack.h>
#include "InverseDynamics.h"



using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InverseDynamics::~InverseDynamics()
{
	deleteStorage();
}
//_____________________________________________________________________________
/**
 */
InverseDynamics::InverseDynamics(Model *aModel) :
	Analysis(aModel)
{
	setNull();

	if(aModel) setModel(aModel);
	else allocateStorage();
}
// Copy constrctor and virtual copy 
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 */
InverseDynamics::InverseDynamics(const InverseDynamics &aInverseDynamics):
	Analysis(aInverseDynamics)
{
	setNull();
	// COPY TYPE AND NAME
	*this = aInverseDynamics;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* InverseDynamics::copy() const
{
	InverseDynamics *object = new InverseDynamics(*this);
	return(object);

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
InverseDynamics& InverseDynamics::
operator=(const InverseDynamics &aInverseDynamics)
{
	// BASE CLASS
	Analysis::operator=(aInverseDynamics);
	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void InverseDynamics::
setNull()
{
	setupProperties();

	// OTHER VARIABLES
	_storage = NULL;
	_uSet = NULL;

	setType("InverseDynamics");
	setName("InverseDynamics");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InverseDynamics::
setupProperties()
{
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the body kinematics files.
 */
void InverseDynamics::
constructDescription()
{
	string descrip = "This file contains inverse dynamics results.\n\n";
	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void InverseDynamics::
constructColumnLabels()
{
	Array<string> labels;
	labels.append("time");
	if(_model) {
		ActuatorSet *ai = _model->getActuatorSet();
		for (int i=0; i < ai->getSize(); i++) labels.append(ai->get(i)->getName());
	}
	setColumnLabels(labels);
}

//_____________________________________________________________________________
/**
 * Allocate storage for the kinematics.
 */
void InverseDynamics::
allocateStorage()
{
	_storage = new Storage(1000,"Inverse Dynamics");
	_storage->setDescription(getDescription());
	_storage->setColumnLabels(getColumnLabels());
}


//=============================================================================
// DESTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Delete storage objects.
 */
void InverseDynamics::
deleteStorage()
{
	delete _storage; _storage = NULL;
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the body kinematics are to be computed.
 *
 * @param aModel Model pointer
 */
void InverseDynamics::
setModel(Model *aModel)
{
	Analysis::setModel(aModel);

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	deleteStorage();
	allocateStorage();

	delete _uSet; _uSet = NULL;
	if(_model) {
//		if(_model->getActuatorSet()->getNumStates() > 0 || _model->getContactSet()->getNumStates() > 0)
//			throw Exception("InverseDynamics analysis can't deal with models that have ActuatorSet/ContactSet with states",__FILE__,__LINE__);

		//TODO: FIX THIS!! MAKE IT GENERAL!
		//
		// Total hack/hardcoded for now
		Storage desiredKinStore("Results.baseline/subject01_walk1_RRA1_Kinematics_q.sto");
		Storage *qStore=NULL, *uStore=NULL;
		_model->getDynamicsEngine().formCompleteStorages(desiredKinStore,qStore,uStore);
		_model->getDynamicsEngine().convertDegreesToRadians(*uStore);
		_uSet = new GCVSplineSet(5,uStore);

		std::cout << "Creating map to accelerations" << std::endl;
		_accelerationIndices.setSize(0);
		SpeedSet *speedSet = _model->getDynamicsEngine().getSpeedSet();
		for(int i=0; i<speedSet->getSize(); i++) {
			AbstractCoordinate *coord = speedSet->get(i)->getCoordinate();
			if(!coord->getLocked() && !coord->getConstrained()) {
				std::cout << speedSet->get(i)->getCoordinate()->getName() << std::endl;
				_accelerationIndices.append(i);
			}
		}

		_dydt.setSize(_model->getNumCoordinates() + _model->getNumSpeeds());

		int nf = _model->getNumActuators();
		int nacc = _accelerationIndices.getSize();

		_constraintMatrix.resize(nacc,nf);
		_constraintVector.resize(nacc);

		_performanceMatrix.resize(nf,nf);
		_performanceMatrix = 0;
		for(int i=0; i<nf; i++) {
			_model->getActuatorSet()->get(i)->setForce(1);
			_performanceMatrix(i,i) = _model->getActuatorSet()->get(i)->getStress();
		}

		_performanceVector.resize(nf);
		_performanceVector = 0;

		int lwork = nf + nf + nacc;
		_lapackWork.resize(lwork);
	}
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
Storage* InverseDynamics::
getStorage()
{
	return(_storage);
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
void InverseDynamics::
setStorageCapacityIncrements(int aIncrement)
{
	_storage->setCapacityIncrement(aIncrement);
}

//=============================================================================
// ANALYSIS
//=============================================================================
//
void InverseDynamics::
computeAcceleration(double aT,double *aX,double *aY,double *aF,double *rAccel) const
{
	// SET
	_model->set(aT,aX,aY);
	_model->getDerivCallbackSet()->set(aT,aX,aY);

	// ACTUATION
	_model->getActuatorSet()->computeActuation();
	int nf = _model->getNumActuators();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	ActuatorSet *actuatorSet = _model->getActuatorSet();
	for(int i=0;i<nf;i++) actuatorSet->get(i)->setForce(aF[i]);
	_model->getActuatorSet()->apply();
	_model->getDerivCallbackSet()->applyActuation(aT,aX,aY);

	// CONTACT
	_model->getContactSet()->computeContact();
	_model->getDerivCallbackSet()->computeContact(aT,aX,aY);
	_model->getContactSet()->apply();
	_model->getDerivCallbackSet()->applyContact(aT,aX,aY);

	// ACCELERATIONS
	int nq = _model->getNumCoordinates();
	_model->getDynamicsEngine().computeDerivatives(&_dydt[0],&_dydt[nq]);
	_model->getDerivCallbackSet()->computeDerivatives(aT,aX,aY,&_dydt[0]);

	for(int i=0; i<_accelerationIndices.getSize(); i++) rAccel[i] = _dydt[nq+_accelerationIndices[i]];
}
//
//_____________________________________________________________________________
/**
 * Record the kinematics.
 */
int InverseDynamics::
record(double aT,double *aX,double *aY)
{
	if(!_model) return -1;

	int nf = _model->getNumActuators();
	int nacc = _accelerationIndices.getSize();

	// Build linear constraint matrix and constant constraint vector
	SimTK::Vector f(nf), c(nacc);
	f = 0;
	computeAcceleration(aT, aX, aY, &f[0], &_constraintVector[0]);

	for(int j=0; j<nf; j++) {
		f[j] = 1;
		computeAcceleration(aT, aX, aY, &f[0], &c[0]);
		for(int i=0; i<nacc; i++) _constraintMatrix(i,j) = (c[i] - _constraintVector[i]);
		f[j] = 0;
	}

	for(int i=0; i<nacc; i++) {
		double targetAcceleration = _uSet->evaluate(_accelerationIndices[i], 1, aT);
		_constraintVector[i] = targetAcceleration - _constraintVector[i];
	}

	// LAPACK SOLVER
	int info;
	dgglse_(nf, nf, &nacc, &_performanceMatrix(0,0), nf, &_constraintMatrix(0,0), nacc, &_performanceVector[0], &_constraintVector[0], &f[0], &_lapackWork[0], _lapackWork.size(), info);

	_storage->append(aT,nf,&f[0]);

	return 0;
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
int InverseDynamics::
begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_storage->reset(aT);

	// RECORD
	int status = 0;
	if(_storage->getSize()<=0) {
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
int InverseDynamics::
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
int InverseDynamics::
end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData)
{
	if(!proceed()) return(0);

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
int InverseDynamics::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACCELERATIONS
	_storage->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_storage,aBaseName+"_"+getName()+"_force",aDir,aDT,aExtension);

	return(0);
}


