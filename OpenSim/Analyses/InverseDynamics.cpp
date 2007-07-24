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
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/SpeedSet.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Model/GeneralizedForce.h>
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
	if(_ownsActuatorSet) delete _actuatorSet;
}
//_____________________________________________________________________________
/**
 */
InverseDynamics::InverseDynamics(Model *aModel) :
	Analysis(aModel),
	_useModelActuatorSet(_useModelActuatorSetProp.getValueBool())
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
	Analysis(aInverseDynamics),
	_useModelActuatorSet(_useModelActuatorSetProp.getValueBool())
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
	_useModelActuatorSet = true;
	_storage = NULL;
	_ownsActuatorSet = false;
	_actuatorSet = NULL;

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
	_useModelActuatorSetProp.setComment("If true, the model's own actuator set will be used in the inverse dynamics computation.  "
													"Otherwise, inverse dynamics generalized forces will be computed for all unconstrained degrees of freedom.");
	_useModelActuatorSetProp.setName("use_model_actuator_set");
	_propertySet.append(&_useModelActuatorSetProp);
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
	if(_model) 
		for (int i=0; i < _actuatorSet->getSize(); i++) labels.append(_actuatorSet->get(i)->getName());
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

	if(_model) {
		// Update the _actuatorSet we'll be computing inverse dynamics for
		if(_ownsActuatorSet) delete _actuatorSet;
		if(_useModelActuatorSet) {
			// Set pointer to model's internal actuator set
			_actuatorSet = _model->getActuatorSet();
			_ownsActuatorSet = false;
		} else {
			// Generate an actuator set consisting of a generalized force actuator for every unconstrained degree of freedom
			_actuatorSet = GeneralizedForce::CreateActuatorSetOfGeneralizedForcesForModel(_model,1,false);
			_ownsActuatorSet = true;
		}

		// Gather indices into speed set corresponding to the unconstrained degrees of freedom (for which we will set acceleration constraints)
		_accelerationIndices.setSize(0);
		SpeedSet *speedSet = _model->getDynamicsEngine().getSpeedSet();
		for(int i=0; i<speedSet->getSize(); i++) {
			AbstractCoordinate *coord = speedSet->get(i)->getCoordinate();
			if(!coord->getLocked() && !coord->getConstrained()) {
				_accelerationIndices.append(i);
			}
		}

		_dydt.setSize(_model->getNumCoordinates() + _model->getNumSpeeds());

		int nf = _actuatorSet->getSize();
		int nacc = _accelerationIndices.getSize();

		if(nf < nacc) 
			throw(Exception("InverseDynamics: ERROR- overconstrained system -- need at least as many actuators as there are degrees of freedom.\n"));

		_constraintMatrix.resize(nacc,nf);
		_constraintVector.resize(nacc);

		_performanceMatrix.resize(nf,nf);
		_performanceMatrix = 0;
		for(int i=0; i<nf; i++) {
			_actuatorSet->get(i)->setForce(1);
			_performanceMatrix(i,i) = _actuatorSet->get(i)->getStress();
		}

		_performanceVector.resize(nf);
		_performanceVector = 0;

		int lwork = nf + nf + nacc;
		_lapackWork.resize(lwork);
	}

	// DESCRIPTION AND LABELS
	constructDescription();
	constructColumnLabels();

	deleteStorage();
	allocateStorage();
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
	_actuatorSet->computeActuation();
	_model->getDerivCallbackSet()->computeActuation(aT,aX,aY);
	for(int i=0;i<_actuatorSet->getSize();i++) _actuatorSet->get(i)->setForce(aF[i]);
	_actuatorSet->apply();
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
record(double aT,double *aX,double *aY,double *aDYDT)
{
	if(!_model) return -1;
	if(!aDYDT) throw Exception("InverseDynamics: ERROR- Needs state derivatives.",__FILE__,__LINE__);

	int nf = _actuatorSet->getSize();
	int nacc = _accelerationIndices.getSize();
	int nq = _model->getNumCoordinates();

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
		double targetAcceleration = aDYDT[nq+_accelerationIndices[i]];
		_constraintVector[i] = targetAcceleration - _constraintVector[i];
	}

	// LAPACK SOLVER
	// NOTE: It destroys the matrices/vectors we pass to it, so we need to pass it copies of performanceMatrix and performanceVector (don't bother making
	// copies of _constraintMatrix/Vector since those are reinitialized each time anyway)
	int info;
	SimTK::Matrix performanceMatrixCopy = _performanceMatrix;
	SimTK::Vector performanceVectorCopy = _performanceVector;
	dgglse_(nf, nf, &nacc, &performanceMatrixCopy(0,0), nf, &_constraintMatrix(0,0), nacc, &performanceVectorCopy[0], &_constraintVector[0], &f[0], &_lapackWork[0], _lapackWork.size(), info);

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
 * @param aYP Current pseudo states.
 * @param aDYDT Current state derivatives.
 * @param aClientData General use pointer for sending in client data.
 *
 * @return -1 on error, 0 otherwise.
 */
int InverseDynamics::
begin(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	// RESET STORAGE
	_storage->reset(aT);

	// RECORD
	int status = 0;
	if(_storage->getSize()<=0) {
		status = record(aT,aX,aY,aDYDT);
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
int InverseDynamics::
step(double *aXPrev,double *aYPrev,double *aYPPrev,
	int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
	void *aClientData)
{
	if(!proceed(aStep)) return(0);

	record(aT,aX,aY,aDYDT);

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
int InverseDynamics::
end(int aStep,double aDT,double aT,double *aX,double *aY,double *aYP,double *aDYDT,
		void *aClientData)
{
	if(!proceed()) return(0);

	record(aT,aX,aY,aDYDT);

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


