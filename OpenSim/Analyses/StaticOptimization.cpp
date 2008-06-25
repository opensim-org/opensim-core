// StaticOptimization.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Jeff Reinbolt
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/IO.h>
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
#include "StaticOptimization.h"
#include "StaticOptimizationTarget.h"


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
StaticOptimization::~StaticOptimization()
{
	deleteStorage();
	if(_ownsActuatorSet) delete _actuatorSet;
}
//_____________________________________________________________________________
/**
 */
StaticOptimization::StaticOptimization(Model *aModel) :
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
StaticOptimization::StaticOptimization(const StaticOptimization &aStaticOptimization):
	Analysis(aStaticOptimization),
	_useModelActuatorSet(_useModelActuatorSetProp.getValueBool())
{
	setNull();
	// COPY TYPE AND NAME
	*this = aStaticOptimization;
}
//_____________________________________________________________________________
/**
 * Clone
 *
 */
Object* StaticOptimization::copy() const
{
	StaticOptimization *object = new StaticOptimization(*this);
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
StaticOptimization& StaticOptimization::
operator=(const StaticOptimization &aStaticOptimization)
{
	// BASE CLASS
	Analysis::operator=(aStaticOptimization);

	_useModelActuatorSet = aStaticOptimization._useModelActuatorSet;

	return(*this);
}

//_____________________________________________________________________________
/**
 * SetNull().
 */
void StaticOptimization::
setNull()
{
	setupProperties();

	// OTHER VARIABLES
	_useModelActuatorSet = true;
	_storage = NULL;
	_ownsActuatorSet = false;
	_actuatorSet = NULL;

	setType("StaticOptimization");
	setName("StaticOptimization");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void StaticOptimization::
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
void StaticOptimization::
constructDescription()
{
	string descrip = "This file contains inverse dynamics results.\n\n";
	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the body kinematics files.
 */
void StaticOptimization::
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
void StaticOptimization::
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
void StaticOptimization::
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
void StaticOptimization::
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
			throw(Exception("StaticOptimization: ERROR- overconstrained system -- need at least as many actuators as there are degrees of freedom.\n"));

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
Storage* StaticOptimization::
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
void StaticOptimization::
setStorageCapacityIncrements(int aIncrement)
{
	_storage->setCapacityIncrement(aIncrement);
}

//=============================================================================
// ANALYSIS
//=============================================================================
//
void StaticOptimization::
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
 * Record the results.
 */
int StaticOptimization::
record(double aT,double *aX,double *aY,double *aDYDT)
{
	if(!_model) return -1;
	if(!aDYDT) throw Exception("StaticOptimization: ERROR- Needs state derivatives.",__FILE__,__LINE__);

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

	// LAPACK SOLVER (may result in negative muscle forces)
	// NOTE: It destroys the matrices/vectors we pass to it, so we need to pass it copies of performanceMatrix and performanceVector (don't bother making
	// copies of _constraintMatrix/Vector since those are reinitialized each time anyway)
	//int info;
	//SimTK::Matrix performanceMatrixCopy = _performanceMatrix;
	//SimTK::Vector performanceVectorCopy = _performanceVector;
	//dgglse_(nf, nf, &nacc, &performanceMatrixCopy(0,0), nf, &_constraintMatrix(0,0), nacc, &performanceVectorCopy[0], &_constraintVector[0], &f[0], &_lapackWork[0], _lapackWork.size(), info);

	// IPOPT
	_optimizerDX = 0.0001;
	_optimizerAlgorithm = "ipopt";
	_printLevel = 0;
	_convergenceCriterion = 1e-006;
	_maxIterations = 2000;


	// Optimization target
	StaticOptimizationTarget target(nf);

	target.setDX(_optimizerDX);

	// Pick optimizer algorithm
	SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
	if(IO::Uppercase(_optimizerAlgorithm) == "CFSQP") {
		if(!SimTK::Optimizer::isAlgorithmAvailable(SimTK::CFSQP)) {
			std::cout << "CFSQP optimizer algorithm unavailable.  Will try to use IPOPT instead." << std::endl;
			algorithm = SimTK::InteriorPoint;
		} else {
			std::cout << "Using CFSQP optimizer algorithm." << std::endl;
			algorithm = SimTK::CFSQP;
		}
	} else if(IO::Uppercase(_optimizerAlgorithm) == "IPOPT") {
		std::cout << "Using IPOPT optimizer algorithm." << std::endl;
		algorithm = SimTK::InteriorPoint;
	} else {
		throw Exception("CMCTool: ERROR- Unrecognized optimizer algorithm: '"+_optimizerAlgorithm+"'",__FILE__,__LINE__);
	}

	SimTK::Optimizer *optimizer = new SimTK::Optimizer(target, algorithm);

	cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
	optimizer->setDiagnosticsLevel(_printLevel);
	cout<<"Setting optimizer convergence criterion to "<<_convergenceCriterion<<".\n";
	optimizer->setConvergenceTolerance(_convergenceCriterion);
	cout<<"Setting optimizer maximum iterations to "<<_maxIterations<<".\n";
	optimizer->setMaxIterations(_maxIterations);
	optimizer->useNumericalGradient(false); // Use our own central difference approximations
	optimizer->useNumericalJacobian(false);
	if(algorithm == SimTK::InteriorPoint) {
		// Some IPOPT-specific settings
		optimizer->setLimitedMemoryHistory(500); // works well for our small systems
		optimizer->setAdvancedBoolOption("warm_start",true);
		optimizer->setAdvancedRealOption("obj_scaling_factor",1);
		optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",100);
	}

	try {
		optimizer->optimize(f);
	}
	catch (const SimTK::Exception::Base &ex) {
		cout << ex.getMessage() << endl;
		cout << "OPTIMIZATION FAILED..." << endl;
	}

	// NNLS: Non-negative least squares based on Lawson and Hanson, "Solving Least Squares Problems", Prentice-Hall, 1974.
	// Initialize
	//SimTK::Array<int> pArray(nf,0);
	//SimTK::Array<int> zArray(nf,0);
	//for(int i=0; i<nf; i++) {
	//	zArray[i] = i;
	//}
	//SimTK::Matrix xMatrix(nf,1);
	//SimTK::Array<int> ppArray(nf,0);
	//SimTK::Array<int> zzArray(nf,0);
	//for(int i=0; i<nf; i++) {
	//	zzArray[i] = i;
	//}
	//SimTK::Matrix residMatrix = _performanceVector.getAsMatrix(); // resid = d-C*x, since x=0 then resid = d
	//SimTK::Matrix wMatrix = ~_performanceMatrix*residMatrix;
	//SimTK::Matrix cpMatrix(nf,nf); cpMatrix = 0;
	//SimTK::Matrix zerosMatrix(nf,1); zerosMatrix = 0;
	//SimTK::Vector littleZVector(nf);

	//SimTK::Matrix tempMatrix(nf,1);

	//// Set up iteration crterion
	//int outerIter = 0;
	//int iter = 0;
	//int iterMax = 3*nf;
	//int t = 0;
	//int counter = 0;

	//// Outer loop to put variables into set to hold positive coefficients
	//int outerFlag1 = 1;
	//int outerFlag2 = 1;
	//while(outerFlag1 && outerFlag2) {
	//	outerIter++;

	//	// Find index of the maximum element in wMatrix
	//	for(int i=1; i<zzArray.size(); i++) {
	//		if(wMatrix(zzArray[i],0) > wMatrix(zzArray[i-1],0)) {
	//			t = zzArray[i];
	//		}
	//	}

	//	// Update element of pArray and zArray
	//	pArray[t] = t;
	//	zArray[t] = -1;

	//	// Find indices of nonzero elements in pArray
	//	counter = 0;
	//	for(int i=0; i<pArray.size(); i++) {
	//		if(pArray[i] != 0) {
	//			ppArray[counter] = pArray[i];
	//			counter++;
	//		}
	//	}
	//	ppArray.resize(counter);

	//	// Find indices of nonzero elements in zArray
	//	counter = 0;
	//	cout<<zArray[0]<<endl;
	//	for(int i=0; i<zArray.size(); i++) {
	//		if(zArray[i] != -1) {
	//			zzArray[counter] = zArray[i];
	//			counter++;
	//		}
	//	}
	//	zzArray.resize(counter);

	//	// Define positive coefficients
	//	for(int i=0; i<ppArray.size(); i++) {
	//		cpMatrix.updCol(ppArray[i]) = _performanceMatrix.col(ppArray[i]);
	//	}

	//	for(int i=0; i<zzArray.size(); i++) {
	//		cpMatrix.updCol(zzArray[i]) = zerosMatrix.col(0);
	//	}
	//	cpMatrix.dump();

		//// LAPACK SOLVER
		//// NOTE: It destroys the matrices/vectors we pass to it, so we need to pass it copies of performanceMatrix and performanceVector (don't bother making
		//// copies of _constraintMatrix/Vector since those are reinitialized each time anyway)
		//int info;
		//SimTK::Matrix performanceMatrixCopy = cpMatrix;
		//SimTK::Vector performanceVectorCopy = _performanceVector;
		//dgglse_(nf, nf, &nacc, &performanceMatrixCopy(0,0), nf, &_constraintMatrix(0,0), nacc, &performanceVectorCopy[0], &_constraintVector[0], &f[0], &_lapackWork[0], _lapackWork.size(), info);

	//	littleZVector = f;
	//	for(int i=0; i<zzArray.size(); i++) {
	//		littleZVector(zzArray[i]) = 0;
	//	}

	//	outerFlag1 = 0;
	//	for(int i=0; i<zArray.size(); i++) {
	//		if(zArray[i] != -1) {
	//			outerFlag1 = 1;
	//			break;
	//		}
	//	}

	//	outerFlag2 = 0;
	//	for(int i=0; i<zzArray.size(); i++) {
	//		if(wMatrix(zzArray[i],0) > 1e-12) {
	//			outerFlag2 = 1;
	//			break;
	//		}
	//	}
	//}
	//cpMatrix.dump();
	//f.dump();

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
int StaticOptimization::
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
int StaticOptimization::
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
int StaticOptimization::
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
int StaticOptimization::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	// ACCELERATIONS
	_storage->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_storage,aBaseName+"_"+getName()+"_force",aDir,aDT,aExtension);

	return(0);
}


