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
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Torque.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
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
	_useModelActuatorSet(_useModelActuatorSetProp.getValueBool()),
	_activationExponent(_activationExponentProp.getValueDbl()),
	_useMusclePhysiology(_useMusclePhysiologyProp.getValueBool())
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
	_useModelActuatorSet(_useModelActuatorSetProp.getValueBool()),
	_activationExponent(_activationExponentProp.getValueDbl()),
	_useMusclePhysiology(_useMusclePhysiologyProp.getValueBool())
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
	_activationExponent=aStaticOptimization._activationExponent;

	_useMusclePhysiology=aStaticOptimization._useMusclePhysiology;
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
	_activationStorage = NULL;
	_forceStorage = NULL;
	_ownsActuatorSet = false;
	_actuatorSet = NULL;
	_activationExponent=2;
	_useMusclePhysiology=true;

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
	_useModelActuatorSetProp.setComment("If true, the model's own actuator set will be used in the static optimization computation.  "
													"Otherwise, inverse dynamics for generalized forces will be computed for all unconstrained degrees of freedom.");
	_useModelActuatorSetProp.setName("use_model_actuator_set");
	_propertySet.append(&_useModelActuatorSetProp);

	_activationExponentProp.setComment(
		"A double indicating the exponent to raise activations to when solving static optimization.  ");
	_activationExponentProp.setName("activation_exponent");
	_propertySet.append(&_activationExponentProp);

	
	_useMusclePhysiologyProp.setComment(
		"If true muscle force-length curve is observed while running optimization.");
	_useMusclePhysiologyProp.setName("use_muscle_physiology");
	_propertySet.append(&_useMusclePhysiologyProp);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a description for the static optimization files.
 */
void StaticOptimization::
constructDescription()
{
	string descrip = "This file contains static optimization results.\n\n";
	setDescription(descrip);
}

//_____________________________________________________________________________
/**
 * Construct column labels for the static optimization files.
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
	_activationStorage = new Storage(1000,"Static Optimization");
	_activationStorage->setDescription(getDescription());
	_activationStorage->setColumnLabels(getColumnLabels());

	_forceStorage = new Storage(1000,"Static Optimization");
	_forceStorage->setDescription(getDescription());
	_forceStorage->setColumnLabels(getColumnLabels());

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
	delete _activationStorage; _activationStorage = NULL;
	delete _forceStorage; _forceStorage = NULL;
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the model for which the static optimization is to be computed.
 *
 * @param aModel Model pointer
 */
void StaticOptimization::
setModel(Model *aModel)
{
	Analysis::setModel(aModel);

	if(_model) {
		// Update the _actuatorSet we'll be computing static optimization/inverse dynamics for
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

		int na = _actuatorSet->getSize();
		int nacc = _accelerationIndices.getSize();

		if(na < nacc) 
			throw(Exception("StaticOptimization: ERROR- overconstrained system -- need at least as many actuators as there are degrees of freedom.\n"));

		_constraintMatrix.resize(nacc,na);
		_constraintVector.resize(nacc);

		_performanceMatrix.resize(na,na);
		_performanceMatrix = 0;
		for(int i=0; i<na; i++) {
			_actuatorSet->get(i)->setForce(1);
			_performanceMatrix(i,i) = _actuatorSet->get(i)->getStress();
		}

		_performanceVector.resize(na);
		_performanceVector = 0;

		int lwork = na + na + nacc;
		_lapackWork.resize(lwork);

		_parameters.resize(na);
		_parameters = 0;
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
 * Get the activation storage.
 *
 * @return Activation storage.
 */
Storage* StaticOptimization::
getActivationStorage()
{
	return(_activationStorage);
}
//_____________________________________________________________________________
/**
 * Get the force storage.
 *
 * @return Force storage.
 */
Storage* StaticOptimization::
getForceStorage()
{
	return(_forceStorage);
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
	_activationStorage->setCapacityIncrement(aIncrement);
	_forceStorage->setCapacityIncrement(aIncrement);
}

//=============================================================================
// ANALYSIS
//=============================================================================
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

	int na = _actuatorSet->getSize();
	int nacc = _accelerationIndices.getSize();

	// IPOPT
	_optimizerDX = 0.0001;
	_optimizerAlgorithm = "ipopt";
	_printLevel = 0;
	_convergenceCriterion = 1e-004;
	_maxIterations = 2000;

	// Optimization target
	StaticOptimizationTarget target(_model,na,nacc,aT,aX,aY,aDYDT,_useMusclePhysiology);
	target.setActivationExponent(_activationExponent);
	target.setDX(_optimizerDX);

	// Pick optimizer algorithm
	SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
	//SimTK::OptimizerAlgorithm algorithm = SimTK::CFSQP;

	// Optimizer
	SimTK::Optimizer *optimizer = new SimTK::Optimizer(target, algorithm);

	// Optimizer options
	//cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
	optimizer->setDiagnosticsLevel(_printLevel);
	//cout<<"Setting optimizer convergence criterion to "<<_convergenceCriterion<<".\n";
	optimizer->setConvergenceTolerance(_convergenceCriterion);
	//cout<<"Setting optimizer maximum iterations to "<<_maxIterations<<".\n";
	optimizer->setMaxIterations(_maxIterations);
	optimizer->useNumericalGradient(false);
	optimizer->useNumericalJacobian(false);
	if(algorithm == SimTK::InteriorPoint) {
		// Some IPOPT-specific settings
		optimizer->setLimitedMemoryHistory(500); // works well for our small systems
		optimizer->setAdvancedBoolOption("warm_start",true);
		optimizer->setAdvancedRealOption("obj_scaling_factor",1);
		optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",1);
	}

	// Parameter bounds
	SimTK::Vector lowerBounds(na), upperBounds(na);
	//for(int i=0; i<na; i++) {
	//	if(_actuatorSet->get(i)->getType()=="Thelen2003Muscle") {
	//		// Note:  Using Thelen2003Muscle here, but getMaxIsometricForce() could be in AbstractMuscle??
	//		Thelen2003Muscle *aMuscle = dynamic_cast<Thelen2003Muscle*>(_actuatorSet->get(i));
	//		lowerBounds(i) = 0;
	//		upperBounds(i) = aMuscle->getMaxIsometricForce();
	//	} else if(_actuatorSet->get(i)->getType()=="Schutte1993Muscle") {
	//		// Note:  Using Schutte1993Muscle here, but getMaxIsometricForce() could be in AbstractMuscle??
	//		Schutte1993Muscle *aMuscle = dynamic_cast<Schutte1993Muscle*>(_actuatorSet->get(i));
	//		lowerBounds(i) = 0;
	//		upperBounds(i) = aMuscle->getMaxIsometricForce();
	//	} else if(_actuatorSet->get(i)->getType()=="GeneralizedForce") { 
	//		// Note:  Using GeneralizedForce here, but getMinForce() and getMaxForce() could be in AbstractActuator??
	//		GeneralizedForce *aGeneralizedForce = dynamic_cast<GeneralizedForce*>(_actuatorSet->get(i));
	//		lowerBounds(i) = aGeneralizedForce->getMinForce();
	//		upperBounds(i) = aGeneralizedForce->getMaxForce();
	//	} else if(_actuatorSet->get(i)->getType()=="Force") { 
	//		// Note:  Using Force here, but getMinForce() and getMaxForce() could be in AbstractActuator??
	//		Force *aForce = dynamic_cast<Force*>(_actuatorSet->get(i));
	//		lowerBounds(i) = aForce->getMinForce();
	//		upperBounds(i) = aForce->getMaxForce();
	//	//} else if(_actuatorSet->get(i)->getType()=="Torque") { 
	//	//	// Note:  Using Torque here, but getMinForce() and getMaxForce() could be in AbstractActuator??
	//	//	OpenSim::Torque *aTorque = dynamic_cast<OpenSim::Torque*>(_actuatorSet->get(i));
	//	//	lowerBounds(i) = aTorque->getMinForce();
	//	//	upperBounds(i) = aTorque->getMaxForce();
	//	} else {
	//		lowerBounds(i) = -10000;
	//		upperBounds(i) = 10000;
	//	}
	//}
	AbstractActuator *act;
	AbstractMuscle *mus;
	for(int a=0;a<na;a++) {
		act = _actuatorSet->get(a);
		mus = dynamic_cast<AbstractMuscle*>(act);
		if(mus==NULL) {
			lowerBounds(a) = -1;
			upperBounds(a) = 1;
		} else {
			lowerBounds(a) = 0.01;
			upperBounds(a) = 1;
		}
	}
	target.setParameterLimits(lowerBounds, upperBounds);

	_parameters = 0; // Set initial guess to zeros

	// Static optimization
	target.prepareToOptimize(&_parameters[0]);

	//LARGE_INTEGER start;
	//LARGE_INTEGER stop;
	//LARGE_INTEGER frequency;

	//QueryPerformanceFrequency(&frequency);
	//QueryPerformanceCounter(&start);

	try {
		optimizer->optimize(_parameters);
	}
	catch (const SimTK::Exception::Base &ex) {
		cout << ex.getMessage() << endl;
		cout << "OPTIMIZATION FAILED..." << endl;
		cout << endl;
		cout << "StaticOptimization.record:  WARN- The optimizer could not find " << "a solution at time = " << aT << endl;
		cout << "Starting at a slightly different initial time may help." << endl;
		cout << "Try increasing the strength of the following actuator(s):" << endl;
		AbstractActuator *act;
		AbstractMuscle *mus;
		double tol = 1e-6;
		for(int a=0;a<na;a++) {
			act = _actuatorSet->get(a);
			mus = dynamic_cast<AbstractMuscle*>(act);
			if(mus==NULL) {
				if(_parameters(a) < (lowerBounds(a)+tol) || _parameters(a) > (upperBounds(a)-tol)) {
					cout << act->getName() << endl;
				}
			} else {
				if(_parameters(a) > (upperBounds(a)-tol)) {
					cout << mus->getName() << endl;
				}
			}
		}
		cout << endl;
	}

	//QueryPerformanceCounter(&stop);
	//double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
	//cout << "optimizer time = " << (duration*1.0e3) << " milliseconds" << endl;

	target.printPerformance(&_parameters[0]);

	_activationStorage->append(aT,na,&_parameters[0]);

	SimTK::Vector forces(na);
	target.getActuation(_parameters,forces);

	_forceStorage->append(aT,na,&forces[0]);

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
	_activationStorage->reset(aT);
	_forceStorage->reset(aT);

	// RECORD
	int status = 0;
	if(_activationStorage->getSize()<=0 && _forceStorage->getSize()<=0) {
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
	// ACTIVATIONS
	_activationStorage->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_activationStorage,aBaseName+"_"+getName()+"_activation",aDir,aDT,aExtension);

	// FORCES
	_forceStorage->scaleTime(_model->getTimeNormConstant());
	Storage::printResult(_forceStorage,aBaseName+"_"+getName()+"_force",aDir,aDT,aExtension);

	// Make a ControlSet out of activations for use in forward dynamics
	ControlSet cs(*_activationStorage);
	std::string path = (aDir=="") ? "." : aDir;
	std::string name = path + "/" + aBaseName+"_"+getName()+"_controls.xml";
	cs.print(name);
	return(0);
}