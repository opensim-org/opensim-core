// rdCMC.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson, Eran Guendelman
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimToolsDLL.h"
#include <iostream>
#include <string>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/RootSolver.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ModelIntegrandForActuators.h>
#include <OpenSim/Simulation/Model/VectorFunctionForActuators.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/SQP/rdOptimizationTarget.h>
#include <simmath/Optimizer.h>
#include "rdCMC.h"
#include "rdCMC_Joint.h"
#include "rdCMC_TaskSet.h"
#include "rdActuatorForceTarget.h"

using namespace std;
using namespace OpenSim;
using SimTK::Vector;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
rdCMC::~rdCMC()
{
	delete _optimizer;
}
//_____________________________________________________________________________
/**
 * Contructor
 *
 * @param aModel Model that is to be controlled.
 * @param aTaskSet Set of tracking tasks.
 */
rdCMC::rdCMC(Model *aModel,rdCMC_TaskSet *aTaskSet) :
	Controller(aModel), _paramList(-1) , _f(0.0)
{
	// NULL
	setNull();

	// TRACK OBJECTS
	_taskSet = aTaskSet;
	if(_taskSet==NULL) {
		char tmp[Object::NAME_LENGTH];
		strcpy(tmp,"rdCMC.rdCMC: ERR- no ");
		strcat(tmp,"track objects.\n");
		throw(new Exception(tmp,__FILE__,__LINE__));
	}

	// STORAGE
	Array<string> labels;
	labels.append("time");
	for(int i=0;i<_taskSet->getSize();i++) labels.append(_taskSet->get(i)->getName());
	_pErrStore = new Storage(1000,"PositionErrors");
	_pErrStore->setColumnLabels(labels);
	_vErrStore = new Storage(1000,"VelocityErrors");
	_pErrStore->setColumnLabels(labels);
	_stressTermWeightStore = new Storage(1000,"StressTermWeight");
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void rdCMC::
setNull()
{
	_optimizer = NULL;
	_target = NULL;
	_taskSet = NULL;
	_dt = 0.0;
	_lastDT = 0.0;
	_restoreDT = false;
	_tf = rdMath::MINUS_INFINITY;
	_targetDT = 1.0e-3;
	_checkTargetTime = false;
	_pErrStore = NULL;
	_vErrStore = NULL;
	_stressTermWeightStore = NULL;
	_controlSet = NULL;
	_useCurvatureFilter = false;
	_useReflexes = false;
	_controlConstraintsFromReflexes = NULL;
	_paramList.setSize(0);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// CONTROL SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the control set that the controller is using to control the
 * simulation.
 *
 * @return Control set.
 */
ControlSet* rdCMC::
getControlSet() const
{
	return(_controlSet);
}

//-----------------------------------------------------------------------------
// TRACK OBJECT SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the task set for this controller.
 *
 * @return Task set.
 */
rdCMC_TaskSet* rdCMC::
getTaskSet() const
{
	return(_taskSet);
}

//-----------------------------------------------------------------------------
// PARAMETER LIST
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the list of parameters in the control set that the controller is
 * using to control the simulation.
 *
 * @return List of parameters in the control set serving as the controls.
 */
Array<int>* rdCMC::
getParameterList()
{
	return(&_paramList);
}

//-----------------------------------------------------------------------------
// OPTIMIZATION TARGET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimization target for this controller.
 *
 * @param aTarget Optimization target.
 * @return Previous optimization target.
 */
rdOptimizationTarget* rdCMC::
setOptimizationTarget(rdOptimizationTarget *aTarget, SimTK::Optimizer *aOptimizer)
{
	// PREVIOUS TARGET
	rdOptimizationTarget *prev = _target;

	// NEW TARGET
	_target = aTarget;

	if(aOptimizer) {
		delete _optimizer;
		_optimizer = aOptimizer;
	}

	return(prev);
}
//_____________________________________________________________________________
/**
 * Get the optimization target for this controller.
 *
 * @return Current optimization target.
 */
rdOptimizationTarget* rdCMC::
getOptimizationTarget() const
{
	return(_target);
}

//-----------------------------------------------------------------------------
// OPTIMIZER
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the optimizer.
 *
 * @return Optimizer.
 */
SimTK::Optimizer* rdCMC::
getOptimizer() const
{
	return _optimizer;
}

//-----------------------------------------------------------------------------
// DT- INTEGRATION STEP SIZE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the requested integrator time step size.
 *
 * @param aDT Step size (0.0 <= aDT).
 */
void rdCMC::
setDT(double aDT)
{
	_dt = aDT;
	if(_dt<0) _dt=0.0;
}
//_____________________________________________________________________________
/**
 * Get the requested integrator time step size.
 *
 * @return Step size.
 */
double rdCMC::
getDT() const
{
	return(_dt);
}

//-----------------------------------------------------------------------------
// TARGET TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the target time (or final time) for the controller.
 *
 * The function of the controller is to compute a set of controls that
 * are appropriate from the current time in a simulation to the
 * target time of the controller.  If an integrator is taking time steps
 * prior to the target time, the controls should not have to be computed again.
 *
 * @param aTargetTime Time in the furture for which the controls have been
 * computed.
 * @see getCheckTargetTime()
 */
void rdCMC::
setTargetTime(double aTargetTime)
{
	_tf = aTargetTime;
}
//_____________________________________________________________________________
/**
 * Get the target time.
 *
 * The target time is the time in the future for which the controls have been
 * calculated.  If an integrator is taking time steps prior to the target
 * time, the controls should not have to be computed again.
 *
 * @return Time in the furture for which the controls have been
 * computed.
 * @see getCheckTargetTime()
 */
double rdCMC::
getTargetTime() const
{
	return(_tf);
}

//-----------------------------------------------------------------------------
// TARGET DT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the target time step size.
 *
 * The target time step size is the step size used to compute a new target
 * time, once the former target time has been reached by the integrator.
 *
 * @param aDT Target time step size.  Must be greater than 1.0e-8.
 * @see setTargetTime()
 */
void rdCMC::
setTargetDT(double aDT)
{
	_targetDT = aDT;
	if(_targetDT<1.0e-8) _targetDT =  1.0e-8;
}
//_____________________________________________________________________________
/**
 * Get the target time step size.
 *
 * The target time step size is the step size used to compute a new target
 * time, once the former target time has been reached by the integrator.
 *
 * @return Target time step size.
 * @see setTargetTime()
 */
double rdCMC::
getTargetDT() const
{
	return(_targetDT);
}

//-----------------------------------------------------------------------------
// CHECK TARGET TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set whether or not to check the target time.
 *
 * @param aTrueFalse If true, the target time will be checked.  If false, the
 * target time will not be checked.
 * @see setTargetTime()
 */
void rdCMC::
setCheckTargetTime(bool aTrueFalse)
{
	_checkTargetTime = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to check the target time.
 *
 * @return True if the target time will be checked.  False if the target
 * time will not be checked.
 * @see setTargetTime()
 */
bool rdCMC::
getCheckTargetTime() const
{
	return(_checkTargetTime);
}

//-----------------------------------------------------------------------------
// ACTUATOR FORCE PREDICTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the predictor for actuator forces.
 */
void rdCMC::
setActuatorForcePredictor(VectorFunctionForActuators *aPredictor)
{
	_predictor = aPredictor;
}
//_____________________________________________________________________________
/**
 * Get the predictor for actuator forces.
 */
VectorFunctionForActuators* rdCMC::
getActuatorForcePredictor()
{
	return(_predictor);
}

//-----------------------------------------------------------------------------
// ERROR STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the storage object for position errors.
 *
 * @return Storage of position errors.
 */
Storage* rdCMC::
getPositionErrorStorage() const
{
	return(_pErrStore);
}
//_____________________________________________________________________________
/**
 * Get the storage object for velocity errors.
 *
 * @return Storage of velocity errors.
 */
Storage* rdCMC::
getVelocityErrorStorage() const
{
	return(_vErrStore);
}
//_____________________________________________________________________________
/**
 * Get the storage object for stress term weights.
 *
 * @return Storage of stress term weights.
 */
Storage* rdCMC::
getStressTermWeightStorage() const
{
	return(_stressTermWeightStore);
}


//=============================================================================
// COMPUTE
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the initial states for a simulation.
 *
 * The caller should send in an initial guess.  The Qs and Us are set
 * based on the desired trajectories.  The actuator states are set by
 * sovling for a desired set of actuator forces, and then letting the states
 * come to equilibrium for those forces.
 *
 * @param rTI Initial time in normalized time.  Note this is changed to
 * the time corresponding to the new initial states on return.
 * @param rYI Initial states.
 */
void rdCMC::
computeInitialStates(double &rTI,double *rYI)
{
	cout<<"\n\n=============================================\n";
	cout<<"rdCMC.computeInitialStates: ti="<<rTI<<endl;
	cout<<"=============================================\n";

	int i,j;

	int nx = _model->getNumControls();
	int ny = _model->getNumStates();
	int nqnu = _model->getNumCoordinates() + _model->getNumSpeeds();
	int N = _predictor->getNX();
	Array<double> y(0.0,ny),yi(0.0,ny);
	Array<double> xmin(0.01,N),forces(0.0,N);
	double normConstant = _model->getTimeNormConstant();
	double tiReal = rTI * normConstant;

	// TURN ANALYSES OFF
	_model->getAnalysisSet()->setOn(false);

	// COPY OF STARTING INITIAL STATES
	for(i=0;i<ny;i++) yi[i] = rYI[i];
	y = yi;

	// CONSTRUCT CONTROL SET
	ControlSet xiSet;
	ControlSet *predictorControlSet = (_predictor && _predictor->getIntegrand()) ? _predictor->getIntegrand()->getControlSet() : 0;
	for(i=0;i<nx;i++) {
		ControlConstant *x = new ControlConstant();
		x->setName(_model->getControlName(i));
		x->setIsModelControl(true);
		// This is not a very good way to set the bounds on the controls because ConrtolConstant only supports constant
		// min/max bounds but we may have time-dependent min/max curves specified in the controls constraints file
		if(predictorControlSet) {
			x->setDefaultParameterMin(predictorControlSet->get(i)->getDefaultParameterMin());
			x->setDefaultParameterMax(predictorControlSet->get(i)->getDefaultParameterMax());
		}
		xiSet.append(x);
	}

	// SET
	_model->setTime(rTI);
	_model->setInitialStates(&yi[0]);
	_model->setStates(&yi[0]);

	// ACTUATOR EQUILIBRIUM
	// 1
	obtainActuatorEquilibrium(tiReal,0.200,xmin,y,true);
	restoreConfiguration(nqnu,&yi[0],&y[0]);
	_model->setInitialStates(&y[0]);
	// 2
	obtainActuatorEquilibrium(tiReal,0.200,xmin,y,true);
	restoreConfiguration(nqnu,&yi[0],&y[0]);
	_model->setInitialStates(&y[0]);


	// CHANGE THE TARGET DT ON THE CONTROLLER TEMPORARILY
	double oldTargetDT = getTargetDT();
	double newTargetDT = 0.030;
	setTargetDT(newTargetDT);


	// GET PREDICTOR CONTROL SET
	// todo - this is not good coding
	ModelIntegrand *predictorIntegrand = _predictor->getIntegrand();
	ControlSet *pControlSet = predictorIntegrand->getControlSet();


	// REPEATEDLY CONTROL OVER THE FIRST TIME STEP
	double dtFirst;
	Array<double> xi(0.0,nx);
	for(i=0;i<2;i++) {

		// CLEAR ANY PREVIOUS CONTROL NODES
		for(j=0;j<pControlSet->getSize();j++) {
			ControlLinear *control = (ControlLinear*)pControlSet->get(j);
			control->clearControlNodes();
		}

		// COMPUTE CONTROLS
		dtFirst = 1.0e-8;
		computeControls(dtFirst,rTI,&y[0],xiSet);
		setTargetTime(rdMath::MINUS_INFINITY);

		// GET CONTROLS
		xiSet.getControlValues(rTI,xi);

		// ALLOW STATES TO CHANGE
		//_predictor->evaluate(&xi[0],&forces[0]);
		//_model->getStates(&y[0]);
		//restoreConfiguration(nqnu,&yi[0],&y[0]);
		//_model->setInitialStates(&y[0]);

		// EXTRACT ACTIVATIONS
		//for(k=0,j=nqnu;j<ny;k+=1,j+=2) xi[k] = y[j];
		//cout<<"Activations:\n";
		//cout<<xi<<endl;

		// OBTAIN EQUILIBRIUM
		if(i<1) {
			//for(j=0;j<nx;j++) xi[j] *= 0.5;
			obtainActuatorEquilibrium(tiReal,0.200,xi,y,true);
			restoreConfiguration(nqnu,&yi[0],&y[0]);
			_model->setInitialStates(&y[0]);
		}
	}

	// GET NEW STATES
	_predictor->evaluate(&xi[0],&forces[0]);
	_model->getStates(&y[0]);
	//restoreConfiguration(nqnu,&yi[0],&y[0]);
	//_model->setInitialStates(&y[0]);

	// SET STATES
	rTI += newTargetDT;
	for(i=0;i<ny;i++) rYI[i] = y[i];

	// CLEANUP
	setTargetDT(oldTargetDT);
	_model->getAnalysisSet()->setOn(true);
}


//_____________________________________________________________________________
/**
 * Obtain actuator equilibrium.  A series of long (e.g., 200 msec) integrations
 * are performed to allow time-dependent actuators forces to reach
 * equilibrium values.
 *
 * @param tiReal Initial time expressed in real time units.
 * @param dtReal Duration of the time interval.
 * @param x Array of control values.
 * @param y Array of states.
 * @param hold Flag indicating whether or not to hold model coordinates
 * constant (true) or let them change according to the desired trajectories
 * (false).
 */
void rdCMC::
obtainActuatorEquilibrium(double tiReal,double dtReal,
								  const Array<double> &x,Array<double> &y,bool hold)
{
	// HOLD COORDINATES
	ModelIntegrandForActuators *predictorIntegrand = 
		(ModelIntegrandForActuators*)_predictor->getIntegrand();
	if(hold) {
		predictorIntegrand->holdCoordinatesConstant(tiReal);
	} else {
		predictorIntegrand->releaseCoordinates();
	}

	// INITIALIZE
	_model->setInitialStates(&y[0]);
	_predictor->setInitialTime(tiReal);
	_predictor->setFinalTime(tiReal+dtReal);

	// INTEGRATE FORWARD
	Array<double> f(0.0,x.getSize());
	_predictor->evaluate(&x[0],&f[0]);
	_model->getStates(&y[0]);

	// RELEASE COORDINATES
	predictorIntegrand->releaseCoordinates();
}
//_____________________________________________________________________________
/**
 * A utility method used to restore the coordinates and speeds to initial
 * values.  The states associated with actuators are not changed.
 *
 * @param nqnu Sum of the number of coordinates and speeds (nq + nu).
 * @param yi Array of initial states.
 * @param y Array of states.
 */
void rdCMC::
restoreConfiguration(int nqnu,const double *yi,double *y)
{
	int i;
	for(i=0;i<nqnu;i++) y[i] = yi[i];
}



//_____________________________________________________________________________
/**
 * Compute the controls for a simulation.
 *
 * The caller should send in an initial guess.
 *
 * @param rDT Integration time step in normalized time that is to be taken
 * next.  Note that the controller can change the value of rDT.
 * @param aT Current time in normalized time.
 * @param aY Current states of the model.
 * @param rControlSet Control set used for the simulation.  This method
 * alters the control set in order to control the simulation.
 */
void rdCMC::
computeControls(double &rDT,double aT,const double *aY,
	ControlSet &rControlSet)
{
	// SET CONTROL SET
	_controlSet = &rControlSet;

	// RESTORE DT?
	if(_restoreDT) {
		rDT = _lastDT / 2.0;
		_restoreDT = false;
		printf("rdCMC.computeControls: restored dt to %.8lf\n",
			rDT);
	}

	// CHECK TO SEE IF TARGET TIME HAS BEEN ACHIEVED
	if(getCheckTargetTime()) {

		// CONTROLS SHOULD BE VALID
		if(aT<_tf) {
			// CHECK THAT THE NEXT STEP WILL NOT OVER STEP THE TARGET TIME
			if((aT + rDT) > _tf) {
				_lastDT = rDT;
				_restoreDT = true;
				rDT = _tf - aT;
			}
			// DO NOT CHANGE CONTROLS
			//cout<<"rdCMC.computeControls: skipping control computation!\n";
			return;

		// CONTROLS SHOULD BE RECOMPUTED- NEED A NEW TARGET TIME
		} else {
			_tf = aT + _targetDT;
		}
	}

	cout<<"\n\n----------------------------------\n";
	cout<<"rdCMC.computeControls: t="<<aT<<" dt="<<rDT<<endl;
	cout<<"rdCMC.computeControls: targetTime = "<<_tf<<endl;

	int i;

	// TURN ANALYSES OFF
	_model->getAnalysisSet()->setOn(false);

	// TIME STUFF
	//BUG? _tf = aT + _targetDT;
	_model->setTime(_tf);
	double normConstant = _model->getTimeNormConstant();
	double tiReal = aT * normConstant;
	double tfReal = _tf * normConstant;

	// SET STATES FOR THE PREDICTOR
	_model->setStates(aY);
	_model->setInitialStates(aY);

	// SET CORRECTIONS FOR AN rdModelIntegradForActuators
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	ModelIntegrand *predictorIntegrand = _predictor->getIntegrand();
	//if(predictorIntegrand->getType()=="rdModelInegrandForActuators") {
		ModelIntegrandForActuators *actuatorIntegrand = (ModelIntegrandForActuators*) predictorIntegrand;
		FunctionSet *qSet = actuatorIntegrand->getCoordinateTrajectories();
		FunctionSet *uSet = actuatorIntegrand->getSpeedTrajectories();
		Array<double> qDesired(0.0,nq),uDesired(0.0,nu);
		qSet->evaluate(qDesired,0,tiReal);
		if(uSet!=NULL) {
			uSet->evaluate(uDesired,0,tiReal);
		} else {
			qSet->evaluate(uDesired,1,tiReal);
		}
		Array<double> qCorrection(0.0,nq),uCorrection(0.0,nu);
		for(i=0;i<nq;i++) qCorrection[i] = aY[i] - qDesired[i];
		for(i=0;i<nu;i++) uCorrection[i] = aY[i+nq] - uDesired[i];
		actuatorIntegrand->setCoordinateCorrections(&qCorrection[0]);
		actuatorIntegrand->setSpeedCorrections(&uCorrection[0]);
		//cout<<"\nQCorrections:\n"<<qCorrection;
		//cout<<"\nUCorrections:\n"<<uCorrection;

	//}

	// ERRORS
	_taskSet->computeErrors(tiReal);
	_taskSet->recordErrorsAsLastErrors();
	Array<double> &pErr = _taskSet->getPositionErrors();
	Array<double> &vErr = _taskSet->getVelocityErrors();
	cout<<"\nErrors at time "<<aT<<":"<<endl;
	rdCMC_Task *task;
	for(i=0;i<pErr.getSize();i++) {
		
		task = _taskSet->get(i);
		if(task==NULL) continue;

		cout<<task->getName()<<":  ";
		cout<<"pErr="<<pErr[i]<<" vErr="<<vErr[i]<<endl;
	}
	double *err = new double[pErr.getSize()];
	for(i=0;i<pErr.getSize();i++) err[i] = pErr[i];
	_pErrStore->append(tiReal,pErr.getSize(),err);
	for(i=0;i<vErr.getSize();i++) err[i] = vErr[i];
	_vErrStore->append(tiReal,vErr.getSize(),err);

	
	// COMPUTE DESIRED ACCELERATIONS
	_taskSet->computeDesiredAccelerations(tiReal,tfReal);

	// Set the weight of the stress term in the optimization target based on this sigmoid-function-blending
	// Note that if no task limits are set then by default the weight will be 1.
	// TODO: clean this up -- currently using dynamic_casts to make sure we're not using fast target, etc.
	if(dynamic_cast<rdActuatorForceTarget*>(_target)) {
		double relativeTau = 0.1;
		rdActuatorForceTarget *realTarget = dynamic_cast<rdActuatorForceTarget*>(_target);
	
		Array<double> &pErr = _taskSet->getPositionErrors();
		double stressTermWeight = 1;
		for(i=0;i<pErr.getSize();i++) {
			if(dynamic_cast<rdCMC_Joint*>(_taskSet->get(i))) {
				rdCMC_Joint *jointTask = dynamic_cast<rdCMC_Joint*>(_taskSet->get(i));
				if(jointTask->getLimit()) {
					double w = rdMath::SigmaDn(jointTask->getLimit() * relativeTau, jointTask->getLimit(), fabs(pErr[i]));
					cout << "Task " << i << ": err=" << pErr[i] << ", limit=" << jointTask->getLimit() << ", sigmoid=" << w << endl;
					stressTermWeight = min(stressTermWeight, w);
				}
			}
		}
		cout << "Setting stress term weight to " << stressTermWeight << " (relativeTau was " << relativeTau << ")" << std::endl;
		realTarget->setStressTermWeight(stressTermWeight);

		for(i=0;i<vErr.getSize();i++) err[i] = vErr[i];
		_stressTermWeightStore->append(tiReal,1,&stressTermWeight);
	}

	// SET BOUNDS ON CONTROLS
	int N = _predictor->getNX();
	Array<double> xmin(0.0,N),xmax(1.0,N);
	for(i=0;i<N;i++) {
		Control *x = _controlSet->get(i);
		xmin[i] = x->getControlValueMin(tiReal);
		xmax[i] = x->getControlValueMax(tiReal);
		// For controls whose constraints are constant min/max values we'll just specify
		// it using the default parameter min/max rather than creating a control curve
		// (using nodes) in the xml.  So we catch this case here.
		if(xmin[i] == rdMath::NAN) xmin[i] = x->getDefaultParameterMin();
		if(xmax[i] == rdMath::NAN) xmax[i] = x->getDefaultParameterMax();
	}

	/*
	// FILTER CONTROL BOUNDS
	ControlSet controlCopyHigh = *_controlSet;
	ControlSet controlCopyLow = *_controlSet;
	double valueMin, valueMax;
	Control *xlow, *xhigh;
	for(i=0;i<N;i++) {
        x = _controlSet->get(i);

		xlow = controlCopyLow.get(i);
		xlow->setControlValue(_tf,-1.0);
		xlow->filter(_tf);
		valueMin = xlow->getControlValue(_tf);
		if(xmin[i]<valueMin) {
			xmin[i] = valueMin;
		}
		//x->setControlValueMin(_tf,valueMin);

		xhigh = controlCopyHigh.get(i);
		xhigh->setControlValue(_tf,1.0);
		xhigh->filter(_tf);
		valueMax = xhigh->getControlValue(_tf);
		if(xmax[i]>valueMax) {
			xmax[i] = valueMax;
		}
		//x->setControlValueMax(_tf,valueMax);

		if(xmin[i]>xmax[i]) {
			xmin[i] = xmax[i];
		}
	}
	*/
	cout<<"\nxmin:\n"<<xmin<<endl;
	cout<<"\nxmax:\n"<<xmax<<endl;

	// ILSE-  For modeling reflexes
	if(_useReflexes) constrainControlsBasedOnReflexes(_tf,xmin,xmax);

	// COMPUTE BOUNDS ON MUSCLE FORCES
	Array<double> zero(0.0,N);
	Array<double> fmin(0.0,N),fmax(0.0,N);
	_predictor->setInitialTime(tiReal);
	_predictor->setFinalTime(tfReal);
	_predictor->setTargetForces(&zero[0]);
	_predictor->evaluate(&xmin[0],&fmin[0]);
	_predictor->evaluate(&xmax[0],&fmax[0]);
	cout<<endl<<endl;
	cout<<"\ntiReal = "<<tiReal<<"  tfReal = "<<tfReal<<endl;
	cout<<"Min forces:\n";
	cout<<fmin<<endl;
	cout<<"Max forces:\n";
	cout<<fmax<<endl;

	// Print actuator force range if range is small
	double range;
	for(i=0;i<N;i++) {
		range = fmax[i] - fmin[i];
		if(range<1.0) {
			try {
				AbstractActuator *actuator = _model->getActuatorSet()->get(i);
				cout<<"WARN- small force range for "<<actuator->getName()<<" ("<<fmin[i]<<" to "<<fmax[i]<<")\n";
			} catch(...) {};
		}
	}
	cout<<endl<<endl;


	// SOLVE STATIC OPTIMIZATION FOR DESIRED ACTUATOR FORCES
	SimTK::Vector lowerBounds(N), upperBounds(N);
	for(i=0;i<N;i++) {
		if(fmin[i]<fmax[i]) {
			lowerBounds[i] = fmin[i];
			upperBounds[i] = fmax[i];
		} else {
			lowerBounds[i] = fmax[i];
			upperBounds[i] = fmin[i];
		}
	}

	_target->setParameterLimits(lowerBounds, upperBounds);

	// OPTIMIZER ERROR TRAP
	_f.setSize(N);
	bool success=false;
	const int optTryLimit=3;
	for(int optTrys=0;!success;optTrys++) {

		if(!_target->prepareToOptimize(&_f[0])) {
			// No direct solution, need to run optimizer
			Vector fVector(N,&_f[0],true);
			success=true;
			try {
				_optimizer->optimize(fVector);
			}
			catch (const SimTK::Exception::Base &ex) {
				cout << ex.getMessage() << endl;
				cout << "OPTIMIZATION FAILED..." << endl;
				success=false;
			}
		} else {
			// Got a direct solution, don't need to run optimizer
			success=true;
		}

		_target->printPerformance(&_f[0]);

		// Solution
		if(success) {
			break;

		// Too many trys
		} else if(optTrys>=optTryLimit) {
			cout<<endl;
			char tmp[1024],msg[1024];
			strcpy(msg,"rdCMC.computeControls:  WARN- The optimizer could not find ");
			sprintf(tmp,"a solution at time = %lf.\n",aT);
			strcat(msg,tmp);
			strcat(msg,"If using the fast target, try using the slow target.\n");
			strcat(msg,"Starting at a slightly different initial time may also help.\n");
			strcat(msg,tmp);
			cout<<"\n"<<msg<<endl<<endl;
			throw(new Exception(msg));

		// Try again
		} else {
			// Alter initial guess by a small amount
			for(i=0;i<_f.getSize();i++) {
				double df = 0.01*(fmax[i]-fmin[i]);
				if(_f[i]>fmin[i]) {
					_f[i] -= df;
				} else {
					_f[i] += df;
				}
				if(_f[i]<fmin[i]) _f[i] = fmin[i];
				if(_f[i]>fmax[i]) _f[i] = fmax[i];
			}
		}
	}

	cout<<"\nDesired actuator forces:\n";
	cout<<_f<<endl;


	// ROOT SOLVE FOR EXCITATIONS
	_predictor->setTargetForces(&_f[0]);
	RootSolver rootSolver(_predictor);
	Array<double> tol(4.0e-3,N);
	Array<double> fErrors(0.0,N);
	Array<double> controls(0.0,N);
	controls = rootSolver.solve(xmin,xmax,tol);
	cout<<"\n\nControls:\n";
	cout<<controls<<endl;
	//_predictor->evaluate(&controls[0],&fErrors[0]);
	//cout<<"\n\nErrors:\n";
	//cout<<fErrors<<endl<<endl;

	// FILTER OSCILLATIONS IN CONTROL VALUES
	if(_useCurvatureFilter) FilterControls(rControlSet,_targetDT,_tf,controls);

	// SET EXCITATIONS
	rControlSet.setControlValues(_tf,&controls[0]);
	//rControlSet.filter(_tf);

	//rControlSet.getControlValues(_tf,&controls[0]);
	//cout<<"\n\nFiltered Controls:\n";
	//cout<<controls<<endl;

	// RETURN MODEL TO ORIGINAL STATES
	//_model->set(aT,&x[0],aY);

	_model->getAnalysisSet()->setOn(true);
}

//_____________________________________________________________________________
/**
 * Set whether or not a curvature filter should be applied to the controls.
 *
 * @param aTrueFalse If true, controls will filtered based on their curvature.
 * If false, they will not be filtered.
 */
void rdCMC::
setUseCurvatureFilter(bool aTrueFalse)
{
	_useCurvatureFilter = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not a curvature filter should be applied to the controls.
 *
 * @return True, if will filtered based on their curvature; false, if
 * they will not be filtered.
 */
bool rdCMC::
getUseCurvatureFilter() const
{
	return(_useCurvatureFilter);
}

//_____________________________________________________________________________
/**
 * Set whether or not to use reflexes to constrain the controls.
 *
 * @param aTrueFalse If true, controls will be constrained based on reflexes.
 * If false, they will not be constrained based on reflexes.
 */
void rdCMC::
setUseReflexes(bool aTrueFalse)
{
	_useReflexes = aTrueFalse;
}
//_____________________________________________________________________________
/**
 * Get whether or not to use reflexes to constrain the controls.
 *
 * @return If true, controls will be constrained based on reflexes.
 * If false, they will not be constrained based on reflexes.
 */
bool rdCMC::
getUseReflexes() const
{
	return(_useReflexes);
}

//_____________________________________________________________________________
/**
 * Constrain the controls based on reflexes.
 *
 * @param t Current time in the simulation in normalized time.
 * @param xmin Minimum allowed values for the controls.
 * @param xmax Maximum allowed values for the controls.
 */
void rdCMC::
constrainControlsBasedOnReflexes(double t,Array<double> &xmin,Array<double> &xmax)
{
	int i,r;

	// NAMES OF MUSCLES THAT HAVE REFLEX-BASED CONSTRAINTS
	Array<string> reflexName("");
	reflexName.append("rect_fem_r.excitation");
	reflexName.append("vas_med_r.excitation");
	int nr = reflexName.getSize();

	// REFLEX THRESHOLDS
	//double gain = 10.0;
	Array<double> threshold(10.0,nr);
	threshold[0] = 0.1;  // RF
	threshold[1] = 0.1;	// VAS MED

	// MAKE REFLEX CONTROL SET IF IT DOESN'T EXIST
	if(_controlConstraintsFromReflexes==NULL) {
		_controlConstraintsFromReflexes = new ControlSet();
		_controlConstraintsFromReflexes->setName("ConstrantsFromReflexes");

		for(i=0;i<nr;i++) {
			ControlLinear *x = new ControlLinear();
			x->setName(reflexName.get(i));
			x->setIsModelControl(true);
			_controlConstraintsFromReflexes->append(x);
		}
	}

	// IDENTIFY WHICH CONTROL IN THE MODEL TO CONSTRAIN
	int nx = _model->getNumControls();
	Array<int> controlIndex(-1,nr);
	for(r=0;r<nr;r++) {
		string name = reflexName.get(r);
		for(i=0;i<nx;i++) {
			if(name == _model->getControlName(i)) {
				cout<<"Found control "<<i<<" for reflex "<<name<<endl;
				controlIndex[r] = i;
			} else {
				cout<<"Did not find a control for reflex "<<name<<endl;
			}
		}
	}

	// LOOP THROUGH THE REFLEXES
	static double controlgain = 10.0;
	double speed;
	for(r=0;r<nr;r++) {

		// Get the speed of the actuator
		// NOTE- This assumes only 1 control for each actuator!!
		try {
			AbstractActuator *actuator = _model->getActuatorSet()->get(controlIndex[r]);
			if(actuator!=NULL) speed = actuator->getSpeed();
		} catch(...) {};


		// Decide on excitation constraints
		if(speed>threshold[r]) {
			i = controlIndex[r];
			xmin[i] = xmin[i] + controlgain * (speed - threshold[r]);
			if(xmin[i]>xmax[i]) xmin[1] = xmax[i];
		}

		// RECORD REFLEX-BASED CONSTRAINT
		ControlLinear *xr = (ControlLinear*)_controlConstraintsFromReflexes->get(r);
		xr->setControlValueMin(t,xmin[i]);
		xr->setControlValueMax(t,xmax[i]);
	}
}


//_____________________________________________________________________________
/**
 * Filter the controls.  This method was introduced as a means of attempting
 * to reduce the sizes of residuals.  Unfortunately, the approach was
 * generally unsuccessful because the desired accelerations were not
 * achieved.
 *
 * @param aControlSet Set of controls computed by CMC.
 * @param aDT Current time window.
 * @param aT Current time
 * @param rControls Array of filtered controls.
 */
void rdCMC::
FilterControls(const ControlSet &aControlSet,double aDT,double aT,
			   Array<double> &rControls)
{
	if(aDT <= rdMath::ZERO) {
		cout<<"\nrdCMC.filterControls: aDT is practically 0.0, skipping!\n\n";
		return;
	}

	cout<<"\n\nFiltering controls to limit curvature...\n";

	int i;
	int size = rControls.getSize();
	Array<double> x0(0.0,size),x1(0.0,size),x2(0.0,size);

	// SET TIMES
	double t0,t1,t2;
	t2 = aT;
	t1 = aT - aDT;
	t0 = t1 - aDT;

	// SET CONTROL VALUES
	x2 = rControls;
	aControlSet.getControlValues(t1,x1);
	aControlSet.getControlValues(t0,x0);

	// LOOP OVER CONTROLS
	double m1,m2;
	double curvature;
	double thresholdCurvature = 2.0 * 0.05 / (aDT * aDT);
	
	//double thresholdSlopeDiff = 0.2 / aDT;
	for(i=0;i<size;i++) {
		m2 = (x2[i]-x1[i]) / aDT;
		m1 = (x1[i]-x0[i]) / aDT;

				
		curvature = (m2 - m1) / aDT;
		curvature = fabs(curvature);

//		cout<<"thresholdCurvature="<<thresholdCurvature<<"  curvature="<<curvature<<endl;
		if(curvature<=thresholdCurvature) continue;
	
//		diff = fabs(m2) - fabs(m1);
//		cout<<"thresholdSlopeDiff="<<thresholdSlopeDiff<<"  slopeDiff="<<diff<<endl;
//		if(diff>thresholdSlopeDiff) continue;
		
        

		// ALTER CONTROL VALUE
		rControls[i] = (3.0*x2[i] + 2.0*x1[i] + x0[i]) / 6.0;

		// PRINT
		cout<<aControlSet[i]->getName()<<": old="<<x2[i]<<" new="<<rControls[i]<<endl;
	}

	cout<<endl<<endl;
}


