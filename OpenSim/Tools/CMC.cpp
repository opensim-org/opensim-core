// CMC.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <iostream>
#include <string>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/RootSolver.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "VectorFunctionForActuators.h"
#include <OpenSim/Simulation/Model/OpenSimForceSubsystem.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Common/OptimizationTarget.h>
#include <simmath/Optimizer.h>
#include "CMC.h"
#include  <OpenSim/Tools/CMC_Point.h>
#include <OpenSim/Tools/CMC_Joint.h>
#include <OpenSim/Tools/CMC_TaskSet.h>
#include <OpenSim/Tools/ActuatorForceTarget.h>
#include <OpenSim/Tools/ForwardTool.h>
#include "SimTKcommon.h" 
#include "MuscleStateTrackingTask.h"

using namespace std;
using SimTK::Vector;
using namespace OpenSim;
using namespace SimTK;


// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class ComputeControlsEventHandler : public PeriodicEventHandler {
public:
    ComputeControlsEventHandler( CMC *controller) :
        PeriodicEventHandler(Stage::Time),
        _controller( controller ) {
    }

    void handleEvent (SimTK::State& s, Real accuracy, const Vector& yWeigths, const Vector&  constraintTols, Stage& lowestModified, bool& terminate)     const {
        lowestModified = Stage::Velocity;
        terminate = false;
        _controller->computeControls( s, _controller->updControlSet() );
        _controller->setTargetTime(s.getTime() + _controller->getTargetDT());
    }

    Real getNextEventTime( const State& s, bool includeCurrent) const {

        if( _controller->getCheckTargetTime() ) {
            return( _controller->getTargetTime() );
        } else {
            return( SimTK::Infinity );
        }
    }
   CMC* _controller;
};
/// @endcond


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
/**
 * Default Constructor.
 */
CMC::CMC() :
    TrackingController(),
    _paramList(-1) , 
    _f(0.0),
    _controlSet()
{
    setNull();
    setupProperties();

}
/**
 *  Copy constructor.
 */
CMC::CMC(const CMC &aController) :
    TrackingController(aController),
    _controlSet()
{
    setNull();
    setupProperties();
    copyData(aController);
}

//_____________________________________________________________________________
/**
 * Contructor
 *
 * @param aModel Model that is to be controlled.
 * @param aTaskSet Set of tracking tasks.
 */
CMC::CMC(Model *aModel,CMC_TaskSet *aTaskSet) :
	TrackingController(*aModel), _paramList(-1) , _f(0.0)
{
	// NULL
	setNull();

	// TRACK OBJECTS
	_taskSet = aTaskSet;
	if(_taskSet==NULL) {
		char tmp[Object::NAME_LENGTH];
		strcpy(tmp,"CMC.CMC: ERR- no ");
		strcat(tmp,"track objects.\n");
		throw(new Exception(tmp,__FILE__,__LINE__));
	}
	//TrackingTask* dTask = new MuscleStateTrackingTask();
	//_taskSet->append(dTask);
	//_taskSet->print("MyTasks.xml");
	// STORAGE
	Array<string> labels;
	labels.append("time");
	for(int i=0;i<_taskSet->getSize();i++) {
		for(int j=0;j<_taskSet->get(i).getNumTaskFunctions();j++) {
			labels.append(_taskSet->get(i).getName());
		}
	}
	_pErrStore = new Storage(1000,"PositionErrors");
	_pErrStore->setColumnLabels(labels);
	_vErrStore = new Storage(1000,"VelocityErrors");
	_pErrStore->setColumnLabels(labels);
	_stressTermWeightStore = new Storage(1000,"StressTermWeight");
}
//_____________________________________________________________________________
/**
 * Copy this CMC controller and return a pointer to the copy.
 * The copy constructor for this class is used.  This method is called
 * when a description of this controller is read in from an XML file.
 *
 * @return Pointer to a copy of this CMC controller.
 */
Object* CMC::copy() const
{
	CMC *object = new CMC(*this);
	return object;
}
void CMC::copyData( const CMC &aCmc ) 
{
   TrackingController::copyData( aCmc );
   _dt                    = aCmc._dt;
   _tf                    = aCmc._tf;
   _lastDT                = aCmc._lastDT;
   _restoreDT             = aCmc._restoreDT;
   _checkTargetTime       = aCmc._checkTargetTime;
   _pErrStore             = aCmc._pErrStore;
   _vErrStore             = aCmc._vErrStore;
   _stressTermWeightStore = aCmc._stressTermWeightStore;
   _controlSet            = aCmc._controlSet;
   _taskSet               = aCmc._taskSet;
   _paramList             = aCmc._paramList;
   _verbose               = aCmc._verbose;
   _predictor             = aCmc._predictor;
   _f                     = aCmc._f;
   _taskSet               = aCmc._taskSet;

}
//_____________________________________________________________________________
/**
 * Constructor from an XML Document
 */
//
CMC::CMC( const std::string &aFileName, bool aUpdateFromXMLNode) :
    TrackingController(aFileName, false)
{
    setNull();
    setupProperties();
    if(aUpdateFromXMLNode) updateFromXMLNode();
}


//_____________________________________________________________________________
/**
 * Destructor.
 */
CMC::~CMC()
{
	delete _optimizer;
}

//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void CMC::
setNull()
{
    setType("CMC");
	_optimizer = NULL;
	_target = NULL;
	_taskSet = NULL;
	_dt = 0.0;
	_lastDT = 0.0;
	_restoreDT = false;
	_tf = 1.0e12;
	_targetDT = 1.0e-3;
	_checkTargetTime = false;
	_pErrStore = NULL;
	_vErrStore = NULL;
	_stressTermWeightStore = NULL;
    _useCurvatureFilter = false;
	_verbose = false;
	_paramList.setSize(0);
    _controlSet.setSize(0);

}
void CMC::setupProperties()
{


}

CMC& CMC::
operator=(const CMC &aCmc)
{
    // BASE CLASS
    TrackingController::operator=(aCmc);

     _controlSet          = aCmc._controlSet;
	 return(*this);

}

//=============================================================================
// GET AND SET
//=============================================================================
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
OpenSim::Array<int>* CMC::
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
OptimizationTarget* CMC::
setOptimizationTarget(OptimizationTarget *aTarget, SimTK::Optimizer *aOptimizer)
{
	// PREVIOUS TARGET
	OptimizationTarget *prev = _target;

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
OptimizationTarget* CMC::
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
SimTK::Optimizer* CMC::
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
void CMC::
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
double CMC::
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
void CMC::
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
double CMC::
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
void CMC::
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
double CMC::
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
void CMC::
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
bool CMC::
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
void CMC::
setActuatorForcePredictor(VectorFunctionForActuators *aPredictor)
{
	_predictor = aPredictor;
}
//_____________________________________________________________________________
/**
 * Get the predictor for actuator forces.
 */
VectorFunctionForActuators* CMC::
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
Storage* CMC::
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
Storage* CMC::
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
Storage* CMC::
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
 * @param s Initial states.
 */
void CMC::
computeInitialStates(SimTK::State& s, double &rTI)
{
	
	int i,j;

	int N = _predictor->getNX();
	SimTK::State initialState = s;
	Array<double> xmin(0.01,N),forces(0.0,N);
	double normConstant = _model->getTimeNormConstant();
	double tiReal = rTI * normConstant;
    if( _verbose ) {
        cout<<"\n\n=============================================\n";
        cout<<"enter CMC.computeInitialStates: ti="<< rTI << "  q's=" << s.getQ() <<endl;
        cout<<"\nenter CMC.computeInitialStates: ti="<< rTI << "  u's=" << s.getU() <<endl;
        cout<<"\nenter CMC.computeInitialStates: ti="<< rTI << "  z's=" << s.getZ() <<endl;
        cout<<"=============================================\n";
    }


	// TURN ANALYSES OFF
	_model->updAnalysisSet().setOn(false);

	// CONSTRUCT CONTROL SET
	ControlSet xiSet;

	for(i=0;i<_numControls;i++) {
		ControlConstant *x = new ControlConstant();
        x->setName(_controlSet.get(i).getName());
        x->setIsModelControl(true);
		// This is not a very good way to set the bounds on the controls because ConrtolConstant only supports constant
		// min/max bounds but we may have time-dependent min/max curves specified in the controls constraints file
		//
	    Control& xPredictor = _controlSet.get(i);
		x->setDefaultParameterMin(xPredictor.getDefaultParameterMin());
		x->setDefaultParameterMax(xPredictor.getDefaultParameterMax());
		double xmin = xPredictor.getControlValueMin(tiReal);
		if(!SimTK::isNaN(xmin)) x->setControlValueMin(tiReal,xmin);
		double xmax = xPredictor.getControlValueMax(tiReal);
		if(!SimTK::isNaN(xmax)) x->setControlValueMax(tiReal,xmax);
		xiSet.append(x);
	}

	// ACTUATOR EQUILIBRIUM
	// 1
	//
    // perform integration but reset the coords and speeds so only actuator
    // states at changed

	obtainActuatorEquilibrium(s,tiReal,0.200,xmin,true);
    if( _verbose ) {
        cout<<"\n\n=============================================\n";
        cout<<"#1 act Equ.  CMC.computeInitialStates: ti="<< rTI << "  q's=" << s.getQ() <<endl;
        cout<<"\n#1 act Equ.  CMC.computeInitialStates: ti="<< rTI << "  u's=" << s.getU() <<endl;
        cout<<"\n#1 act Equ.  CMC.computeInitialStates: ti="<< rTI << "  z's=" << s.getZ() <<endl;
        cout<<"=============================================\n";
    }
	restoreConfiguration( s, initialState ); // set internal coord,speeds to initial vals. 

	// 2
	obtainActuatorEquilibrium(s,tiReal,0.200,xmin,true);
    if( _verbose ) {
        cout<<"\n\n=============================================\n";
        cout<<"#2 act Equ.  CMC.computeInitialStates: ti="<< rTI << "  q's=" << s.getQ() <<endl;
        cout<<"\n#2 act Equ.  CMC.computeInitialStates: ti="<< rTI << "  u's=" << s.getU() <<endl;
        cout<<"\n#2 act Equ.  CMC.computeInitialStates: ti="<< rTI << "  z's=" << s.getZ() <<endl;
        cout<<"=============================================\n";
    }
	restoreConfiguration( s, initialState );

	// CHANGE THE TARGET DT ON THE CONTROLLER TEMPORARILY
	double oldTargetDT = getTargetDT();
	double newTargetDT = 0.030;
	setTargetDT(newTargetDT);

	// REPEATEDLY CONTROL OVER THE FIRST TIME STEP
	Array<double> xi(0.0,_numControls);
	for(i=0;i<2;i++) {

		// CLEAR ANY PREVIOUS CONTROL NODES
		for(j=0;j<_controlSet.getSize();j++) {
            ControlLinear& control = (ControlLinear&)_controlSet.get(j);
			control.clearControlNodes();
		}

		// COMPUTE CONTROLS

        s.updTime() = rTI;
		computeControls( s, xiSet);
		_model->updAnalysisSet().setOn(false);

		// GET CONTROLS
		xiSet.getControlValues(rTI,xi);

		// OBTAIN EQUILIBRIUM
		if(i<1) {

			obtainActuatorEquilibrium(s,tiReal,0.200,xi,true);
			restoreConfiguration(s, initialState );
		}
	}

	// GET NEW STATES
	_predictor->evaluate(s, &xi[0], &forces[0]);
	
	rTI += newTargetDT;

	// CLEANUP
	setTargetDT(oldTargetDT);
	_model->updAnalysisSet().setOn(true);
    if( _verbose ) {
        cout<<"\n\n=============================================\n";
        cout<<"finish CMC.computeInitialStates: ti="<< rTI << "  q's=" << s.getQ() <<endl;
        cout<<"\nfinish CMC.computeInitialStates: ti="<< rTI << "  u's=" << s.getU() <<endl;
        cout<<"\nfinish CMC.computeInitialStates: ti="<< rTI << "  z's=" << s.getZ() <<endl;
        cout<<"=============================================\n";
    }
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
void CMC::
obtainActuatorEquilibrium(SimTK::State& s, double tiReal,double dtReal,
								  const OpenSim::Array<double> &x,bool hold)
{
	// HOLD COORDINATES
	if(hold) {
		_predictor->getCMCActSubsys()->holdCoordinatesConstant(tiReal);
	} else {
		_predictor->getCMCActSubsys()->releaseCoordinates();
	}

	// INITIALIZE
	_predictor->setInitialTime(tiReal);
	_predictor->setFinalTime(tiReal+dtReal);
    _predictor->getCMCActSubsys()->setCompleteState( s );

	// INTEGRATE FORWARD
	Array<double> f(0.0,x.getSize());

	_predictor->evaluate(s, &x[0], &f[0]);
	// update the muscle states
	_model->getForceSubsystem().updZ(s) = 
	_model->getForceSubsystem().getZ(_predictor->getCMCActSubsys()->getCompleteState());

	// RELEASE COORDINATES
	 _predictor->getCMCActSubsys()->releaseCoordinates();
}
//_____________________________________________________________________________
/**
 * A utility method used to restore the coordinates and speeds to initial
 * values.  The states associated with actuators are not changed.
 *
 * @param nqnu Sum of the number of coordinates and speeds (nq + nu).
 * @param s current state.
 * @param initialState initial state to be restored.
 */
void CMC::
restoreConfiguration(SimTK::State& s, const SimTK::State& initialState)
{   
	_model->getMatterSubsystem().updQ(s) = _model->getMatterSubsystem().getQ(initialState);
	_model->getMatterSubsystem().updU(s) = _model->getMatterSubsystem().getU(initialState);
}



//_____________________________________________________________________________
/**
 * Compute the controls for a simulation.
 *
 * The caller should send in an initial guess.
 *
 * @param s state of system model
 * @param rDT Integration time step in normalized time that is to be taken
 * next.  Note that the controller can change the value of rDT.
 * @param aT Current time in normalized time.
 * @param rControlSet Control set used for the simulation.  This method
 * alters the control set in order to control the simulation.
 */
void CMC::
computeControls(SimTK::State& s, ControlSet &controlSet)
{
	// CONTROLS SHOULD BE RECOMPUTED- NEED A NEW TARGET TIME
	_tf = s.getTime() + _targetDT;

	int i,j;

	// TURN ANALYSES OFF
	_model->updAnalysisSet().setOn(false);

	// TIME STUFF
	double normConstant = _model->getTimeNormConstant();
	double tiReal = s.getTime() * normConstant;
	double tfReal = _tf * normConstant;

    cout<<"CMC.computeControls:  t = "<<s.getTime()<<endl;
    if(_verbose) { 
        cout<<"\n\n----------------------------------\n";
        cout<<"integration step size = "<<_targetDT<<",  target time = "<<_tf<<endl;
    }

	// SET CORRECTIONS 
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	FunctionSet *qSet = _predictor->getCMCActSubsys()->getCoordinateTrajectories();
	FunctionSet *uSet = _predictor->getCMCActSubsys()->getSpeedTrajectories();
	Array<double> qDesired(0.0,nq),uDesired(0.0,nu);
	qSet->evaluate(qDesired,0,tiReal);
	if(uSet!=NULL) {
		uSet->evaluate(uDesired,0,tiReal);
	} else {
		qSet->evaluate(uDesired,1,tiReal);
	}
	Array<double> qCorrection(0.0,nq),uCorrection(0.0,nu);
       const Vector& q = s.getQ();
       const Vector& u = s.getU();

	for(i=0;i<nq;i++) qCorrection[i] = q[i] - qDesired[i];
	for(i=0;i<nu;i++) uCorrection[i] = u[i] - uDesired[i];

	_predictor->getCMCActSubsys()->setCoordinateCorrections(&qCorrection[0]);
	_predictor->getCMCActSubsys()->setSpeedCorrections(&uCorrection[0]);

    if( _verbose ) {
        cout << "\n=============================" << endl;
        cout << "\nCMC:computeControls"  << endl;
        cout << "\nq's = " << s.getQ() << endl;
        cout << "\nu's = " << s.getU() << endl;
        cout << "\nz's = " << s.getZ() << endl;
        cout<<"\nqDesired:"<<qDesired << endl;
        cout<<"\nuDesired:"<<uDesired << endl;
        cout<<"\nQCorrections:"<<qCorrection << endl;
        cout<<"\nUCorrections:"<<uCorrection << endl;
    }

	// ERRORS
	_taskSet->computeErrors(s, tiReal);
	_taskSet->recordErrorsAsLastErrors();
	Array<double> &pErr = _taskSet->getPositionErrors();
	Array<double> &vErr = _taskSet->getVelocityErrors();
	if(_verbose) cout<<"\nErrors at time "<<s.getTime()<<":"<<endl;
	int e=0;
	for(i=0;i<_taskSet->getSize();i++) {
		
		TrackingTask& task = _taskSet->get(i);

		if(_verbose) {
			for(j=0;j<task.getNumTaskFunctions();j++) {
				cout<<task.getName()<<":  ";
				cout<<"pErr="<<pErr[e]<<" vErr="<<vErr[e]<<endl;
				e++;
			}
		}
	}

	double *err = new double[pErr.getSize()];
	for(i=0;i<pErr.getSize();i++) err[i] = pErr[i];
	_pErrStore->append(tiReal,pErr.getSize(),err);
	for(i=0;i<vErr.getSize();i++) err[i] = vErr[i];
	_vErrStore->append(tiReal,vErr.getSize(),err);

	
	// COMPUTE DESIRED ACCELERATIONS
	_taskSet->computeDesiredAccelerations(s, tiReal,tfReal);

	// Set the weight of the stress term in the optimization target based on this sigmoid-function-blending
	// Note that if no task limits are set then by default the weight will be 1.
	// TODO: clean this up -- currently using dynamic_casts to make sure we're not using fast target, etc.
	if(dynamic_cast<ActuatorForceTarget*>(_target)) {
		double relativeTau = 0.1;
		ActuatorForceTarget *realTarget = dynamic_cast<ActuatorForceTarget*>(_target);
	
		Array<double> &pErr = _taskSet->getPositionErrors();
		double stressTermWeight = 1;
		for(i=0;i<_taskSet->getSize();i++) {
			if(dynamic_cast<CMC_Joint*>(&_taskSet->get(i))) {
				CMC_Joint& jointTask = dynamic_cast<CMC_Joint&>(_taskSet->get(i));
				if(jointTask.getLimit()) {
					double w = ForwardTool::SigmaDn(jointTask.getLimit() * relativeTau, jointTask.getLimit(), fabs(pErr[i]));
					if(_verbose) cout << "Task " << i << ": err=" << pErr[i] << ", limit=" << jointTask.getLimit() << ", sigmoid=" << w << endl;
					stressTermWeight = min(stressTermWeight, w);
				}
			}
		}
		if(_verbose) cout << "Setting stress term weight to " << stressTermWeight << " (relativeTau was " << relativeTau << ")" << std::endl;
		realTarget->setStressTermWeight(stressTermWeight);

		for(i=0;i<vErr.getSize();i++) err[i] = vErr[i];
		_stressTermWeightStore->append(tiReal,1,&stressTermWeight);
	}

	// SET BOUNDS ON CONTROLS
	int N = _predictor->getNX();
	Array<double> xmin(0.0,N),xmax(1.0,N);
	for(i=0;i<N;i++) {
		Control& x = controlSet.get(i);
		xmin[i] = x.getControlValueMin(tiReal);
		xmax[i] = x.getControlValueMax(tiReal);
		// For controls whose constraints are constant min/max values we'll just specify
		// it using the default parameter min/max rather than creating a control curve
		// (using nodes) in the xml.  So we catch this case here.
        if(SimTK::isNaN(xmin[i])) xmin[i] = x.getDefaultParameterMin();
	    if(SimTK::isNaN(xmax[i])) xmax[i] = x.getDefaultParameterMax();
	}
	
	if(_verbose) {
		cout<<"\nxmin:\n"<<xmin<<endl;
		cout<<"\nxmax:\n"<<xmax<<endl;
	}

	// COMPUTE BOUNDS ON MUSCLE FORCES
	Array<double> zero(0.0,N);
	Array<double> fmin(0.0,N),fmax(0.0,N);
	_predictor->setInitialTime(tiReal);
	_predictor->setFinalTime(tfReal);
	_predictor->setTargetForces(&zero[0]);
	_predictor->evaluate(s, &xmin[0], &fmin[0]);
	_predictor->evaluate(s, &xmax[0], &fmax[0]);
     SimTK::State newState = _predictor->getCMCActSubsys()->getCompleteState();
	if(_verbose) {
		cout<<endl<<endl;
		cout<<"\ntiReal = "<<tiReal<<"  tfReal = "<<tfReal<<endl;
		cout<<"Min forces:\n";
		cout<<fmin<<endl;
		cout<<"Max forces:\n";
		cout<<fmax<<endl;
	}

	// Print actuator force range if range is small
	double range;
	for(i=0;i<N;i++) {
		range = fmax[i] - fmin[i];
		if(range<1.0) {
			try {
                cout<<"WARN- small force range for "<<_model->getActuators().get(i).getName()<<" ("<<fmin[i]<<" to "<<fmax[i]<<")\n";
			} catch(...) {};
		}
	}


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

	if(!_target->prepareToOptimize(newState, &_f[0])) {
		// No direct solution, need to run optimizer
		Vector fVector(N,&_f[0],true);

		try {
			_optimizer->optimize(fVector);
		}
		catch (const SimTK::Exception::Base &ex) {
			cout << ex.getMessage() << endl;
			cout << "OPTIMIZATION FAILED..." << endl;
			cout<<endl;
			char tmp[1024],msg[1024];
			strcpy(msg,"CMC.computeControls:  WARN- The optimizer could not find ");
			sprintf(tmp,"a solution at time = %lf.\n",s.getTime() );
			strcat(msg,tmp);
			strcat(msg,"If using the fast target, try using the slow target.\n");
			strcat(msg,"Starting at a slightly different initial time may also help.\n");
			
			cout<<"\n"<<msg<<endl<<endl;
			string msgString = string(msg);
         throw(new OpenSim::Exception(msg, __FILE__,__LINE__));
		}
	} else {
		// Got a direct solution, don't need to run optimizer
	}

	if(_verbose) _target->printPerformance(&_f[0]);

	if(_verbose) {
		cout<<"\nDesired actuator forces:\n";
		cout<<_f<<endl;
	}


	// ROOT SOLVE FOR EXCITATIONS
	_predictor->setTargetForces(&_f[0]);
	RootSolver rootSolver(_predictor);
	Array<double> tol(4.0e-3,N);
	Array<double> fErrors(0.0,N);
	Array<double> controls(0.0,N);
	controls = rootSolver.solve(s, xmin,xmax,tol);
	if(_verbose) {
       cout<<"\n\nXXX t=" << _tf << "   Controls:" <<controls<<endl;
	}
	
	// FILTER OSCILLATIONS IN CONTROL VALUES
	if(_useCurvatureFilter) FilterControls(s, controlSet,_targetDT,controls,_verbose);

	// SET EXCITATIONS
	controlSet.setControlValues(_tf,&controls[0]);

	_model->updAnalysisSet().setOn(true);
}

//_____________________________________________________________________________
/**
 * Set whether or not a curvature filter should be applied to the controls.
 *
 * @param aTrueFalse If true, controls will filtered based on their curvature.
 * If false, they will not be filtered.
 */
void CMC::
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
bool CMC::
getUseCurvatureFilter() const
{
	return(_useCurvatureFilter);
}

const CMC_TaskSet& CMC::getTaskSet() const{
   return( *_taskSet );
}

CMC_TaskSet& CMC::updTaskSet() const {
   return( *_taskSet );
}
//_____________________________________________________________________________
/**
 * Set whether or not to use verbose printing.
 *
 * @param aTrueFalse If true, print verbose information.
 * If false, print only critical information.
 */
void CMC::
setUseVerbosePrinting(bool aTrueFalse)
{
	_verbose = aTrueFalse;
}
bool CMC::
getUseVerbosePrinting() const
{
	return(_verbose);
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
 * @param rControls Array of filtered controls.
 */
void CMC::
FilterControls(const SimTK::State& s, const ControlSet &aControlSet,double aDT,
			   OpenSim::Array<double> &rControls,bool aVerbosePrinting)
{
	if(aDT <= SimTK::Zero) {
		if(aVerbosePrinting) cout<<"\nCMC.filterControls: aDT is practically 0.0, skipping!\n\n";
		return;
	}

	if(aVerbosePrinting) cout<<"\n\nFiltering controls to limit curvature...\n";

	int i;
	int size = rControls.getSize();
	Array<double> x0(0.0,size),x1(0.0,size),x2(0.0,size);

	// SET TIMES
	double t0,t1,t2;
	t2 = s.getTime();
	t1 = s.getTime() - aDT;
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

		if(curvature<=thresholdCurvature) continue;
	
//		diff = fabs(m2) - fabs(m1);
//		cout<<"thresholdSlopeDiff="<<thresholdSlopeDiff<<"  slopeDiff="<<diff<<endl;
//		if(diff>thresholdSlopeDiff) continue;
		
        

		// ALTER CONTROL VALUE
		rControls[i] = (3.0*x2[i] + 2.0*x1[i] + x0[i]) / 6.0;

		// PRINT
		if(aVerbosePrinting) cout<<aControlSet[i].getName()<<": old="<<x2[i]<<" new="<<rControls[i]<<endl;
	}

	if(aVerbosePrinting) cout<<endl<<endl;
}


double CMC::computeControl(const SimTK::State& s, int index) const {
    SimTK_ASSERT( index < _controlSet.getSize(),
   "CMC::computeControl:  index > size of actuator set" );

   return(_controlSet.get(index).getControlValue( s.getTime()) ) ;
}

void CMC::setActuators( Set<Actuator>& actSet )  {

    _controlSet.setName(_model->getName());

	_controlSet.setSize(0);
    for(int i=0; i<actSet.getSize(); i++ ) {
        Actuator& act = actSet.get(i);
        act.setController(this);
        act.setControlIndex(i);
        act.setIsControlled(true);

        ControlLinear *control = new ControlLinear();
        control->setName(act.getName() + ".excitation" );
        _controlSet.append(control);
    }
    _numControls = _controlSet.getSize();

}
// for any post XML deserialization intialization
void CMC::setup(Model& model)   {

	TrackingController::setup(model);

	// STORAGE
	Array<string> labels;
	labels.append("time");
	for(int i=0;i<_taskSet->getSize();i++) {
		for(int j=0;j<_taskSet->get(i).getNumTaskFunctions();j++) {
			labels.append(_taskSet->get(i).getName());
		}
	}
	_pErrStore = new Storage(1000,"PositionErrors");
	_pErrStore->setColumnLabels(labels);
	_vErrStore = new Storage(1000,"VelocityErrors");
	_pErrStore->setColumnLabels(labels);
	_stressTermWeightStore = new Storage(1000,"StressTermWeight");

}
// for adding any components to the model
void CMC::createSystem( SimTK::MultibodySystem& system)  const
{
     // add event handler for updating controls for next window 
	 CMC* mutableThis = const_cast<CMC *>(this);
     ComputeControlsEventHandler* computeControlsHandler = new ComputeControlsEventHandler(mutableThis);

     system.updDefaultSubsystem().addEventHandler(computeControlsHandler );
}

// for any intialization requiring a state or the complete system 
void CMC::initState( SimTK::State& s) 
{
   
}
