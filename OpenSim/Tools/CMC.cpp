/* -------------------------------------------------------------------------- *
 *                             OpenSim:  CMC.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "CMC.h"
#include "VectorFunctionForActuators.h"
#include <OpenSim/Common/RootSolver.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Tools/CMC_Joint.h>
#include <OpenSim/Tools/CMC_TaskSet.h>
#include <OpenSim/Tools/ActuatorForceTarget.h>
#include <OpenSim/Tools/ForwardTool.h>
#include <OpenSim/Simulation/Model/CMCActuatorSubsystem.h>
#include <OpenSim/Simulation/Model/Model.h>

using namespace std;
using SimTK::Vector;
using namespace OpenSim;
using namespace SimTK;

#define MIN_CMC_CONTROL_VALUE 0.02
#define MAX_CMC_CONTROL_VALUE 1.00

#define MAX_CONTROLS_FOR_RRA 10000
// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class ComputeControlsEventHandler : public PeriodicEventHandler {
public:
    ComputeControlsEventHandler( CMC *controller) :
        PeriodicEventHandler(Stage::Time),
        _controller( controller ) {
    }

    void handleEvent (SimTK::State& s, Real accuracy, bool& terminate) const override {
        terminate = false;
        _controller->computeControls( s, _controller->updControlSet() );
        _controller->setTargetTime(s.getTime() + _controller->getTargetDT());
    }

    Real getNextEventTime( const State& s, bool includeCurrent) const override {

        if( _controller->getCheckTargetTime() ) {
            return( _controller->getTargetTime() );
        } else {
            return( std::numeric_limits<SimTK::Real>::infinity() );
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
    _controlSet(),
    _paramList(-1),
    _f(0.0)
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
 * Constructor
 *
 * @param aModel Model that is to be controlled.
 * @param aTaskSet Set of tracking tasks.
 */
CMC::CMC(Model *aModel,CMC_TaskSet *aTaskSet) :
    _paramList(-1) , _f(0.0)
{
    // NULL
    setNull();

    // TRACK OBJECTS
    _taskSet = aTaskSet;
    if(_taskSet==NULL) {
        std::string msg="CMC.CMC: ERR- no track objects.\n";
        throw(new Exception(msg));
    }
    // STORAGE
    Array<string> labels;
    labels.append("time");
    for(int i=0;i<_taskSet->getSize();i++) {
        for(int j=0;j<_taskSet->get(i).getNumTaskFunctions();j++) {
            labels.append(_taskSet->get(i).getName());
        }
    }
    _pErrStore.reset(new Storage(1000,"PositionErrors"));
    _pErrStore->setColumnLabels(labels);
    _vErrStore.reset(new Storage(1000,"VelocityErrors"));
    _pErrStore->setColumnLabels(labels);
    _stressTermWeightStore.reset(new Storage(1000,"StressTermWeight"));
}

void CMC::copyData( const CMC &aCmc ) 
{
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
    TrackingController()
{
    setNull();
    setupProperties();
    if(aUpdateFromXMLNode) updateFromXMLDocument();
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
    _optimizer = NULL;
    _target = NULL;
    _taskSet = NULL;
    _dt = 0.0;
    _lastDT = 0.0;
    _restoreDT = false;
    _tf = 1.0e12;
    _targetDT = 1.0e-3;
    _checkTargetTime = false;
    _pErrStore.reset();
    _vErrStore.reset();
    _stressTermWeightStore.reset();
    _useCurvatureFilter = false;
    _verbose = false;
    _paramList.setSize(0);
    _controlSet.setSize(0);
    setAuthors("Frank Anderson");

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
 * @param aTargetTime Time in the future for which the controls have been
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
 * @return Time in the future for which the controls have been
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
    return(_pErrStore.get());
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
    return(_vErrStore.get());
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
    return(_stressTermWeightStore.get());
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
 * solving for a desired set of actuator forces, and then letting the states
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

    double tiReal = rTI;
    if( _verbose ) {
        log_info("-------------------------------------------");
        log_info("CMC::computeInitialStates, guess (ti = {}):", rTI);
        log_info("-------------------------------------------");
        log_info(" -- Q = {}", s.getQ());
        log_info(" -- U = {}", s.getU());
        log_info(" -- Z = {}", s.getZ());
        log_info("-------------------------------------------");
        log_info("");
    }


    // TURN ANALYSES OFF
    _model->updAnalysisSet().setOn(false);

    // CONSTRUCT CONTROL SET
    ControlSet xiSet;

    for(i=0;i< getNumControls();i++) {
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
        xiSet.adoptAndAppend(x);
    }

    // ACTUATOR EQUILIBRIUM
    // 1
    //
    // perform integration but reset the coords and speeds so only actuator
    // states at changed

    obtainActuatorEquilibrium(s,tiReal,0.200,xmin,true);
    if( _verbose ) {
        log_info("------------------------------------------------------------");
        log_info("CMC::computeInitialStates, actuator equilibrium #1 (ti = {}):", rTI);
        log_info("------------------------------------------------------------");
        log_info(" -- Q = {}", s.getQ());
        log_info(" -- U = {}", s.getU());
        log_info(" -- Z = {}", s.getZ());
        log_info("------------------------------------------------------------");
        log_info("");
    }
    restoreConfiguration( s, initialState ); // set internal coord,speeds to initial vals. 

    // 2
    obtainActuatorEquilibrium(s,tiReal,0.200,xmin,true);
    if( _verbose ) {
        log_info("------------------------------------------------------------");
        log_info("CMC::computeInitialStates, actuator equilibrium #2 (ti = {}):", rTI);
        log_info("------------------------------------------------------------");
        log_info(" -- Q = {}", s.getQ());
        log_info(" -- U = {}", s.getU());
        log_info(" -- Z = {}", s.getZ());
        log_info("------------------------------------------------------------");
        log_info("");
    }
    restoreConfiguration( s, initialState );

    // CHANGE THE TARGET DT ON THE CONTROLLER TEMPORARILY
    double oldTargetDT = getTargetDT();
    double newTargetDT = 0.030;
    setTargetDT(newTargetDT);

    // REPEATEDLY CONTROL OVER THE FIRST TIME STEP
    Array<double> xi(0.0, getNumControls());
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
        log_info("-------------------------------------------");
        log_info("CMC::computeInitialStates, final (ti = {}):", rTI);
        log_info("-------------------------------------------");
        log_info(" -- Q = {}", s.getQ());
        log_info(" -- U = {}", s.getU());
        log_info(" -- Z = {}", s.getZ());
        log_info("-------------------------------------------");
        log_info("");
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
    //_model->getForceSubsystem().updZ(s) = 
    //_model->getForceSubsystem().getZ(_predictor->getCMCActSubsys()->getCompleteState());
    _model->updMultibodySystem().updDefaultSubsystem().updZ(s) = 
        _model->getMultibodySystem().getDefaultSubsystem().getZ(_predictor->getCMCActSubsys()->getCompleteState());

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
 * This method alters the control set in order to control the simulation.
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
    double tiReal = s.getTime(); 
    double tfReal = _tf; 

    log_info("CMC::computeControls, t = {}", tiReal);
    if(_verbose) { 
        log_info(" -- step size = {}, target time = {}", _targetDT, _tf);
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

    const CoordinateSet& coords = _model->getCoordinateSet();
    for (i = 0; i < nq; ++i) {
        qCorrection[i] = coords[i].getValue(s) - qDesired[i];
        uCorrection[i] = coords[i].getSpeedValue(s) - uDesired[i];
    }

    _predictor->getCMCActSubsys()->setCoordinateCorrections(&qCorrection[0]);
    _predictor->getCMCActSubsys()->setSpeedCorrections(&uCorrection[0]);

    if( _verbose ) {
        log_info("------------------------------");
        log_info("CMC::computeControls, summary:");
        log_info("------------------------------");
        log_info(" -- Q = {}", s.getQ());
        log_info(" -- U = {}", s.getU());
        log_info(" -- Z = {}", s.getZ());
        log_info(" -- Qdesired = {}", qDesired);
        log_info(" -- Udesired = {}", uDesired);
        log_info(" -- Qcorrection = {}", qCorrection);
        log_info(" -- Ucorrection = {}", uCorrection);
        log_info("------------------------------");
        log_info("");
    }

    // realize to Velocity because some tasks (eg. CMC_Point) need to be
    // at velocity to compute errors
    _model->getMultibodySystem().realize(s, Stage::Velocity );

    // ERRORS
    _taskSet->computeErrors(s, tiReal);
    _taskSet->recordErrorsAsLastErrors();
    Array<double> &pErr = _taskSet->getPositionErrors();
    Array<double> &vErr = _taskSet->getVelocityErrors();
    if(_verbose) {
        log_info("Errors at time {}: ", tiReal);
    }
    int e=0;
    for(i=0;i<_taskSet->getSize();i++) {
        
        TrackingTask& task = _taskSet->get(i);

        if(_verbose) {
            for(j=0;j<task.getNumTaskFunctions();j++) {
                log_warn("Task '{}': pErr = {}, vErr = {}.", task.getName(),
                        pErr[e], vErr[e]);
                e++;
            }
            log_info("");
        }
    }

    std::unique_ptr<double[]> err{new double[pErr.getSize()]};
    for(i=0;i<pErr.getSize();i++) err[i] = pErr[i];
    _pErrStore->append(tiReal,pErr.getSize(),err.get());
    for(i=0;i<vErr.getSize();i++) err[i] = vErr[i];
    _vErrStore->append(tiReal,vErr.getSize(),err.get());

    
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
                    if(_verbose) {
                        log_info("Task {}: pErr = {}, limit = {}, sigmoid = {}.",
                            i, pErr[i], jointTask.getLimit(), w);
                    }
                    stressTermWeight = min(stressTermWeight, w);
                }
            }
        }
        if(_verbose) {
            log_info("Setting stress term weight to {} (relativeTau was {}).",
                stressTermWeight, relativeTau);
            log_info("");
        }
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
    }

    if(_verbose) {
        log_info("xmin: {}", xmin);
        log_info("xmax: {}", xmax);
        log_info("");
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
        log_info("tiReal = {}, tfReal = {}", tiReal, tfReal);
        log_info("Min forces: {}", fmin);
        log_info("Max forces: {}", fmax);
        log_info("");
    }

    // Print actuator force range if range is small
    double range;
    for(i=0;i<N;i++) {
        range = fmax[i] - fmin[i];
        if(range<1.0) {
            log_warn("CMC::computeControls: small force range for {} ({} to {})",
                getActuatorSet()[i].getName(), fmin[i], fmax[i]);
            log_info("");

            // if the force range is so small it means the control value, x, 
            // is inconsequential and we might as well choose the smallest control
            // value possible, or else the RootSolver will choose the last value
            // it used to evaluate the force, which will be the maximum control
            // value. In other words, if the fiber length is so short that no level
            // of activation can produce force, the RootSolver gets the same answer
            // for force if it uses xmin or:: xmax, but since it uses xmax last
            // it returns xmax as the control value. Make xmax = xmin to avoid that.
            xmax[i] = xmin[i];
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
        catch (const SimTK::Exception::Base& ex) {
            log_error(ex.getMessage());
            log_error("OPTIMIZATION FAILED...");

            ostringstream msg;
            msg << "CMC::computeControls: Optimizer could not find a solution." << endl;
            msg << "Unable to find a feasible solution at time = " << s.getTime() << "." << endl;
            msg << "Model cannot generate the forces necessary to achieve the target acceleration." << endl;
            msg << "Possible issues: 1. not all model degrees-of-freedom are actuated, " << endl;
            msg << "2. there are tracking tasks for locked coordinates, and/or" << endl;
            msg << "3. there are unnecessary control constraints on reserve/residual actuators." << endl;
            msg << endl;
                   
            log_error(msg.str());

         throw(new OpenSim::Exception(msg.str(), __FILE__,__LINE__));
        }
    } else {
        // Got a direct solution, don't need to run optimizer
    }

    if(_verbose) _target->printPerformance(&_f[0]);

    if(_verbose) {
        log_info("Desired actuator forces: {}", _f);
        log_info("");
    }


    // ROOT SOLVE FOR EXCITATIONS
    _predictor->setTargetForces(&_f[0]);
    RootSolver rootSolver(_predictor);
    Array<double> tol(4.0e-3,N);
    Array<double> fErrors(0.0,N);
    Array<double> controls(0.0,N);
    controls = rootSolver.solve(s, xmin,xmax,tol);
    if(_verbose) {
        log_info("CMC::computeControls, root solve (tFinal = {}):", _tf);
        log_info(" -- controls = {}", _tf, controls);
        log_info("");
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
        if(aVerbosePrinting) {
            log_info("CMC::filterControls: aDT is practically 0.0, skipping!");
            log_info("");
        }
        return;
    }

    if(aVerbosePrinting) {
        log_info("Filtering controls to limit curvature...");
        log_info("");
    }

    int i;
    int size = rControls.getSize();
    Array<double> x0(0.0,size),x1(0.0,size),x2(0.0,size);

    // SET TIMES
    double t0,t1/*,t2*/;
    // t2 = s.getTime();
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
    
//      diff = fabs(m2) - fabs(m1);
//      cout<<"thresholdSlopeDiff="<<thresholdSlopeDiff<<"  slopeDiff="<<diff<<endl;
//      if(diff>thresholdSlopeDiff) continue;
        
        

        // ALTER CONTROL VALUE
        rControls[i] = (3.0*x2[i] + 2.0*x1[i] + x0[i]) / 6.0;

        // PRINT
        if(aVerbosePrinting) {
            log_info("ControlSet '{}': old = {}, new = {}", 
                    aControlSet[i].getName(), x2[i], rControls[i]);
            log_info("");
        }
    }
}


// Controller Interface. 
// compute the control value for all actuators this Controller is responsible for
void CMC::computeControls(const SimTK::State& s, SimTK::Vector& controls)  const
{
    SimTK_ASSERT( _controlSet.getSize() == getActuatorSet().getSize() , 
        "CMC::computeControls number of controls does not match number of actuators.");
    
    SimTK::Vector actControls(1, 0.0);
    for(int i=0; i<getActuatorSet().getSize(); i++){
        actControls[0] = _controlSet[_controlSetIndices[i]].getControlValue(s.getTime());
        getActuatorSet()[i].addInControls(actControls, controls);
    }

    // double *val = &controls[0];
}

// for any post XML deserialization initialization
void CMC::extendConnectToModel(Model& model)
{
    Super::extendConnectToModel(model);

    // STORAGE
    Array<string> labels;
    labels.append("time");
    for(int i=0;i<_taskSet->getSize();i++) {
        for(int j=0;j<_taskSet->get(i).getNumTaskFunctions();j++) {
            labels.append(_taskSet->get(i).getName());
        }
    }
    _pErrStore.reset(new Storage(1000,"PositionErrors"));
    _pErrStore->setColumnLabels(labels);
    _vErrStore.reset(new Storage(1000,"VelocityErrors"));
    _pErrStore->setColumnLabels(labels);
    _stressTermWeightStore.reset(new Storage(1000,"StressTermWeight"));

}
// for adding any components to the model
void CMC::extendAddToSystem( SimTK::MultibodySystem& system)  const
{
    Super::extendAddToSystem(system);

    // add event handler for updating controls for next window 
    CMC* mutableThis = const_cast<CMC *>(this);
    ComputeControlsEventHandler* computeControlsHandler = 
        new ComputeControlsEventHandler(mutableThis);

    system.updDefaultSubsystem().addEventHandler(computeControlsHandler );

    const Set<const Actuator>& fSet = getActuatorSet();
    int nActs = fSet.getSize();

    mutableThis->_controlSetIndices.setSize(nActs);

    // Create the control set that will hold the controls computed by CMC
    mutableThis->_controlSet.setName(_model->getName());
    mutableThis->_controlSet.setSize(0);

    // Define the control set used to specify control bounds and to hold 
    // the computed control values from the CMC algorithm
    double xmin =0, xmax=0;

    std::string actName = "";
    
    for(int i=0; i < nActs; ++i ) {

        auto* act = dynamic_cast<const ScalarActuator*>(&fSet[i]);
        //Actuator& act = getActuatorSet().get(i);

        ControlLinear *control = new ControlLinear();
        control->setName(act->getName() + ".excitation" );

        xmin = act->getMinControl();
        if (xmin ==-SimTK::Infinity)
            xmin =-MAX_CONTROLS_FOR_RRA;
        
        xmax =  act->getMaxControl();
        if (xmax ==SimTK::Infinity)
            xmax =MAX_CONTROLS_FOR_RRA;

        auto *musc = dynamic_cast<const Muscle *>(act);
        // if controlling muscles, CMC requires that the control be constant (i.e. piecewise constant or use steps)
        // since it uses this assumption to rootsolve for the required controls over the CMC time-window.
        if(musc){
            control->setUseSteps(true);
            if(xmin < MIN_CMC_CONTROL_VALUE){
                log_warn("CMC::extendAddToSystem: CMC cannot compute controls for muscles with muscle controls less than {}.", MIN_CMC_CONTROL_VALUE);
                log_warn("CMC::extendAddToSystem: The minimum control limit for muscle '{}' has been reset to {}.", musc->getName(), MIN_CMC_CONTROL_VALUE);
                xmin = MIN_CMC_CONTROL_VALUE;
            }
            if(xmax < MAX_CMC_CONTROL_VALUE){
                log_warn("CMC::extendAddToSystem: CMC cannot compute controls for muscles with muscle controls greater than {}.", MAX_CMC_CONTROL_VALUE);
                log_warn("CMC::extendAddToSystem: The maximum control limit for muscle '{}' has been reset to {}.", musc->getName(), MAX_CMC_CONTROL_VALUE);
                xmax = MAX_CMC_CONTROL_VALUE;
            }
        }

        control->setDefaultParameterMin(xmin);
        control->setDefaultParameterMax(xmax);

        mutableThis->_controlSet.adoptAndAppend(control);
        mutableThis->_controlSetIndices.set(i, i);
    }

    mutableThis->setNumControls(_controlSet.getSize());
}

