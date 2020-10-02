/* -------------------------------------------------------------------------- *
 *                           OpenSim:  RRATool.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Contributor(s): Frank C. Anderson, Eran Guendelman, Chand T. John          *
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
#include "RRATool.h"
#include "CMC.h"
#include "CMC_TaskSet.h"
#include "ActuatorForceTarget.h"
#include "ActuatorForceTargetFast.h"
#include "AnalyzeTool.h"
#include "VectorFunctionForActuators.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/CMCActuatorSubsystem.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/InverseDynamics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Common/DebugUtilities.h>


using namespace std;
using namespace SimTK;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
RRATool::~RRATool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
RRATool::RRATool() :
    AbstractTool(),
    _desiredPointsFileName(_desiredPointsFileNameProp.getValueStr()),
    _desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
    _taskSetFileName(_taskSetFileNameProp.getValueStr()),
    _constraintsFileName(_constraintsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
    _numericalDerivativeStepSize(_numericalDerivativeStepSizeProp.getValueDbl()),
    _optimizationConvergenceTolerance(_optimizationConvergenceToleranceProp.getValueDbl()),
    _adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool()),
    _initialTimeForCOMAdjustment(_initialTimeForCOMAdjustmentProp.getValueDbl()),
    _finalTimeForCOMAdjustment(_finalTimeForCOMAdjustmentProp.getValueDbl()),
    _adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
    _outputModelFile(_outputModelFileProp.getValueStr()),
    _verbose(_verboseProp.getValueBool())
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct from an XML property file.
 *
 * @param aFileName File name of the XML document.
 */
RRATool::RRATool(const string &aFileName, bool aLoadModel) :
    AbstractTool(aFileName, false),
    _desiredPointsFileName(_desiredPointsFileNameProp.getValueStr()),
    _desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
    _taskSetFileName(_taskSetFileNameProp.getValueStr()),
    _constraintsFileName(_constraintsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
    _numericalDerivativeStepSize(_numericalDerivativeStepSizeProp.getValueDbl()),
    _optimizationConvergenceTolerance(_optimizationConvergenceToleranceProp.getValueDbl()),
    _adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool()),
    _initialTimeForCOMAdjustment(_initialTimeForCOMAdjustmentProp.getValueDbl()),
    _finalTimeForCOMAdjustment(_finalTimeForCOMAdjustmentProp.getValueDbl()),
    _adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
    _outputModelFile(_outputModelFileProp.getValueStr()),
    _verbose(_verboseProp.getValueBool())
{
    setNull();
    updateFromXMLDocument();
    if(aLoadModel){
        loadModel(aFileName, &_originalForceSet);
        // Append to or replace model forces with those (i.e. actuators) specified by the analysis
        updateModelForces(*_model, aFileName);
        setModel(*_model);  
        setToolOwnsModel(true);
    }
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Tools do not copy the Object's DOMnode
 * and XMLDocument.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for a Tool:
 *
 * 1) Construction based on XML file (@see Tool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Tool(const XMLDocument *aDocument).
 * This constructor explicitly requests construction based on an
 * XML document that is held in memory.  In this way the proper connection
 * between an object's node and the corresponding node within the XML
 * document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Tool member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Tool member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the Tool from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aTool Object to be copied.
 * @see Tool(const XMLDocument *aDocument)
 * @see Tool(const char *aFileName)
 * @see generateXMLDocument()
 */
RRATool::
RRATool(const RRATool &aTool) :
    AbstractTool(aTool),
    _desiredPointsFileName(_desiredPointsFileNameProp.getValueStr()),
    _desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
    _taskSetFileName(_taskSetFileNameProp.getValueStr()),
    _constraintsFileName(_constraintsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
    _numericalDerivativeStepSize(_numericalDerivativeStepSizeProp.getValueDbl()),
    _optimizationConvergenceTolerance(_optimizationConvergenceToleranceProp.getValueDbl()),
    _adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool()),
    _initialTimeForCOMAdjustment(_initialTimeForCOMAdjustmentProp.getValueDbl()),
    _finalTimeForCOMAdjustment(_finalTimeForCOMAdjustmentProp.getValueDbl()),
    _adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
    _outputModelFile(_outputModelFileProp.getValueStr()),
    _verbose(_verboseProp.getValueBool())
{
    setNull();
    *this = aTool;
}


//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void RRATool::
setNull()
{
    setupProperties();

    _desiredPointsFileName = "";
    _desiredKinematicsFileName = "";
    _taskSetFileName = "";
    _constraintsFileName = "";
    _lowpassCutoffFrequency = -1.0;
    //_lowpassCutoffFrequencyForLoadKinematics = -1.0;
    _optimizerAlgorithm = "ipopt";
    _numericalDerivativeStepSize = 1.0e-4;
    _optimizationConvergenceTolerance = 1.0e-5;
    _adjustedCOMBody = "";
    _adjustCOMToReduceResiduals = false;
    _initialTimeForCOMAdjustment = -1;
    _finalTimeForCOMAdjustment = -1;
    _outputModelFile = "";
    _adjustKinematicsToReduceResiduals=true;
    _verbose = false;
    _targetDT = .001;
    _replaceForceSet = false;   // default should be false for Forward.


}
//_____________________________________________________________________________
/**
 * Give this object's properties their XML names and add them to the property
 * list held in class Object (@see OpenSim::Object).
 */
void RRATool::setupProperties()
{
    string comment;

    comment = "Motion (.mot) or storage (.sto) file containing the desired point trajectories.";
    _desiredPointsFileNameProp.setComment(comment);
    _desiredPointsFileNameProp.setName("desired_points_file");
    _propertySet.append( &_desiredPointsFileNameProp );

    comment = "Motion (.mot) or storage (.sto) file containing the desired kinematic trajectories.";
    _desiredKinematicsFileNameProp.setComment(comment);
    _desiredKinematicsFileNameProp.setName("desired_kinematics_file");
    _propertySet.append( &_desiredKinematicsFileNameProp );

    comment = "File containing the tracking tasks. Which coordinates are tracked and with what weights are specified here.";         
    _taskSetFileNameProp.setComment(comment);        
    _taskSetFileNameProp.setName("task_set_file");       
    _propertySet.append( &_taskSetFileNameProp );

    comment = "DEPRECATED File containing the constraints on the controls.";
    _constraintsFileNameProp.setComment(comment);
    _constraintsFileNameProp.setName("constraints_file");
    _propertySet.append( &_constraintsFileNameProp );

    comment = "Low-pass cut-off frequency for filtering the desired kinematics.";
    comment += " A negative value results in no filtering. The default value is -1.0, so no filtering.";
    _lowpassCutoffFrequencyProp.setComment(comment);
    _lowpassCutoffFrequencyProp.setName("lowpass_cutoff_frequency");
    _propertySet.append( &_lowpassCutoffFrequencyProp );


    comment = "Preferred optimizer algorithm (currently support \"ipopt\" or \"cfsqp\", "
                 "the latter requiring the osimCFSQP library.";
    _optimizerAlgorithmProp.setComment(comment);
    _optimizerAlgorithmProp.setName("optimizer_algorithm");
    _propertySet.append( &_optimizerAlgorithmProp );

    comment = "Step size used by the optimizer to compute numerical derivatives. "
                 "A value between 1.0e-4 and 1.0e-8 is usually appropriate.";
    _numericalDerivativeStepSizeProp.setComment(comment);
    _numericalDerivativeStepSizeProp.setName("numerical_derivative_step_size");
    _propertySet.append( &_numericalDerivativeStepSizeProp );

    comment = "Convergence criterion for the optimizer. The smaller this value, the deeper the convergence. "
                 "Decreasing this number can improve a solution, but will also likely increase computation time.";
    _optimizationConvergenceToleranceProp.setComment(comment);
    _optimizationConvergenceToleranceProp.setName("optimization_convergence_tolerance");
    _propertySet.append( &_optimizationConvergenceToleranceProp );

    comment = "Flag (true or false) indicating whether or not to make an adjustment "
                 "in the center of mass of a body to reduced DC offsets in MX and MZ. "
                 "If true, a new model is written out that has altered anthropometry.";
    _adjustCOMToReduceResidualsProp.setComment(comment);
    _adjustCOMToReduceResidualsProp.setName("adjust_com_to_reduce_residuals");
    _propertySet.append( &_adjustCOMToReduceResidualsProp );

    comment = "Initial time used when computing average residuals in order to adjust "
                 "the body's center of mass.  If both initial and final time are set to "
                 "-1 (their default value) then the main initial and final time settings will be used.";
    _initialTimeForCOMAdjustmentProp.setComment(comment);
    _initialTimeForCOMAdjustmentProp.setName("initial_time_for_com_adjustment");
    _propertySet.append( &_initialTimeForCOMAdjustmentProp );

    comment = "Final time used when computing average residuals in order to adjust "
                 "the body's center of mass.";
    _finalTimeForCOMAdjustmentProp.setComment(comment);
    _finalTimeForCOMAdjustmentProp.setName("final_time_for_com_adjustment");
    _propertySet.append( &_finalTimeForCOMAdjustmentProp );

    comment = "Name of the body whose center of mass is adjusted. "
                 "The heaviest segment in the model should normally be chosen. "
                 "For a gait model, the torso segment is usually the best choice.";
    _adjustedCOMBodyProp.setComment(comment);
    _adjustedCOMBodyProp.setName("adjusted_com_body");
    _propertySet.append( &_adjustedCOMBodyProp );

    comment = "Name of the output model file (.osim) containing adjustments to anthropometry "
                 "made to reduce average residuals. This file is written if the property "
                 "adjust_com_to_reduce_residuals is set to true. If a name is not specified, "
                 "the model is written out to a file called adjusted_model.osim.";
    _outputModelFileProp.setComment(comment);
    _outputModelFileProp.setName("output_model_file");
    _propertySet.append( &_outputModelFileProp );

    comment = "True-false flag indicating whether or not to turn on verbose printing for cmc.";
    _verboseProp.setComment(comment);
    _verboseProp.setName("use_verbose_printing");
    _propertySet.append( &_verboseProp );

}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
RRATool& RRATool::
operator=(const RRATool &aTool)
{
    // BASE CLASS
    AbstractTool::operator=(aTool);

    // MEMBER VARIABLES
    _desiredPointsFileName = aTool._desiredPointsFileName;
    _desiredKinematicsFileName = aTool._desiredKinematicsFileName;
    _taskSetFileName = aTool._taskSetFileName;
    _constraintsFileName = aTool._constraintsFileName;
    _lowpassCutoffFrequency = aTool._lowpassCutoffFrequency;
    _targetDT = aTool._targetDT;         
    _numericalDerivativeStepSize = aTool._numericalDerivativeStepSize;
    _optimizationConvergenceTolerance = aTool._optimizationConvergenceTolerance;
    _optimizerAlgorithm = aTool._optimizerAlgorithm;
    _adjustedCOMBody = aTool._adjustedCOMBody;
    _outputModelFile = aTool._outputModelFile;
    _adjustCOMToReduceResiduals = aTool._adjustCOMToReduceResiduals;
    _initialTimeForCOMAdjustment = aTool._initialTimeForCOMAdjustment;
    _finalTimeForCOMAdjustment = aTool._finalTimeForCOMAdjustment;
    _verbose = aTool._verbose;

    return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the investigation.
 */
bool RRATool::run()
{
    log_info("Running tool {}.", getName());

    // CHECK FOR A MODEL
    if(_model==NULL) {
        string msg = "A model has not been set.";
        log_error(msg);
        throw(Exception(msg,__FILE__,__LINE__));
    }
    // OUTPUT DIRECTORY
    // Do the maneuver to change then restore working directory 
    // so that the parsing code behaves properly if called from a different directory
    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    try {

    // SET OUTPUT PRECISION
    IO::SetPrecision(_outputPrecision);


    // CHECK PROPERTIES FOR ERRORS/INCONSISTENCIES
    if(_adjustCOMToReduceResiduals) {
        if(_adjustedCOMBody == "")
            throw Exception("RRATool: ERROR- "+_adjustCOMToReduceResidualsProp.getName()+" set to true but "+
                                 _adjustedCOMBodyProp.getName()+" not set",__FILE__,__LINE__);
        else if (_model->getBodySet().getIndex(_adjustedCOMBody) == -1)
            throw Exception("RRATool: ERROR- Body '"+_adjustedCOMBody+"' specified in "+
                                 _adjustedCOMBodyProp.getName()+" not found",__FILE__,__LINE__);
    }

    /*bool externalLoads = */createExternalLoads(_externalLoadsFileName, *_model);

    CMC_TaskSet taskSet(_taskSetFileName);           
    log_info("\ttaskSet size = {}.", taskSet.getSize());         

    CMC* controller = new CMC(_model,&taskSet); // Need to make it a pointer since Model takes ownership 
    controller->setName( "CMC" );
    controller->setActuators(_model->updActuators());
    _model->addController(controller );
    controller->setEnabled(true);
    controller->setUseCurvatureFilter(false);
    controller->setTargetDT(.001);
    controller->setCheckTargetTime(true);

    //Make sure system is up-to-date with model (i.e. added actuators, etc...)
    SimTK::State& s = _model->initSystem();
    _model->getMultibodySystem().realize(s, Stage::Position );
     taskSet.setModel(*_model);
    _model->equilibrateMuscles(s);
  
    // ---- INPUT ----
    // DESIRED POINTS AND KINEMATICS
    if(_desiredPointsFileName=="" && _desiredKinematicsFileName=="") {
        log_error("A desired points file and desired kinematics file were not "
                  "specified.");
        return false;
    }

    Storage *desiredPointsStore=NULL;
    bool desiredPointsFlag = false;
    if(_desiredPointsFileName=="") {
        log_warn("A desired points file was not specified.");
    } else {
        log_info("Loading desired points from file '{}'...",
                _desiredPointsFileName);
        desiredPointsStore = new Storage(_desiredPointsFileName);
        desiredPointsFlag = true;
    }

    Storage *desiredKinStore=NULL;
    bool desiredKinFlag = false;
    if(_desiredKinematicsFileName=="") {
        log_warn("A desired kinematics file was not specified.");
    } else {
        log_info("Loading desired kinematics from file '{}'...",
            _desiredKinematicsFileName);
        desiredKinStore = new Storage(_desiredKinematicsFileName);
        desiredKinFlag = true;
    }

    // ---- INITIAL AND FINAL TIME ----
    // NOTE: important to do this before padding (for filtering)
    // Initial Time
    if(desiredPointsFlag) {
        double ti = desiredPointsStore->getFirstTime();
        if(_ti<ti) {
            log_warn("The initial time set for the cmc run precedes the first "
                "time in the desired points file '{}'. Resetting the initial "
                "time from {} to {}.\n", _desiredPointsFileName, _ti, ti);
            _ti = ti;
        }
        // Final time
        double tf = desiredPointsStore->getLastTime();
        if(_tf>tf) {
            log_warn("The final time set for the cmc run is past the last time "
                "stamp in the desired points file '{}'. Resetting the final "
                "time from {} to {}.\n", _desiredPointsFileName, _tf, tf);
            _tf = tf;
        }
    }

    // Initial Time
    if(desiredKinFlag) {
        double ti = desiredKinStore->getFirstTime();
        if(_ti<ti) {
            log_warn("The initial time set for the cmc run precedes the first "
                "time in the desired kinematics file '{}'. Resetting the "
                "initial time from {} to {}.\n", _desiredKinematicsFileName, _ti,
                ti);
            _ti = ti;
        }
        // Final time
        double tf = desiredKinStore->getLastTime();
        if(_tf>tf) {
            log_warn("The final time set for the cmc run is past the last time "
                "stamp in the desired kinematics file '{}'. Resetting the "
                "final time from {} to {}.", _desiredKinematicsFileName, _tf,
                tf);
            _tf = tf;
        }
    }

    // Filter
    // Eran: important to filter *before* calling formCompleteStorages because we need the
    // constrained coordinates (e.g. tibia-patella joint angle) to be consistent with the
    // filtered trajectories
    if(desiredPointsFlag) {
        desiredPointsStore->pad(60);
        desiredPointsStore->print("desiredPoints_padded.sto");
        if(_lowpassCutoffFrequency>=0) {
            int order = 50;
            log_info("Low-pass filtering desired points with a cutoff "
                "frequency of {}...", _lowpassCutoffFrequency);
            desiredPointsStore->lowpassFIR(order,_lowpassCutoffFrequency);
        } else {
            log_info("Not filtering the desired points.");
        }
    }

    if(desiredKinFlag) {
        desiredKinStore->pad(60);
        if (_verbose) desiredKinStore->print("desiredKinematics_padded.sto");
        if(_lowpassCutoffFrequency>=0) {
            int order = 50;
            log_info("Low-pass filtering desired kinematics with a cutoff "
                "frequency of {}...", _lowpassCutoffFrequency);
            desiredKinStore->lowpassFIR(order,_lowpassCutoffFrequency);
        } else {
            log_info("Not filtering the desired kinematics.");
        }
    }

     // TASK SET
    if(_taskSetFileName=="") {           
        log_error("A task set was not specified.");
        return false;        
    }

    _model->printDetailedInfo(s, cout);

    int nq = _model->getNumCoordinates();
    int nu = _model->getNumSpeeds();
    int na = _model->getActuators().getSize();

    // Form complete storage objects for the q's and u's
    // This means filling in unspecified generalized coordinates and
    // setting constrained coordinates to their valid values.
    Storage *qStore=NULL;
    Storage *uStore=NULL;

    if(desiredKinFlag) {
        _model->getMultibodySystem().realize(s, Stage::Time );
        // qStore and uStore returned are in radians
        _model->getSimbodyEngine().formCompleteStorages(s, *desiredKinStore,
            qStore, uStore);
    }

    // Adjust COM to reduce residuals (formerly RRA pass 1) if requested
    string massAdjMsg;
    if(desiredKinFlag) {
        if(_adjustCOMToReduceResiduals) {
        
            massAdjMsg = adjustCOMToReduceResiduals(s, *qStore,*uStore);

            // If not adjusting kinematics, we don't proceed with CMC, and just stop here.
            if(!_adjustKinematicsToReduceResiduals) {
                log_info("No kinematics adjustment requested.");
                delete qStore;
                delete uStore;
                writeAdjustedModel();
                return true;
            }
        }
    }

    // Spline
    GCVSplineSet *posSet=NULL;
    if(desiredPointsFlag) {
        log_info("Constructing function set for tracking desired points...");
        posSet = new GCVSplineSet(5,desiredPointsStore);

        Storage *velStore=posSet->constructStorage(1);
        GCVSplineSet velSet(5,velStore);
        delete velStore; velStore=NULL;

        // Print acc for debugging
        if (_verbose) {
            Storage *accStore=posSet->constructStorage(2);
            accStore->print("desiredPoints_splinefit_accelerations.sto");
            delete accStore; accStore=NULL; 
        }
    }

    GCVSplineSet *qSet=NULL;
    GCVSplineSet *uSet=NULL;
    GCVSplineSet *uDotSet=NULL;

    if(desiredKinFlag) {
        log_info("Constructing function set for tracking desired kinematics...");
        qSet = new GCVSplineSet(5,qStore);
        delete qStore; qStore = NULL;

        uSet = new GCVSplineSet(5,uStore);
        delete uStore; uStore=NULL;

        Storage *dudtStore = uSet->constructStorage(1);
        uDotSet = new GCVSplineSet(5,dudtStore);

        // Print dudt for debugging
        if (_verbose) {
            dudtStore->print("desiredKinematics_splinefit_accelerations.sto");
        }
        delete dudtStore; dudtStore=NULL;
    }

    // ANALYSES
    addNecessaryAnalyses();

    GCVSplineSet *qAndPosSet=NULL;
    qAndPosSet = new GCVSplineSet();
    if(desiredPointsFlag) {
        int nps=posSet->getSize();
        for(int i=0;i<nps;i++) {
            qAndPosSet->adoptAndAppend(&posSet->get(i));
        }
    }
    if(desiredKinFlag) {
        int nqs=qSet->getSize();
        for(int i=0;i<nqs;i++) {
            qAndPosSet->adoptAndAppend(&qSet->get(i));
        }
    }
    if (taskSet.getDataFileName()!=""){
        // Add functions from data files to track states
        Storage stateStorage(taskSet.getDataFileName());
        GCVSplineSet* stateFuntcions = new GCVSplineSet(3, &stateStorage);
        for (int i=0; i< stateFuntcions->getSize(); i++)
            qAndPosSet->cloneAndAppend(stateFuntcions->get(i));

    }

    taskSet.setFunctions(*qAndPosSet);
    taskSet.setFunctionsForVelocity(*uSet);
    taskSet.setFunctionsForAcceleration(*uDotSet);
 
    // CONSTRAINTS ON THE CONTROLS
    ControlSet *controlConstraints = NULL;
    if(_constraintsFileName!="") {
        controlConstraints = new ControlSet(_constraintsFileName);
        log_warn("Using DEPRECATED Control Constraints file '{}' in RRA, "
            "generally unnecessary. Support will be dropped in the future.\n", 
            _constraintsFileName);
    }

    // ---- INITIAL STATES ----
    Array<double> q(0.0,nq);
    Array<double> u(0.0,nu);
    if(desiredKinFlag) {
        log_info("Using the generalized coordinates specified in '{}' to set "
            "the initial configuration.", _desiredKinematicsFileName);
        qSet->evaluate(q,0,_ti);
        uSet->evaluate(u,0,_ti);
    } else {
        log_info("Using the generalized coordinates specified as zeros to set "
            "the initial configuration.");
    }

    // formCompleteStorages ensures qSet is in order of model Coordinates
    // but we cannot assume order of coordinates is the same in the State,
    // so set each Coordinate value and speed individually.
    const CoordinateSet& coords = _model->getCoordinateSet();
    for (int i = 0; i < nq; ++i) {
        // The last argument to setValue is a bool to enforce kinematic constraints
        // or not. It is being set to true when we set the last coordinate value.
        coords[i].setValue(s, q[i], i==(nq-1));
        coords[i].setSpeedValue(s, u[i]);
    }

    // Actuator force predictor
    // This requires the trajectories of the generalized coordinates
    // to be specified.
    
    string rraControlName;
    CMCActuatorSystem actuatorSystem;
    CMCActuatorSubsystem cmcActSubsystem(actuatorSystem, _model);
    cmcActSubsystem.setCoordinateTrajectories(qSet);
    actuatorSystem.realizeTopology();
    // initialize the actuator states 
    SimTK::State& actuatorSystemState = actuatorSystem.updDefaultState(); 
    
    SimTK::Vector &actSysZ = actuatorSystemState.updZ();
    const SimTK::Vector &modelZ = _model->getForceSubsystem().getZ(s);
    
    // int nra = actSysZ.size();
    // int nrm = modelZ.size();

    assert(actSysZ.size() == modelZ.size());
    actSysZ = modelZ;

    VectorFunctionForActuators *predictor =
        new VectorFunctionForActuators(&actuatorSystem, _model, &cmcActSubsystem);

    controller->setActuatorForcePredictor(predictor);
    controller->updTaskSet().setFunctions(*qAndPosSet);

    // Optimization target
    OptimizationTarget *target = NULL;
    if(false) {
        target = new ActuatorForceTargetFast(s, na,controller);
    } else {
        target = new ActuatorForceTarget(na,controller);
    }
    target->setDX(_numericalDerivativeStepSize);

    // Pick optimizer algorithm
    SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
    if(IO::Uppercase(_optimizerAlgorithm) == "CFSQP") {
        if(!SimTK::Optimizer::isAlgorithmAvailable(SimTK::CFSQP)) {
            log_warn("CFSQP optimizer algorithm unavailable. Will try to use "
                "IPOPT instead.");
            algorithm = SimTK::InteriorPoint;
        } else {
            log_info("Using CFSQP optimizer algorithm.");
            algorithm = SimTK::CFSQP;
        }
    } else if(IO::Uppercase(_optimizerAlgorithm) == "IPOPT") {
        log_info("Using IPOPT optimizer algorithm.");
        algorithm = SimTK::InteriorPoint;
    } else {
        throw Exception("RRATool: ERROR- Unrecognized optimizer algorithm: '"+_optimizerAlgorithm+"'",__FILE__,__LINE__);
    }

    SimTK::Optimizer *optimizer = new SimTK::Optimizer(*target, algorithm);
    controller->setOptimizationTarget(target, optimizer);

    log_info("Setting optimizer print level to {}.", _verbose ? 4 : 0);
    optimizer->setDiagnosticsLevel(_verbose ? 4 : 0);
    log_info("Setting optimizer convergence tolerance to {}.",
        _optimizationConvergenceTolerance);
    optimizer->setConvergenceTolerance(_optimizationConvergenceTolerance);
    log_info("Setting optimizer maximum iterations to {}.", 2000);
    optimizer->setMaxIterations(2000);
    optimizer->useNumericalGradient(false); // Use our own central difference approximations
    optimizer->useNumericalJacobian(false);
    if(algorithm == SimTK::InteriorPoint) {
        // Some IPOPT-specific settings
        optimizer->setLimitedMemoryHistory(500); // works well for our small systems
        optimizer->setAdvancedBoolOption("warm_start",true);
        optimizer->setAdvancedRealOption("obj_scaling_factor",1);
        optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",100);
    }

    if(_verbose) {
        log_info("Setting cmc controller to use verbose printing.\n");
    } else {
        log_info("Setting cmc controller to not use verbose printing.\n");
    }
    controller->setUseVerbosePrinting(_verbose);

    controller->setCheckTargetTime(true);

    // ---- SIMULATION ----
    //
    // Manager
    Manager manager(*_model);
    manager.setIntegratorMaximumStepSize(_maxDT);
    manager.setIntegratorMinimumStepSize(_minDT);
    manager.setIntegratorAccuracy(_errorTolerance);
    
    _model->setAllControllersEnabled( true );

    manager.setSessionName(getName());
    s.setTime(_ti);
    double finalTime = _tf - _targetDT - SimTK::Zero;

    // Initialize integrand controls using controls read in from file (which specify min/max control values)
    initializeControlSetUsingConstraints(NULL,controlConstraints, controller->updControlSet());

    // Initial auxiliary states
    time_t startTime,finishTime;
    double elapsedTime;
    if( s.getNZ() > 0) { // If there are actuator states (i.e. muscles dynamics)
        log_info("-----------------------------------------------------------------");
        log_info("Computing initial values for muscles states (activation, length):");
        log_info("-----------------------------------------------------------------");
        time(&startTime);
        log_info(" -- Start time = {}", getTimeString(startTime));
        log_info("-----------------------------------------------------------------");
        log_info("");

        try {
        controller->computeInitialStates(s,_ti);
        }
        catch(const Exception& x) {
        // TODO: eventually might want to allow writing of partial results
            log_error(x.what());
            return false;
        }
        catch(...) {
            // TODO: eventually might want to allow writing of partial results
            // close open files if we die prematurely (e.g. Opt fail)
            return false;
        }
        time(&finishTime);
        // copy the final states from the last integration 
        s.updY() = cmcActSubsystem.getCompleteState().getY();
        log_info("----------------------------------");
        log_info("Finished computing initial states:");
        log_info("----------------------------------");
        log_info(" -- Start time = {}", getTimeString(startTime));
        log_info(" -- Finish time = {}", getTimeString(finishTime));
        elapsedTime = difftime(finishTime, startTime);
        log_info(" -- Elapsed time = {} seconds.", elapsedTime);
        log_info("----------------------------------");
        log_info("");

    } else {
        cmcActSubsystem.setCompleteState( s );
        actuatorSystemState.updTime() = _ti; 
        s.updTime() = _ti;
        actuatorSystem.realize(actuatorSystemState, Stage::Time );
        controller->setTargetDT(1.0e-8);
        controller->computeControls( s, controller->updControlSet() );
        controller->setTargetDT(_targetDT);
    }

    // ---- INTEGRATE ----
    log_info("--------------------------------------------");
    log_info("Using CMC to track the specified kinematics:");
    log_info("--------------------------------------------");
    log_info(" -- Integrating from {} to {}", _ti, _tf);
    s.updTime() = _ti;
    controller->setTargetTime( _ti );
    time(&startTime);
    log_info(" -- Start time = {}", getTimeString(startTime));
    log_info("--------------------------------------------");
    log_info("");

    _model->getMultibodySystem().realize(s, Stage::Acceleration );

    controller->updTaskSet().computeAccelerations(s);

    cmcActSubsystem.setCompleteState( s );

    // Set output file names so that files are flushed regularly in case we fail
    IO::makeDir(getResultsDir());   // Create directory for output in case it doesn't exist
    manager.getStateStorage().setOutputFileName(getResultsDir() + "/" + getName() + "_states.sto");
    try {
        manager.initialize(s);
        manager.integrate(finalTime);
    }
    catch(const Exception& x) {
        // TODO: eventually might want to allow writing of partial results
        log_error(x.what());
        cwd.restore();
        // close open files if we die prematurely (e.g. Opt fail)
        manager.getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");
        return false;
    }
    catch(...) {
        // TODO: eventually might want to allow writing of partial results
        cwd.restore();
        // close open files if we die prematurely (e.g. Opt fail)
        manager.getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");
        return false;
    }
    time(&finishTime);
    log_info("-------------------------------------------");
    log_info("Finished tracking the specified kinematics:");
    log_info("-------------------------------------------");
    if( _verbose ){
      log_info(" -- States = {}", s.getY());
    }
    log_info(" -- Start time = {}", getTimeString(startTime));
    log_info(" -- Finish time = {}", getTimeString(finishTime));
    elapsedTime = difftime(finishTime, startTime);
    log_info(" -- Elapsed time = {} seconds.", elapsedTime);
    log_info("-------------------------------------------");
    log_info("");

    // ---- RESULTS -----
    printResults(getName(),getResultsDir()); // this will create results directory if necessary
    controller->updControlSet().print(getResultsDir() + "/" + getName() + "_controls.xml");
    _model->printControlStorage(getResultsDir() + "/" + getName() + "_controls.sto");
    manager.getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");
    /*
    Storage statesDegrees(manager.getStateStorage());
    _model->getSimbodyEngine().convertRadiansToDegrees(statesDegrees);
    statesDegrees.setWriteSIMMHeader(true);
    statesDegrees.print(getResultsDir() + "/" + getName() + "_states_degrees.mot");
    */
    controller->getPositionErrorStorage()->print(getResultsDir() + "/" + getName() + "_pErr.sto");

    stringstream adjQMsg;
    if(_model->getAnalysisSet().getIndex("Actuation") != -1) {
        Actuation& actuation = (Actuation&)_model->getAnalysisSet().get("Actuation");
        Array<double> FAve(0.0,3),MAve(0.0,3);
        Storage *forceStore = actuation.getForceStorage();
        computeAverageResiduals(*forceStore,FAve,MAve);

        adjQMsg << endl;
        adjQMsg << "************************************************************" << endl;
        adjQMsg << "*                   Final Average Residuals                *" << endl;
        adjQMsg << "************************************************************" << endl;
        adjQMsg << "* After "<<_adjustedCOMBody<<" COM and Kinematics adjustments:"<< endl;
        adjQMsg << "*  FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
        adjQMsg << "*  MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl;
        adjQMsg << "************************************************************\n" << endl;

        // Write the average residuals (DC offsets) out to a file
        ofstream residualFile((getResultsDir() + "/" + getName() + "_avgResiduals.txt").c_str());
        residualFile << "Average Residuals:\n\n";
        residualFile << "FX average = " << FAve[0] << "\n";
        residualFile << "FY average = " << FAve[1] << "\n";
        residualFile << "FZ average = " << FAve[2] << "\n";
        residualFile << "MX average = " << MAve[0] << "\n";
        residualFile << "MY average = " << MAve[1] << "\n";
        residualFile << "MZ average = " << MAve[2] << "\n";
        residualFile.close();
    }

    // Write new model file
    if(_adjustCOMToReduceResiduals) writeAdjustedModel();

    // Report mass adjustments.
    log_info(massAdjMsg);

    // Report kinematic adjustments.
    log_info(adjQMsg.str());

    //_model->removeController(controller); // So that if this model is from GUI it doesn't double-delete it.

    } catch(const Exception& x) {
        // TODO: eventually might want to allow writing of partial results
        log_error(x.what());
        // close open files if we die prematurely (e.g. Opt fail)
        
        return false;
    }

    return true;
}


//=============================================================================
// UTILITY
//=============================================================================
void RRATool::writeAdjustedModel() 
{
    if(_outputModelFile=="") {
        stringstream ss;
        ss << "Warning: A name for the output model was not set.\n";
        ss << "Specify a value for the property "
           << _outputModelFileProp.getName();
        ss << " in the setup file.\n";
        if (getDocument()){
            string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
            _outputModelFile = directoryOfSetupFile+"adjusted_model.osim";
        } else {
            ss << "Writing to adjusted_model.osim ...\n\n";
            _outputModelFile = "adjusted_model.osim";
        }
        ss << "Writing to " <<_outputModelFile << " ...\n\n";
        log_error(ss.str());
    }

    // Set the model's actuator set back to the original set.  e.g. in RRA1
    // we load the model but replace its (muscle) actuators with torque actuators.
    // So we need to put back the muscles before writing out the adjusted model.
    // NOTE: use operator= so actuator groups are properly copied over
    _model->updForceSet() = _originalForceSet;

    removeExternalLoadsFromModel();

    // CMC was added as a model controller, now remove before printing out
    int c = _model->updControllerSet().getIndex("CMC");
    _model->updControllerSet().remove(c);

    _model->print(_outputModelFile);
}
//_____________________________________________________________________________
/**
 * Compute the average residuals.
 *
 * @param rFAve The computed average force residuals.  The size of rFAve is
 * set to 3.
 * @param rMAve The computed average moment residuals.  The size of rMAve is
 * set to 3.
 */
void RRATool::
computeAverageResiduals(const Storage &aForceStore,OpenSim::Array<double> &rFAve,OpenSim::Array<double> &rMAve)
{

    int iFX, iFY, iFZ, iMX, iMY, iMZ;
    // COMPUTE AVERAGE
    int size = aForceStore.getSmallestNumberOfStates();
    Array<double> ave(0.0);
    ave.setSize(size);
    aForceStore.computeAverage(size,&ave[0]);
  

    // GET INDICES
      iFX = aForceStore.getStateIndex("FX");
      iFY = aForceStore.getStateIndex("FY");
      iFZ = aForceStore.getStateIndex("FZ");
      iMX = aForceStore.getStateIndex("MX");
      iMY = aForceStore.getStateIndex("MY");
      iMZ = aForceStore.getStateIndex("MZ");

//cout << "storage = " << aForceStore.getName() << endl;
//cout << "computeAverageResiduals size=" << size << "  ave=" << ave << endl;
//cout << "indexs=" << iFX << "  " << iFY << "  " << iFZ << "  " <<  iMX << "  " << iMY << "  " << iMZ <<  endl;

    // GET AVE FORCES
    if(iFX>=0) rFAve[0] = ave[iFX];
    if(iFY>=0) rFAve[1] = ave[iFY];
    if(iFZ>=0) rFAve[2] = ave[iFZ];
    
    // GET AVE MOMENTS
    if(iMX>=0) rMAve[0] = ave[iMX];
    if(iMY>=0) rMAve[1] = ave[iMY];
    if(iMZ>=0) rMAve[2] = ave[iMZ];
//cout << "forces= " << rFAve <<  endl;
//cout << "moments=" << rMAve <<  endl;

}
string RRATool::
adjustCOMToReduceResiduals(SimTK::State& s, const Storage &qStore, const Storage &uStore)
{
    // Create a states storage from q's and u's
    Storage *statesStore = AnalyzeTool::createStatesStorageFromCoordinatesAndSpeeds(*_model, qStore, uStore);

    double ti = _ti;
    double tf = _tf;
    if(_initialTimeForCOMAdjustment!=-1 || _finalTimeForCOMAdjustment!=-1) {
        ti = _initialTimeForCOMAdjustment;
        tf = _finalTimeForCOMAdjustment;
    }

    Array<double> FAve(0.0,3),MAve(0.0,3);

    double actualTi, actualTf;
    statesStore->getTime(statesStore->findIndex(ti),actualTi);
    statesStore->getTime(statesStore->findIndex(tf),actualTf);
    log_info("Requested COM adjustment time range {} to {}, clamped to nearest "
        "available data times: {} to {}.", ti, tf, actualTi, actualTf);

    computeAverageResiduals(s, *_model, ti, tf, *statesStore, FAve, MAve);

    std::stringstream resMsg;
    resMsg << endl;
    resMsg <<  "* Average residuals before adjusting "<<_adjustedCOMBody<<" COM:"<<endl;
    resMsg <<  "*  FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
    resMsg <<  "*  MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl;
    resMsg <<  "************************************************************" << endl;

    SimTK::Vector  restoreStates(s.getNY());
    restoreStates = s.getY();

    string massMsg = adjustCOMToReduceResiduals(FAve,MAve);
    SimTK::State &si = _model->initSystem();

    si.updY() = restoreStates;
    _model->getMultibodySystem().realize(si, Stage::Position );
    
    computeAverageResiduals(si, *_model, ti, tf, *statesStore, FAve, MAve);

    resMsg <<  "* Average residuals after adjusting "<<_adjustedCOMBody<<" COM:"<<endl;
    resMsg <<  "*  FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
    resMsg <<  "*  MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl;
    resMsg <<  "************************************************************\n" << endl;

    delete statesStore;

    return massMsg+resMsg.str();
}

// Uses an inverse dynamics analysis to compute average residuals
void RRATool::
computeAverageResiduals(SimTK::State& s, Model &aModel,double aTi,double aTf,const Storage &aStatesStore,OpenSim::Array<double>& rFAve,OpenSim::Array<double>& rMAve)
{
    // Turn off whatever's currently there (but remember whether it was on/off)
    AnalysisSet& analysisSet = aModel.updAnalysisSet();
    Array<bool> analysisSetOn = analysisSet.getOn();
    analysisSet.setOn(false);

    // add inverse dynamics analysis
    InverseDynamics* inverseDynamics = new InverseDynamics(&aModel);
    aModel.addAnalysis(inverseDynamics);
    inverseDynamics->setModel(aModel);
    /* undo interpolation at bounds for now as it changes results a little. Could fix evaluation at bounds issue
    Array<double> bounds;
    bounds.append(aTi);
    bounds.append(aTf);
    const_cast<Storage &>(aStatesStore).interpolateAt(bounds);*/
    int iInitial = aStatesStore.findIndex(aTi);
    int iFinal = aStatesStore.findIndex(aTf);
    aStatesStore.getTime(iInitial,aTi);
    aStatesStore.getTime(iFinal,aTf);
   
    aModel.getMultibodySystem().realize(s, Stage::Position );

    log_info("Computing average residuals between {} and {}...", aTi, aTf);
    AnalyzeTool::run(s, aModel, iInitial, iFinal, aStatesStore, false);

    computeAverageResiduals(*inverseDynamics->getStorage(),rFAve,rMAve);

//cout << "\nrFAve= " << rFAve << endl;
//cout << "\nrMAve= " << rMAve << endl;

    aModel.removeAnalysis(inverseDynamics); // This deletes the analysis as well

    // Turn off whatever's currently there
    analysisSet.setOn(analysisSetOn);
}
//_____________________________________________________________________________
/**
 * Adjust the center of mass to reduce any DC offsets in MX and MZ.
 *
 * @param aFAve The average residual forces.  The dimension of aFAve should
 * be 3.
 * @param aMAve The average residual moments.  The dimension of aMAve should
 * be 3.
 */
string RRATool::
adjustCOMToReduceResiduals(const OpenSim::Array<double> &aFAve,const OpenSim::Array<double> &aMAve)
{
    // CHECK SIZE
    assert(aFAve.getSize()==3 && aMAve.getSize()==3);

    // GRAVITY
    Vec3 g = _model->getGravity();

    // COMPUTE SEGMENT WEIGHT
    Body *body = &_model->updBodySet().get(_adjustedCOMBody);
    double bodyMass = body->get_mass();
    double bodyWeight = fabs(g[1])*bodyMass;
    if(bodyWeight < SimTK::Zero) {
        log_error("RRATool::adjustCOMToReduceResiduals: {} has no weight.", 
            _adjustedCOMBody);
        return "";
    }
    
    //---- COM CHANGE ----
    double limit = 0.100;
    double dx =  aMAve[2] / bodyWeight;
    double dz = -aMAve[0] / bodyWeight;
    if(dz > limit) dz = limit;
    if(dz < -limit) dz = -limit;
    if(dx > limit) dx = limit;
    if(dx < -limit) dx = -limit;

    //cout<<"RRATool.adjustCOMToReduceResiduals:\n";
    //cout<<_adjustedCOMBody<<" weight = "<<bodyWeight<<"\n";
    //cout<<"dx="<<dx<<", dz="<<dz<<endl;

    // GET EXISTING COM
    SimTK::Vec3 com = body->get_mass_center();

    // COMPUTE ALTERED COM
    com[0] -= dx;
    com[2] -= dz;


    // ALTER THE MODEL
    body->set_mass_center(com);

    //---- MASS CHANGE ----
    // Get recommended mass change.
    double dmass = aFAve[1] / g[1];
    //cout<<"\ndmass = "<<dmass<<endl;
    // Loop through bodies
    int i;
    int nb = _model->getNumBodies();
    double massTotal=0.0;
    Array<double> mass(0.0,nb),massChange(0.0,nb),massNew(0.0,nb);
    const BodySet & bodySet = _model->getBodySet();
    nb = bodySet.getSize();
    for(i=0;i<nb;i++) {
        mass[i] = bodySet[i].get_mass();
        massTotal += mass[i];
    }

    std::stringstream msg;
    msg << endl;
    msg <<"\n************************************************************" << endl;
    msg <<  "*      Summary of Mass Adjustments to Reduce Residuals     *" << endl;
    msg <<  "************************************************************" << endl;
    msg <<  "* Body adjusted: " << _adjustedCOMBody << endl;
    msg <<  "* Mass Center (COM) adjustment: dx ="<< dx <<", dz =" << dz <<endl;
    msg <<  "* New COM location: " << com << endl;
    msg <<  "************************************************************" << endl;
    msg <<  "* Recommended mass adjustments:                             "  <<endl;
    msg <<  "*  Total mass change: " << dmass << endl; 

    for(i=0;i<nb;i++) {
        massChange[i] = dmass * mass[i]/massTotal;
        massNew[i] = mass[i] + massChange[i];
        msg << "*  " << bodySet[i].getName()<<": orig mass = "
            <<mass[i]<<", new mass = "<<massNew[i] << endl;
    }
    msg <<  "************************************************************" << endl;
    msg <<  "* Note: Edit the model to make recommended adjustments to  *" << endl; 
    msg <<  "*       mass properties.                                   *" << endl;
    msg <<  "************************************************************" << endl;

    return msg.str();
}

//_____________________________________________________________________________
/**
 * Add Actuation/Kinematics analyses if necessary
 */
void RRATool::
addNecessaryAnalyses()
{
    int stepInterval = 1;
    AnalysisSet& as = _model->updAnalysisSet();
    // Add Actuation if necessary
    Actuation *act = NULL;
    for(int i=0; i<as.getSize(); i++) 
        if(as.get(i).getConcreteClassName() == "Actuation") { act = (Actuation*)&as.get(i); break; }
    if(!act) {
        log_warn("No Actuation analysis found in analysis set, adding one...");
        act = new Actuation(_model);
        act->setModel(*_model );
        act->setStepInterval(stepInterval);
        _model->addAnalysis(act);
    }
    
    // Add Kinematics if necessary
    // NOTE: also checks getPrintResultFiles() so that the Kinematics analysis added from the GUI does not count
    Kinematics *kin = NULL;
    for(int i=0; i<as.getSize(); i++) 
        if(as.get(i).getConcreteClassName() == "Kinematics" && as.get(i).getPrintResultFiles()) { kin = (Kinematics*)&as.get(i); break; }
    if(!kin) {
        log_warn("No Kinematics analysis found in analysis set, adding one...");
        kin = new Kinematics(_model);
        kin->setModel(*_model );
        kin->setStepInterval(stepInterval);
        //kin->getPositionStorage()->setWriteSIMMHeader(true);
        kin->setInDegrees(true);
        _model->addAnalysis(kin);
    } else {
        kin->setInDegrees(true);
        //kin->getPositionStorage()->setWriteSIMMHeader(true);
    }
    
}

//_____________________________________________________________________________
/**
 * Set controls to use steps except for the residuals, which use linear
 * interpolation.  This method also sets the min and max values for
 * the controls.
 *
 * @param aRRAControlSet Controls that were output by an RRA pass.
 * @param rControlSet Controls for the current run of CMC.
 * @todo The residuals are assumed to be the first 6 actuators.  This
 * needs to be made more general.  And, the control bounds need to
 * be set elsewhere, preferably in a file.
 */
void RRATool::
initializeControlSetUsingConstraints(
    const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints, ControlSet& rControlSet)
{
    // Initialize control set with control constraints file
    
    int size = rControlSet.getSize();
    if(aControlConstraints) {
        for(int i=0;i<size;i++) {
            int index = aControlConstraints->getIndex(rControlSet.get(i).getName());
            if(index == -1) {
                // backwards compatibility with old version of OpenSim that appended the 
                //                 //  control suffix to the name of the control
                index = aControlConstraints->getIndex(rControlSet.get(i).getName()+".excitation");
            }
            if( index > -1 ) { // if we have an associated control constraint 
                // then, use it initialize the model's control 
                rControlSet.set(i, aControlConstraints->get(index).clone() );
            }
        }
    }

    // FOR RESIDUAL CONTROLS, SET TO USE LINEAR INTERPOLATION
    if(aRRAControlSet!=NULL) {
        OPENSIM_FUNCTION_NOT_IMPLEMENTED();
        // Need to make sure code below still works after changes to controls/control constraints
#if 0
        size = aRRAControlSet->getSize();
        for(i=0;i<size;i++){
            string rraControlName = aRRAControlSet->get(i).getName();
            ControlLinear *control;
            try {
                control = (ControlLinear*)&rControlSet->get(rraControlName);
            } catch(const Exception& x) {
                continue;
            }
            if(control==NULL) continue;
            control->setUseSteps(false);
            log_info("Set {} to use linear interpolation.", rraControlName);
        }
#endif
    }   
}

//_____________________________________________________________________________
/**
 * Get a pointer to the Storage object holding forces. This's a utility routine used 
 * by the SimTrack GUI primarily to get access to residuals while running RRA.
 *
 * User-beware, the storage will get out of scope and potentially get deleted when the 
 * analysis is done, so no assumptions about the lifetime of the returned storage object 
 * outside the owning analysis should be made.
 */
Storage& RRATool::getForceStorage(){
        Actuation& actuation = (Actuation&)_model->getAnalysisSet().get("Actuation");
        return *actuation.getForceStorage();
}

void RRATool::setOriginalForceSet(const ForceSet &aForceSet) {
    _originalForceSet = aForceSet;
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the tool to match current version, issue warning for unexpected inputs
 */
/*virtual*/ void RRATool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = getDocument()->getDocumentVersion();
    std::string controlsFileName ="";
    if ( documentVersion < XMLDocument::getLatestVersion()){
        // Replace names of properties
        if (documentVersion<20301){
            // Check that no unexpected settings are used for RRA that was really CMC, we'll do this by checking that
            // replace_force_set is true
            // cmc_time_window is .001
            // solve_for_equilibrium_for_auxiliary_states is off
            // ControllerSet is empty
            SimTK::Xml::Document doc = SimTK::Xml::Document(getDocumentFileName());
            Xml::Element oldRoot = doc.getRootElement();
            Xml::Element toolNode;
            bool isCMCTool = true;
            if (oldRoot.getElementTag()=="OpenSimDocument"){
                Xml::element_iterator iterTool(oldRoot.element_begin("CMCTool"));
                if (iterTool==oldRoot.element_end()){
                    isCMCTool = false;
                }
                else
                    toolNode = *iterTool;
            }
            else { // older document that had no OpenSimDocument tag
                toolNode = oldRoot;
                isCMCTool = (oldRoot.getElementTag()=="CMCTool");
            }
            if (isCMCTool){
                Xml::element_iterator replace_force_setIter(toolNode.element_begin("replace_force_set"));
                if (replace_force_setIter != toolNode.element_end()){
                    String replace_forcesStr = replace_force_setIter->getValueAs<String>();
                    if (replace_forcesStr.toLower() != "true") {
                        log_warn("Old RRA setup file has replace_force_set set "
                            " to false, will be ignored.");
                    }
                }
                Xml::element_iterator timeWindowIter(toolNode.element_begin("cmc_time_window"));
                if (timeWindowIter != toolNode.element_end()){
                    double timeWindow = timeWindowIter->getValueAs<double>();
                    if (timeWindow != .001) {
                        log_warn("Old setup file has cmc_time_window set to {} "
                            ", will be ignored and .001 used instead.",
                            timeWindow);
                    }
                }
            }
        }
    }
    AbstractTool::updateFromXMLNode(aNode, versionNumber);
}
