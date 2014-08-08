/* -------------------------------------------------------------------------- *
 *                           OpenSim:  CMCTool.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <time.h>
#include "CMCTool.h"
#include "AnalyzeTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "VectorFunctionForActuators.h"
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Model/CMCActuatorSubsystem.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Kinematics.h>
#include "ForwardTool.h"
#include <OpenSim/Common/DebugUtilities.h>
#include "CMC.h" 
#include "CMC_TaskSet.h"
#include "ActuatorForceTarget.h"
#include "ActuatorForceTargetFast.h"

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
CMCTool::~CMCTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CMCTool::CMCTool() :
    AbstractTool(),
    _excludedActuators(_excludedActuatorsProp.getValueStrArray()),
    _desiredPointsFileName(_desiredPointsFileNameProp.getValueStr()),
    _desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
    //_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
    //_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
    _taskSetFileName(_taskSetFileNameProp.getValueStr()),
    _constraintsFileName(_constraintsFileNameProp.getValueStr()),
    _rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    //_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
    _targetDT(_targetDTProp.getValueDbl()),  	 	 
    //_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
    _useFastTarget(_useFastTargetProp.getValueBool()),
    _optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
    _numericalDerivativeStepSize(_numericalDerivativeStepSizeProp.getValueDbl()),
    _optimizationConvergenceTolerance(_optimizationConvergenceToleranceProp.getValueDbl()),
    _maxIterations(_maxIterationsProp.getValueInt()),
    _printLevel(_printLevelProp.getValueInt()),
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
CMCTool::CMCTool(const string &aFileName, bool aLoadModel) :
    AbstractTool(aFileName, false),
    _excludedActuators(_excludedActuatorsProp.getValueStrArray()),
    _desiredPointsFileName(_desiredPointsFileNameProp.getValueStr()),
    _desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
    //_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
    //_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
    _taskSetFileName(_taskSetFileNameProp.getValueStr()),
    _constraintsFileName(_constraintsFileNameProp.getValueStr()),
    _rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    //_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
    _targetDT(_targetDTProp.getValueDbl()),  	 	 
    //_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
    _useFastTarget(_useFastTargetProp.getValueBool()),
    _optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
    _numericalDerivativeStepSize(_numericalDerivativeStepSizeProp.getValueDbl()),
    _optimizationConvergenceTolerance(_optimizationConvergenceToleranceProp.getValueDbl()),
    _maxIterations(_maxIterationsProp.getValueInt()),
    _printLevel(_printLevelProp.getValueInt()),
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
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for a Tool:
 *
 * 1) Construction based on XML file (@see Tool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Tool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
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
CMCTool::
CMCTool(const CMCTool &aTool) :
    AbstractTool(aTool),
    _excludedActuators(_excludedActuatorsProp.getValueStrArray()),
    _desiredPointsFileName(_desiredPointsFileNameProp.getValueStr()),
    _desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
    //_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
    //_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
    _taskSetFileName(_taskSetFileNameProp.getValueStr()),
    _constraintsFileName(_constraintsFileNameProp.getValueStr()),
    _rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    //_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
    _targetDT(_targetDTProp.getValueDbl()),  	 	 
    //_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
    _useFastTarget(_useFastTargetProp.getValueBool()),
    _optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
    _numericalDerivativeStepSize(_numericalDerivativeStepSizeProp.getValueDbl()),
    _optimizationConvergenceTolerance(_optimizationConvergenceToleranceProp.getValueDbl()),
    _maxIterations(_maxIterationsProp.getValueInt()),
    _printLevel(_printLevelProp.getValueInt()),
    _verbose(_verboseProp.getValueBool())
{
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void CMCTool::
setNull()
{
    setupProperties();

    _desiredPointsFileName = "";
    _desiredKinematicsFileName = "";
    //_externalLoadsFileName = "";
    //_externalLoadsModelKinematicsFileName = "";
    _taskSetFileName = "";
    _constraintsFileName = "";
    _rraControlsFileName = "";
    _lowpassCutoffFrequency = -1.0;
    //_lowpassCutoffFrequencyForLoadKinematics = -1.0;
    _targetDT = 0.010;  	 	 
    //_useCurvatureFilter = false; 		 
    _useFastTarget = true;
    _optimizerAlgorithm = "ipopt";
    _numericalDerivativeStepSize = 1.0e-4;
    _optimizationConvergenceTolerance = 1.0e-4;
    _maxIterations = 1000;
    _printLevel = 0;
    _verbose = false;

    _replaceForceSet = false;   // default should be false for Forward.
    _solveForEquilibriumForAuxiliaryStates = true;

}
//_____________________________________________________________________________
/**
 * Give this object's properties their XML names and add them to the property
 * list held in class Object (@see OpenSim::Object).
 */
void CMCTool::setupProperties()
{
    string comment;

    _excludedActuatorsProp.setComment("List of individual Actuators by individual or user-defined group name "
        " to be excluded from CMC's control.");
    _excludedActuatorsProp.setName("actuators_to_exclude");
    _propertySet.append(&_excludedActuatorsProp);

    comment = "Motion (.mot) or storage (.sto) file containing the desired point trajectories.";
    _desiredPointsFileNameProp.setComment(comment);
    _desiredPointsFileNameProp.setName("desired_points_file");
    _propertySet.append( &_desiredPointsFileNameProp );

    comment = "Motion (.mot) or storage (.sto) file containing the desired kinematic trajectories.";
    _desiredKinematicsFileNameProp.setComment(comment);
    _desiredKinematicsFileNameProp.setName("desired_kinematics_file");
    _propertySet.append( &_desiredKinematicsFileNameProp );

    //comment = "Motion file (.mot) or storage file (.sto) containing the model kinematics corresponding to the external loads.";
    //_externalLoadsModelKinematicsFileNameProp.setComment(comment);
    //_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file");
    //_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

    comment = "File containing the tracking tasks. Which coordinates are tracked and with what weights are specified here.";  	 	 
    _taskSetFileNameProp.setComment(comment); 		 
    _taskSetFileNameProp.setName("task_set_file"); 		 
    _propertySet.append( &_taskSetFileNameProp );

    comment = "File containing the constraints on the controls.";
    _constraintsFileNameProp.setComment(comment);
    _constraintsFileNameProp.setName("constraints_file");
    _propertySet.append( &_constraintsFileNameProp );

    comment = "File containing the controls output by RRA.";
    comment += " These can be used to place constraints on the residuals during CMC.";
    _rraControlsFileNameProp.setComment(comment);
    _rraControlsFileNameProp.setName("rra_controls_file");
    _propertySet.append( &_rraControlsFileNameProp );

    comment = "Low-pass cut-off frequency for filtering the desired kinematics.";
    comment += " A negative value results in no filtering. The default value is -1.0, so no filtering.";
    _lowpassCutoffFrequencyProp.setComment(comment);
    _lowpassCutoffFrequencyProp.setName("lowpass_cutoff_frequency");
    _propertySet.append( &_lowpassCutoffFrequencyProp );

    //comment = "Low-pass cut-off frequency for filtering the model kinematics corresponding ";
    //comment += "to the external loads. A negative value results in no filtering. ";
    //comment += "The default value is -1.0, so no filtering.";
    //_lowpassCutoffFrequencyForLoadKinematicsProp.setComment(comment);
    //_lowpassCutoffFrequencyForLoadKinematicsProp.setName("lowpass_cutoff_frequency_for_load_kinematics");
    //_propertySet.append( &_lowpassCutoffFrequencyForLoadKinematicsProp );

    comment = "Time window over which the desired actuator forces are achieved. "  	 	 
                    "Muscles forces cannot change instantaneously, so a finite time window must be allowed. " 		 
                       "The recommended time window for RRA is about 0.001 sec, and for CMC is about 0.010 sec."; 		 
    _targetDTProp.setComment(comment); 		 
    _targetDTProp.setName("cmc_time_window"); 		 
    _propertySet.append( &_targetDTProp ); 		 
         
    comment = "Flag (true or false) indicating whether or not to use the curvature filter. " 		 
                       "Setting this flag to true can reduce oscillations in the computed muscle excitations."; 		 
    //_useCurvatureFilterProp.setComment(comment); 		 
    //_useCurvatureFilterProp.setName("use_curvature_filter"); 		 
    //_propertySet.append( &_useCurvatureFilterProp );

    comment = "Flag (true or false) indicating whether to use the fast CMC optimization target. ";  	 	 
    comment += "The fast target requires the desired accelerations to be met. "; 		 
    comment += "The optimizer fails if the acclerations constraints cannot be "; 		 
    comment += "met, so the fast target can be less robust.  The regular target "; 		 
    comment += "does not require the acceleration constraints to be met; it "; 		 
    comment += "meets them as well as it can, but it is slower and less accurate."; 		 
    _useFastTargetProp.setComment(comment); 		 
    _useFastTargetProp.setName("use_fast_optimization_target"); 		 
    _propertySet.append( &_useFastTargetProp );

    comment = "Preferred optimizer algorithm (currently support \"ipopt\" or \"cfsqp\", "
                 "the latter requiring the osimFSQP library.";
    _optimizerAlgorithmProp.setComment(comment);
    _optimizerAlgorithmProp.setName("optimizer_algorithm");
    _propertySet.append( &_optimizerAlgorithmProp );

    comment = "Step size used by the optimizer to compute numerical derivatives. "
                 "A value between 1.0e-4 and 1.0e-8 is usually appropriate.";
    _numericalDerivativeStepSizeProp.setComment(comment);
    _numericalDerivativeStepSizeProp.setName("numerical_derivative_step_size");
    _propertySet.append( &_numericalDerivativeStepSizeProp );

    comment = "Convergence tolerance for the optimizer. The smaller this value, the deeper the convergence. "
                 "Decreasing this number can improve a solution, but will also likely increase computation time.";
    _optimizationConvergenceToleranceProp.setComment(comment);
    _optimizationConvergenceToleranceProp.setName("optimization_convergence_tolerance");
    _propertySet.append( &_optimizationConvergenceToleranceProp );

    comment = "Maximum number of iterations for the optimizer.";
    _maxIterationsProp.setComment(comment);
    _maxIterationsProp.setName("optimizer_max_iterations");
    _propertySet.append( &_maxIterationsProp );

    comment = "Print level for the optimizer, 0 - 3. 0=no printing, 3=detailed printing, 2=in between";
    _printLevelProp.setComment(comment);
    _printLevelProp.setName("optimizer_print_level");
    _propertySet.append( &_printLevelProp );

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
CMCTool& CMCTool::
operator=(const CMCTool &aTool)
{
    // BASE CLASS
    AbstractTool::operator=(aTool);

    // MEMEBER VARIABLES
    _excludedActuators = aTool._excludedActuators;
    _desiredPointsFileName = aTool._desiredPointsFileName;
    _desiredKinematicsFileName = aTool._desiredKinematicsFileName;
    //_externalLoadsFileName = aTool._externalLoadsFileName;
    //_externalLoadsModelKinematicsFileName = aTool._externalLoadsModelKinematicsFileName;
    _taskSetFileName = aTool._taskSetFileName;
    _constraintsFileName = aTool._constraintsFileName;
    _rraControlsFileName = aTool._rraControlsFileName;
    _lowpassCutoffFrequency = aTool._lowpassCutoffFrequency;
    //_lowpassCutoffFrequencyForLoadKinematics = aTool._lowpassCutoffFrequencyForLoadKinematics;
    _targetDT = aTool._targetDT;  	 	 
    //_useCurvatureFilter = aTool._useCurvatureFilter;
    _numericalDerivativeStepSize = aTool._numericalDerivativeStepSize;
    _optimizationConvergenceTolerance = aTool._optimizationConvergenceTolerance;
    _useFastTarget = aTool._useFastTarget;
    _optimizerAlgorithm = aTool._optimizerAlgorithm;
    _maxIterations = aTool._maxIterations;
    _printLevel = aTool._printLevel;
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
bool CMCTool::run()
{
    cout<<"Running tool "<<getName()<<".\n";

    // CHECK FOR A MODEL
    if(_model==NULL) {
        string msg = "ERROR- A model has not been set.";
        cout<<endl<<msg<<endl;
        throw(Exception(msg,__FILE__,__LINE__));
    }
    // OUTPUT DIRECTORY
    // Do the maneuver to change then restore working directory 
    // so that the parsing code behaves prope()rly if called from a different directory
    string saveWorkingDirectory = IO::getCwd();
    string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
    IO::chDir(directoryOfSetupFile);

    try {

    // SET OUTPUT PRECISION
    IO::SetPrecision(_outputPrecision);

    bool externalLoads = createExternalLoads(_externalLoadsFileName, *_model);

    CMC_TaskSet taskSet(_taskSetFileName);  	 	 
    //taskSet.print("cmcTasksRT.xml");
    cout<<"\n\n taskSet size = "<<taskSet.getSize()<<endl<<endl; 		 

    CMC* controller = new CMC(_model,&taskSet);	// Need to make it a pointer since Model takes ownership 
    controller->setName( "CMC" );

    // Don't automatically give CMC all the model actuators
    // Check to see if user has explicitly listed Actuators to be
    // excluded from CMC's control.

    controller->setActuators(getActuatorsForCMC(_excludedActuators));
    _model->addController(controller );
    controller->setDisabled(false);
    controller->setUseCurvatureFilter(false);
    controller->setTargetDT(_targetDT);
    controller->setCheckTargetTime(true);

    //Make sure system is uptodate with model (i.e. added actuators, etc...)
    SimTK::State& s = _model->initSystem();
    _model->getMultibodySystem().realize(s, Stage::Position );
     taskSet.setModel(*_model);
    _model->equilibrateMuscles(s);
  
    // ---- INPUT ----
    // DESIRED POINTS AND KINEMATICS
    if(_desiredPointsFileName=="" && _desiredKinematicsFileName=="") {
        cout<<"ERROR- a desired points file and desired kinematics file were not specified.\n\n";
        IO::chDir(saveWorkingDirectory);
        return false;
    }

    Storage *desiredPointsStore=NULL;
    bool desiredPointsFlag = false;
    if(_desiredPointsFileName=="") {
        cout<<"\n\nWARN- a desired points file was not specified.\n\n";
    } else {
        cout<<"\n\nLoading desired points from file "<<_desiredPointsFileName<<" ...\n";
        desiredPointsStore = new Storage(_desiredPointsFileName);
        desiredPointsFlag = true;
    }

    Storage *desiredKinStore=NULL;
    bool desiredKinFlag = false;
    if(_desiredKinematicsFileName=="") {
        cout<<"\n\nWARN- a desired kinematics file was not specified.\n\n";
    } else {
        cout<<"\n\nLoading desired kinematics from file "<<_desiredKinematicsFileName<<" ...\n";
        desiredKinStore = new Storage(_desiredKinematicsFileName);
        desiredKinFlag = true;
    }

    // ---- INITIAL AND FINAL TIME ----
    // NOTE: important to do this before padding (for filtering)
    // Initial Time
    if(desiredPointsFlag) {
        double ti = desiredPointsStore->getFirstTime();
        if(_ti<ti) {
            cout<<"\nThe initial time set for the cmc run precedes the first time\n";
            cout<<"in the desired points file "<<_desiredPointsFileName<<".\n";
            cout<<"Resetting the initial time from "<<_ti<<" to "<<ti<<".\n\n";
            _ti = ti;
        }
        // Final time
        double tf = desiredPointsStore->getLastTime();
        if(_tf>tf) {
            cout<<"\n\nWARN- The final time set for the cmc run is past the last time stamp\n";
            cout<<"in the desired points file "<<_desiredPointsFileName<<".\n";
            cout<<"Resetting the final time from "<<_tf<<" to "<<tf<<".\n\n";
            _tf = tf;
        }
    }

    // Initial Time
    if(desiredKinFlag) {
        double ti = desiredKinStore->getFirstTime();
        if(_ti<ti) {
            cout<<"\nThe initial time set for the cmc run precedes the first time\n";
            cout<<"in the desired kinematics file "<<_desiredKinematicsFileName<<".\n";
            cout<<"Resetting the initial time from "<<_ti<<" to "<<ti<<".\n\n";
            _ti = ti;
        }
        // Final time
        double tf = desiredKinStore->getLastTime();
        if(_tf>tf) {
            cout<<"\n\nWARN- The final time set for the cmc run is past the last time stamp\n";
            cout<<"in the desired kinematics file "<<_desiredKinematicsFileName<<".\n";
            cout<<"Resetting the final time from "<<_tf<<" to "<<tf<<".\n\n";
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
            cout<<"\n\nLow-pass filtering desired points with a cutoff frequency of ";
            cout<<_lowpassCutoffFrequency<<"...";
            desiredPointsStore->lowpassFIR(order,_lowpassCutoffFrequency);
        } else {
            cout<<"\n\nNote- not filtering the desired points.\n\n";
        }
    }

    if(desiredKinFlag) {
        desiredKinStore->pad(60);
        if (_verbose) desiredKinStore->print("desiredKinematics_padded.sto");
        if(_lowpassCutoffFrequency>=0) {
            int order = 50;
            cout<<"\n\nLow-pass filtering desired kinematics with a cutoff frequency of ";
            cout<<_lowpassCutoffFrequency<<"...\n\n";
            desiredKinStore->lowpassFIR(order,_lowpassCutoffFrequency);
        } else {
            cout<<"\n\nNote- not filtering the desired kinematics.\n\n";
        }
    }

     // TASK SET
    if(_taskSetFileName=="") {  	 	 
        cout<<"ERROR- a task set was not specified\n\n"; 		 
        IO::chDir(saveWorkingDirectory); 		 
        return false; 		 
    }

    _model->printDetailedInfo(s, cout);

    int nq = _model->getNumCoordinates();
    int nu = _model->getNumSpeeds();
    int na = controller->getActuatorSet().getSize();

    // Form complete storage objects for the q's and u's
    // This means filling in unspecified generalized coordinates and
    // setting constrained coordinates to their valid values.
    Storage *qStore=NULL;
    Storage *uStore=NULL;

    if(desiredKinFlag) {
        _model->getMultibodySystem().realize(s, Stage::Time );
        _model->getSimbodyEngine().formCompleteStorages(s, *desiredKinStore,qStore,uStore);
        if(qStore->isInDegrees()){
            _model->getSimbodyEngine().convertDegreesToRadians(*qStore);
            _model->getSimbodyEngine().convertDegreesToRadians(*uStore);
        }
    }

    // Spline
    GCVSplineSet *posSet=NULL;
    if(desiredPointsFlag) {
        cout<<"\nConstructing function set for tracking desired points...\n\n";
        posSet = new GCVSplineSet(5,desiredPointsStore);

        Storage *velStore=posSet->constructStorage(1);
        GCVSplineSet velSet(5,velStore);
        delete velStore; velStore=NULL;

        // Print acc for debugging
        Storage *accStore=posSet->constructStorage(2);
        accStore->print("desiredPoints_splinefit_accelerations.sto");
        delete accStore; accStore=NULL;	
    }

    GCVSplineSet *qSet=NULL;
    GCVSplineSet *uSet=NULL;
    GCVSplineSet *uDotSet=NULL;

    if(desiredKinFlag) {
        cout<<"\nConstructing function set for tracking desired kinematics...\n\n";
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
    }

    // RRA CONTROLS
    ControlSet *rraControlSet = constructRRAControlSet(controlConstraints);

    // ---- INITIAL STATES ----
    Array<double> q(0.0,nq);
    Array<double> u(0.0,nu);
    if(desiredKinFlag) {
        cout<<"Using the generalized coordinates specified in "<<_desiredKinematicsFileName;
        cout<<" to set the initial configuration.\n" << endl;
        qSet->evaluate(q,0,_ti);
        uSet->evaluate(u,0,_ti);
    } else {
        cout<<"Using the generalized coordinates specified as zeros ";
        cout<<" to set the initial configuration.\n" << endl;
    }

    for(int i=0;i<nq;i++) s.updQ()[i] = q[i];
    for(int i=0;i<nu;i++) s.updU()[i] = u[i];

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
    const SimTK::Vector &modelZ = _model->getMultibodySystem()
                                            .getDefaultSubsystem().getZ(s);
    
    int nra = actSysZ.size();
    int nrm = modelZ.size();

    assert(nra == nrm);
    actSysZ = modelZ;

    VectorFunctionForActuators *predictor =
        new VectorFunctionForActuators(&actuatorSystem, _model, &cmcActSubsystem);

    controller->setActuatorForcePredictor(predictor);
    controller->updTaskSet().setFunctions(*qAndPosSet);

    // Optimization target
    OptimizationTarget *target = NULL;
    if(_useFastTarget) {
        target = new ActuatorForceTargetFast(s, na,controller);
    } else {
        target = new ActuatorForceTarget(na,controller);
    }
    target->setDX(_numericalDerivativeStepSize);

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

    SimTK::Optimizer *optimizer = new SimTK::Optimizer(*target, algorithm);
    controller->setOptimizationTarget(target, optimizer);

    cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
    optimizer->setDiagnosticsLevel(_printLevel);
    cout<<"Setting optimizer convergence tolerance to "<<_optimizationConvergenceTolerance<<".\n";
    optimizer->setConvergenceTolerance(_optimizationConvergenceTolerance);
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

    if(_verbose) cout<<"\nSetting cmc controller to use verbose printing."<<endl;
    else cout<<"\nSetting cmc controller to not use verbose printing."<<endl;
    controller->setUseVerbosePrinting(_verbose);

    controller->setCheckTargetTime(true);

    // ---- SIMULATION ----
    //
    // Manager
    RungeKuttaMersonIntegrator integrator(_model->getMultibodySystem());
    integrator.setMaximumStepSize(_maxDT);
    integrator.setMinimumStepSize(_minDT);
    integrator.setAccuracy(_errorTolerance);
    Manager manager(*_model, integrator);
    
    _model->setAllControllersEnabled( true );

    manager.setSessionName(getName());
    manager.setInitialTime(_ti);
    manager.setFinalTime(_tf-_targetDT-SimTK::Zero);

    // Initialize integrand controls using controls read in from file (which specify min/max control values)
    initializeControlSetUsingConstraints(rraControlSet,controlConstraints, controller->updControlSet());

    // Initial auxilliary states
    time_t startTime,finishTime;
    struct tm *localTime;
    double elapsedTime;
    if( s.getNZ() > 0) { // If there are actuator states (i.e. muscles dynamics)
        cout<<"\n\n\n";
        cout<<"================================================================\n";
        cout<<"================================================================\n";
        cout<<"Computing initial values for muscles states (activation, length)\n";
        time(&startTime);
        localTime = localtime(&startTime);
        cout<<"Start time = "<<asctime(localTime);
        cout<<"\n================================================================\n";
        try {
        controller->computeInitialStates(s,_ti);
        }
        catch(const Exception& x) {
        // TODO: eventually might want to allow writing of partial results
            x.print(cout);
            IO::chDir(saveWorkingDirectory);
            return false;
        }
        catch(...) {
            // TODO: eventually might want to allow writing of partial results
            IO::chDir(saveWorkingDirectory);
            // close open files if we die prematurely (e.g. Opt fail)
            return false;
        }
        time(&finishTime);
        cout<<endl;
        // copy the final states from the last integration 
        s.updY() = cmcActSubsystem.getCompleteState().getY();
        cout<<"-----------------------------------------------------------------\n";
        cout<<"Finished computing initial states:\n";
        cout<<"-----------------------------------------------------------------\n";
        cout<<"=================================================================\n";
        localTime = localtime(&startTime);
        cout<<"Start time   = "<<asctime(localTime);
        localTime = localtime(&finishTime);
        cout<<"Finish time  = "<<asctime(localTime);
        elapsedTime = difftime(finishTime,startTime);
        cout<<"Elapsed time = "<<elapsedTime<<" seconds.\n";
        cout<<"=================================================================\n";
    } else {
        cmcActSubsystem.setCompleteState( s );
        actuatorSystemState.updTime() = _ti; 
        s.updTime() = _ti;
        actuatorSystem.realize(actuatorSystemState, Stage::Time );
        controller->setTargetDT(1.0e-8);
        controller->computeControls( s, controller->updControlSet() );
        controller->setTargetDT(_targetDT);
    }
    manager.setInitialTime(_ti);

    // ---- INTEGRATE ----
    cout<<"\n\n\n";
    cout<<"================================================================\n";
    cout<<"================================================================\n";
    cout<<"Using CMC to track the specified kinematics\n";
    cout<<"Integrating from "<<_ti<<" to "<<_tf<<endl;
    s.updTime() = _ti;
    controller->setTargetTime( _ti );
    time(&startTime);
    localTime = localtime(&startTime);
    cout<<"Start time = "<<asctime(localTime);
    cout<<"================================================================\n";

    _model->getMultibodySystem().realize(s, Stage::Acceleration );

    controller->updTaskSet().computeAccelerations(s);

    cmcActSubsystem.setCompleteState( s );

    // Set output file names so that files are flushed regularly in case we fail
    IO::makeDir(getResultsDir());	// Create directory for output in case it doesn't exist
    manager.getStateStorage().setOutputFileName(getResultsDir() + "/" + getName() + "_states.sto");
    try {
        manager.integrate(s);
    }
    catch(const Exception& x) {
        // TODO: eventually might want to allow writing of partial results
        x.print(cout);
        IO::chDir(saveWorkingDirectory);
        // close open files if we die prematurely (e.g. Opt fail)
        manager.getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");
        return false;
    }
    catch(...) {
        // TODO: eventually might want to allow writing of partial results
        IO::chDir(saveWorkingDirectory);
        // close open files if we die prematurely (e.g. Opt fail)
        manager.getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");
        return false;
    }
    time(&finishTime);
    cout<<"----------------------------------------------------------------\n";
    cout<<"Finished tracking the specified kinematics\n";
    if( _verbose ){
      std::cout << "states= " << s.getY() << std::endl;
    }
    cout<<"=================================================================\n";
    localTime = localtime(&startTime);
    cout<<"Start time   = "<<asctime(localTime);
    localTime = localtime(&finishTime);
    cout<<"Finish time  = "<<asctime(localTime);
    elapsedTime = difftime(finishTime,startTime);
    cout<<"Elapsed time = "<<elapsedTime<<" seconds.\n";
    cout<<"================================================================\n\n\n";

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

    //_model->removeController(controller); // So that if this model is from GUI it doesn't double-delete it.

    } catch(const Exception& x) {
        // TODO: eventually might want to allow writing of partial results
        x.print(cout);
        IO::chDir(saveWorkingDirectory);
        // close open files if we die prematurely (e.g. Opt fail)
        
        return false;
    }

    IO::chDir(saveWorkingDirectory);

    return true;
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Add Actuation/Kinematics analyses if necessary
 */
void CMCTool::
addNecessaryAnalyses()
{
    int stepInterval = 1;
    AnalysisSet& as = _model->updAnalysisSet();
    // Add Actuation if necessary
    Actuation *act = NULL;
    for(int i=0; i<as.getSize(); i++) 
        if(as.get(i).getConcreteClassName() == "Actuation") { act = (Actuation*)&as.get(i); break; }
    if(!act) {
        std::cout << "No Actuation analysis found in analysis set -- adding one" << std::endl;
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
        std::cout << "No Kinematics analysis found in analysis set -- adding one" << std::endl;
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
void CMCTool::
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
            cout<<"Set "<<rraControlName<<" to use linear interpolation.\n";
        }
#endif
    }	
}
//_____________________________________________________________________________
/**
 * Create a set of control constraints based on an RRA solution.
 * If RRA (Residual Reduction Algorithm) was run to compute or reduce the 
 * residuals as a preprocessing step, those residuals need to be applied 
 * during the CMC run.  They are applied by reading in the residuals computed
 * during RRA and then using these controls to place narrow constraints on 
 * the controls for the residual actuators active during the CMC run.
 *
 * @param aControlConstraints Constraints based on a previous RRA solution
 * to be used during this run of CMC.
 */
ControlSet* CMCTool::
constructRRAControlSet(ControlSet *aControlConstraints)
{
    if(_rraControlsFileName=="") return(NULL);
    
    OPENSIM_FUNCTION_NOT_IMPLEMENTED();
    // Need to make sure code below still works after changes to controls/control constraints
#if 0
    int i;
    ControlLinear *controlConstraint=NULL;
    string rraControlName,cmcControlName;
    ControlLinear *rraControl=NULL;

    // LOAD RRA CONTROLS
    ControlSet *rraControlSet=NULL;
    rraControlSet = new ControlSet(_rraControlsFileName);

    // Loop through controls looking for corresponding actuators
    int nrra = rraControlSet->getSize();
    for(i=0;i<nrra;i++) {
        rraControl = (ControlLinear*)&rraControlSet->get(i);
        if(rraControl==NULL) continue;
        rraControlName = rraControl->getName();

        // Does control exist in the model?
        // TODO- We are not using this functionality at the moment,
        // so I'm going to comment this out.
        //int index = _model->getControlIndex(rraControlName);
        int index = 0;

        // Add a constraint based on the rra control 
        if(index>=0) {

            // Create control constraint set if necessary
            if(aControlConstraints==NULL) {
                aControlConstraints = new ControlSet();
                aControlConstraints->setName("ControlConstraints");
            }

            // Get control constraint
            controlConstraint = (ControlLinear*)&&aControlConstraints->get(rraControlName);
            // Control constraint already exists, so clear the existing nodes
            if(controlConstraint!=NULL) {
                    controlConstraint->getNodeArray().setSize(0);
            // Make a new control constraint
            } else {
                controlConstraint = new ControlLinear();
                controlConstraint->setName(rraControlName);
                aControlConstraints->append(controlConstraint);
            }

            // Set max and min values
            ArrayPtrs<ControlLinearNode> &nodes = rraControl->getNodeArray();
            int j,nnodes = nodes.getSize();
            double t,x,max,min,dx;
            for(j=0;j<nnodes;j++) {
                t = nodes[j]->getTime();
                x = nodes[j]->getValue();
                dx = 1.00 * (nodes[j]->getMax() - nodes[j]->getMin());
                max = x + dx;
                min = x - dx;
                controlConstraint->setControlValue(t,x);
                controlConstraint->setControlValueMax(t,max);
                controlConstraint->setControlValueMin(t,min);
                //cout<<controlConstraint->getName()<<": t="<<t<<" min="<<min<<" x="<<x<<" max="<<max<<endl;
            }
        }
    }

    // Print out modified control constraints
    //if(aControlConstraints!=NULL) aControlConstraints->print("cmc_aControlConstraints_check.xml");

    return(rraControlSet);
#endif
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
Storage& CMCTool::getForceStorage(){
        Actuation& actuation = (Actuation&)_model->getAnalysisSet().get("Actuation");
        return *actuation.getForceStorage();
}

void CMCTool::setOriginalForceSet(const ForceSet &aForceSet) {
    _originalForceSet = aForceSet;
}


/* Get the Set of model actuators for CMC that exclude user specified Actuators */
Set<Actuator> CMCTool::
    getActuatorsForCMC(const Array<std::string> &actuatorsByNameOrGroup)
{	
    Set<Actuator> actuatorsForCMC = _model->getActuators();
    for (int i=actuatorsForCMC.getSize()-1; i>0; i--){
        if (actuatorsForCMC.get(i).get_isDisabled())
            actuatorsForCMC.remove(i);
    }
    Array<string> groupNames;
    actuatorsForCMC.getGroupNames(groupNames);

    /* The search for inidividual group or force names IS case-sensitive BUT keywords are not*/
    for(int i=0; i<actuatorsByNameOrGroup.getSize(); i++){
        // index result when a name is not a force or group name for forces in the model
        int k = -1;  
        // It is possible to list actuators by name and group
        // So check if name is a group name first	
        if(groupNames.getSize() > 0){
            k = groupNames.findIndex(actuatorsByNameOrGroup[i]);
            if(k > -1){ //found
                const ObjectGroup* group = actuatorsForCMC.getGroup(k);
                Array<Object*> members = group->getMembers();
                for(int j=0; j<members.getSize(); j++)
                    actuatorsForCMC.remove((Actuator *)members[j]);
                actuatorsForCMC.removeGroup(actuatorsByNameOrGroup[i]);
            }
        } //otherwise, check for an individual acuator
        else{
            k = actuatorsForCMC.getIndex(actuatorsByNameOrGroup[i]);
            if(k > -1){ //found
                actuatorsForCMC.remove(k);
            }
        }
        // No actuator(s) by group or individual name was found
        if(k < 0)
            cout << "\nCMCTool::WARNING could not find actuator or group named '" << actuatorsByNameOrGroup[i] << "' to be excluded from CMC." << endl;

    }

    return actuatorsForCMC;
}