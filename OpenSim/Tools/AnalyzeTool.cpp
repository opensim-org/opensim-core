/* -------------------------------------------------------------------------- *
 *                         OpenSim:  AnalyzeTool.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Common/XMLDocument.h>
#include "AnalyzeTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/GCVSplineSet.h>

#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Analyses/MuscleAnalysis.h>
#include <OpenSim/Analyses/ProbeReporter.h>
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
AnalyzeTool::~AnalyzeTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AnalyzeTool::AnalyzeTool() :
    AbstractTool(),
    _statesFileName(_statesFileNameProp.getValueStr()),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _speedsFileName(_speedsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _loadModelAndInput(false),
    _printResultFiles(true)
{
    setNull();
}
//_____________________________________________________________________________
/**
 * Construct from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
AnalyzeTool::AnalyzeTool(const string &aFileName, bool aLoadModelAndInput) :
    AbstractTool(aFileName, false),
    _statesFileName(_statesFileNameProp.getValueStr()),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _speedsFileName(_speedsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _loadModelAndInput(aLoadModelAndInput),
    _printResultFiles(true)
{
    setNull();
    updateFromXMLDocument();

    if(aLoadModelAndInput) {
        loadModel(aFileName);
        // Append to or replace model forces with those (i.e. actuators) specified by the analysis
        updateModelForces(*_model, aFileName);
        setModel(*_model);  
        setToolOwnsModel(true);

    }
}
//_____________________________________________________________________________
/**
 * Construct with a passed in model.
 *
 * Typically used from the GUI where the model is readily available.
 * This special constructor avoid many steps/generalities in th AnalyzeTool 
 * Analyses are added to the model beforehand.
 *
 * @param aModel model in the GUI.
 * 
 */
AnalyzeTool::AnalyzeTool(Model& aModel) :
    AbstractTool(),
    _statesFileName(_statesFileNameProp.getValueStr()),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _speedsFileName(_speedsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _loadModelAndInput(false),
    _printResultFiles(true)
{
    setNull();
    setModel(aModel);
    // By default add a muscleAnalysis and a MomentArmAnalysis and turn them off if they
    // they have not been included already
    AnalysisSet& analysisSet = aModel.updAnalysisSet();
    if (analysisSet.getIndex("MuscleAnalysis")==-1){
        MuscleAnalysis* muscleAnalysis = new MuscleAnalysis(&aModel);
        muscleAnalysis->setOn(false);
        aModel.addAnalysis(muscleAnalysis);
        //this->getAnalysisSet().append(muscleAnalysis);
    }
    
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Tools only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Tool:
 *
 * 1) Construction based on XML file (@see Tool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Tool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
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
AnalyzeTool::
AnalyzeTool(const AnalyzeTool &aTool) :
    AbstractTool(aTool),
    _statesFileName(_statesFileNameProp.getValueStr()),
    _coordinatesFileName(_coordinatesFileNameProp.getValueStr()),
    _speedsFileName(_speedsFileNameProp.getValueStr()),
    _lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
    _loadModelAndInput(false)
{
    setNull();
    *this = aTool;
}


//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void AnalyzeTool::
setNull()
{
    setupProperties();

    _statesFileName = "";
    _coordinatesFileName = "";
    _speedsFileName = "";
    _lowpassCutoffFrequency = -1.0;

    _statesStore = NULL;

    _printResultFiles = true;
    _replaceForceSet = false;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AnalyzeTool::setupProperties()
{
    string comment;


    comment = "Storage file (.sto) containing the time history of states for the model. "
                 "This file often contains multiple rows of data, each row being a time-stamped array of states. "
                 "The first column contains the time.  The rest of the columns contain the states in the order "
                 "appropriate for the model. In a storage file, unlike a motion file (.mot), non-uniform time spacing "
                 "is allowed.  If the user-specified initial time for a simulation does not correspond exactly to "
                 "one of the time stamps in this file, inerpolation is NOT used because it is sometimes necessary to "
                 "an exact set of states for analyses.  Instead, the closest earlier set of states is used.";
    _statesFileNameProp.setComment(comment);
    _statesFileNameProp.setName("states_file");
    _propertySet.append( &_statesFileNameProp );

    comment = "Motion file (.mot) or storage file (.sto) containing the time history of the generalized coordinates for the model. "
                 "These can be specified in place of the states file.";
    _coordinatesFileNameProp.setComment(comment);
    _coordinatesFileNameProp.setName("coordinates_file");
    _propertySet.append( &_coordinatesFileNameProp );

    comment = "Storage file (.sto) containing the time history of the generalized speeds for the model. "
                 "If coordinates_file is used in place of states_file, these can be optionally set as well to give the speeds. "
                 "If not specified, speeds will be computed from coordinates by differentiation.";
    _speedsFileNameProp.setComment(comment);
    _speedsFileNameProp.setName("speeds_file");
    _propertySet.append( &_speedsFileNameProp );

    comment = "Low-pass cut-off frequency for filtering the coordinates_file data (currently does not apply to states_file or speeds_file). "
                 "A negative value results in no filtering. The default value is -1.0, so no filtering.";
    _lowpassCutoffFrequencyProp.setComment(comment);
    _lowpassCutoffFrequencyProp.setName("lowpass_cutoff_frequency_for_coordinates");
    _propertySet.append( &_lowpassCutoffFrequencyProp );

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
AnalyzeTool& AnalyzeTool::
operator=(const AnalyzeTool &aTool)
{
    // BASE CLASS
    AbstractTool::operator=(aTool);

    // MEMEBER VARIABLES
    _statesFileName = aTool._statesFileName;
    _coordinatesFileName = aTool._coordinatesFileName;
    _speedsFileName = aTool._speedsFileName;
    _lowpassCutoffFrequency= aTool._lowpassCutoffFrequency;
    _statesStore = aTool._statesStore;
    _printResultFiles = aTool._printResultFiles;
    return(*this);
}


//_____________________________________________________________________________
/**
 * aUStore is optional.
 * Assumes coordinates and speeds are already in radians.
 * Fills in zeros for actuator and contact set states.
 */
Storage *AnalyzeTool::
createStatesStorageFromCoordinatesAndSpeeds(const Model& aModel, const Storage& aQStore, const Storage& aUStore)
{
    int nq = aModel.getNumCoordinates();
    int nu = aModel.getNumSpeeds();
    int ny = aModel.getNumStateVariables();

    if(aQStore.getSmallestNumberOfStates() != nq)
        throw Exception("AnalyzeTool.initializeFromFiles: ERROR- Coordinates storage does not have correct number of coordinates.",__FILE__,__LINE__);
    if(aUStore.getSmallestNumberOfStates() != nu)
        throw Exception("AnalyzeTool.initializeFromFiles: ERROR- Speeds storage does not have correct number of coordinates.",__FILE__,__LINE__);
    if(aQStore.getSize() != aUStore.getSize())
        throw Exception("AnalyzeTool.initializeFromFiles: ERROR- The coordinates storage and speeds storage should have the same number of rows, but do not.",__FILE__,__LINE__);

    Array<string> stateNames("", ny);
    Array<string> qLabels = aQStore.getColumnLabels();
    Array<string> uLabels = aUStore.getColumnLabels();
    stateNames = aModel.getStateVariableNames();
    stateNames.insert(0, "time");
    
    // Preserve the labels from the data file which are typically abreviated
    // label[0] = time
    for(int i=1; i<=nq; ++i){
        stateNames[i] = qLabels[i];
    }
    for(int i=1; i<=nu; ++i){
        stateNames[i+nq] = uLabels[i];
    }
    
    //Get the default state resulting from initializing the state after system creation
    const SimTK::State &s = aModel.getWorkingState();

    Storage *statesStore = new Storage(512,"states");
    statesStore->setColumnLabels(stateNames);
    Array<double> y(0.0,ny);

    // initialize the state storage from the default state so that states have relevant values
    // that are not zero (for example muscle activations and fiber-lengths)
    SimTK::Vector stateValues = aModel.getStateVariableValues(s);

    for(int index=0; index<aQStore.getSize(); index++) {
        double t;
        aQStore.getTime(index,t);
        aQStore.getData(index,nq,&y[0]);
        aUStore.getData(index,nu,&y[nq]);
        statesStore->append(t,ny,&y[0]);
    }

    return statesStore;
}

//_____________________________________________________________________________
/**
 * Set the states storage.  A states storage is required to run the analyze
 * tool.  The rows of a states storage consist of time-stamped vectors of
 * all the model states.  Time is in the first column and is assumed to
 * increasing monotonically.
 *
 * @param Pointer to storage file containing the time history of model
 * states.
 */
void AnalyzeTool::
setStatesStorage(Storage& aStore)
{
    _statesStore = &aStore;
}
//_____________________________________________________________________________
/**
 * Get the states storage.
 *
 * @return Pointer to the states storage.
 */
Storage& AnalyzeTool::
getStatesStorage()
{
    return *_statesStore;
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Initialize the controls, states, and external loads from
 * files.  The file names are stored in the property set.  The file names
 * can either come from the XML setup file, or they can be set explicitly.
 * Either way, this method should be called to read all the needed information
 * in from file.
 */
void AnalyzeTool::
loadStatesFromFile(SimTK::State& s)
{
    delete _statesStore; _statesStore = NULL;
    if(_statesFileNameProp.isValidFileName()) {
        if(_coordinatesFileNameProp.isValidFileName()) cout << "WARNING: Ignoring " << _coordinatesFileNameProp.getName() << " since " << _statesFileNameProp.getName() << " is also set" << endl;
        if(_speedsFileNameProp.isValidFileName()) cout << "WARNING: Ignoring " << _speedsFileNameProp.getName() << " since " << _statesFileNameProp.getName() << " is also set" << endl;
        cout<<"\nLoading states from file "<<_statesFileName<<"."<<endl;
        Storage temp(_statesFileName);
        _statesStore = new Storage();
        _model->formStateStorage(temp, *_statesStore);
    } else {
        if(!_coordinatesFileNameProp.isValidFileName()) 
            throw Exception("AnalyzeTool.initializeFromFiles: Either a states file or a coordinates file must be specified.",__FILE__,__LINE__);

        cout<<"\nLoading coordinates from file "<<_coordinatesFileName<<"."<<endl;
        Storage coordinatesStore(_coordinatesFileName);

        if(_lowpassCutoffFrequency>=0) {
            cout<<"\n\nLow-pass filtering coordinates data with a cutoff frequency of "<<_lowpassCutoffFrequency<<"..."<<endl<<endl;
            //coordinatesStore.pad(60);
            //coordinatesStore.lowpassFIR(50,_lowpassCutoffFrequency);
            //coordinatesStore.smoothSpline(5,_lowpassCutoffFrequency);
            coordinatesStore.pad(coordinatesStore.getSize()/2);
            coordinatesStore.lowpassIIR(_lowpassCutoffFrequency);
        }

        Storage *qStore=NULL, *uStore=NULL;

        _model->getSimbodyEngine().formCompleteStorages( s, coordinatesStore,qStore,uStore);

        if(_speedsFileName!="") {
            delete uStore;
            cout<<"\nLoading speeds from file "<<_speedsFileName<<"."<<endl;
            uStore = new Storage(_speedsFileName);
        }

        _model->getSimbodyEngine().convertDegreesToRadians(*qStore);
        _model->getSimbodyEngine().convertDegreesToRadians(*uStore);

        _statesStore = createStatesStorageFromCoordinatesAndSpeeds(*_model, *qStore, *uStore);

        delete qStore;
        delete uStore;
    }

    cout<<"Found "<<_statesStore->getSize()<<" state vectors with time stamps ranging "
         <<"from "<<_statesStore->getFirstTime()<<" to "<<_statesStore->getLastTime()<<"."<<endl;
}

void AnalyzeTool::
setStatesFromMotion(const SimTK::State& s, const Storage &aMotion, bool aInDegrees)
{
    cout<<endl<<"Creating states from motion storage"<<endl;

    // Make a copy in case we need to convert to degrees and/or filter
    Storage motionCopy(aMotion);

    if(!aInDegrees) _model->getSimbodyEngine().convertRadiansToDegrees(motionCopy);

    if(_lowpassCutoffFrequency>=0) {
        cout<<"\nLow-pass filtering coordinates data with a cutoff frequency of "<<_lowpassCutoffFrequency<<"..."<<endl;

        motionCopy.pad(motionCopy.getSize()/2);
        motionCopy.lowpassIIR(_lowpassCutoffFrequency);
    }

    Storage *qStore=NULL, *uStore=NULL;
    _model->getSimbodyEngine().formCompleteStorages(s,motionCopy,qStore,uStore);

    _model->getSimbodyEngine().convertDegreesToRadians(*qStore);
    _model->getSimbodyEngine().convertDegreesToRadians(*uStore);

    _statesStore = createStatesStorageFromCoordinatesAndSpeeds(*_model, *qStore, *uStore);

    delete qStore;
    delete uStore;
}
//_____________________________________________________________________________
/**
 * Verify that the controls and states are consistent with the
 * model.
 */
void AnalyzeTool::
verifyControlsStates()
{
    int ny = _model->getNumStateVariables();

    // DO WE HAVE STATES?
    // States
    if(_statesStore==NULL) {
        string msg = "analyzeTool.verifyControlsStates: ERROR- a storage object containing "
                            "the time histories of states was not specified.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    // States
    
    int NY = _statesStore->getSmallestNumberOfStates();
    if(NY!=ny) {
        string msg = "AnalyzeTool.verifyControlsStates: ERROR- Number of states in " + _statesFileName;
        msg += " doesn't match number of states in model " + _model->getName() + ".";
        throw Exception(msg,__FILE__,__LINE__);
    }
}

//_____________________________________________________________________________
/**
 * Turn On/Off writing result storages to files.
 */
void AnalyzeTool::
setPrintResultFiles(bool aToWrite)
{
    _printResultFiles=aToWrite;
}

void AnalyzeTool::
disableIntegrationOnlyProbes()
{
    AnalysisSet& analysisSet = _model->updAnalysisSet();

    for(int i=0;i<analysisSet.getSize();i++) {
        Analysis& an = analysisSet.get(i);
        if (an.getClassName()=="ProbeReporter"){
            ProbeReporter& pReporter = dynamic_cast<ProbeReporter&>(an);
            pReporter.disableIntegrationOnlyProbes();
        }
    }

}
//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the investigation.
 */
bool AnalyzeTool::run()
{
    return run(false);
}
bool AnalyzeTool::run(bool plotting)
{
    //cout<<"Running analyze tool "<<getName()<<"."<<endl;

    // CHECK FOR A MODEL
    if(_model==NULL) {
        string msg = "ERROR- A model has not been set.";
        cout<<endl<<msg<<endl;
        throw(Exception(msg,__FILE__,__LINE__));
    }

    // Use the Dynamics Tool API to handle external loads instead of outdated AbstractTool
    bool externalLoads = createExternalLoads(_externalLoadsFileName, *_model);

//printf("\nbefore AnalyzeTool.run() initSystem \n");
    // Call initSystem except when plotting
    SimTK::State& s = (!plotting)? _model->initSystem(): _model->updWorkingState();

    _model->getMultibodySystem().realize(s, SimTK::Stage::Position );
//printf("after AnalyzeTool.run() initSystem \n\n");

    if(_loadModelAndInput) {
        loadStatesFromFile(s);
    }


    // Do the maneuver to change then restore working directory 
    // so that the parsing code behaves properly if called from a different directory.
    string saveWorkingDirectory = IO::getCwd();
    if (getDocument())  // When the tool is created live from GUI it has no file/document association
        IO::chDir(IO::getParentDirectory(getDocumentFileName()));

    bool completed = true;

    try {

    // VERIFY THE CONTROL SET, STATES, AND PSEUDO STATES ARE TENABLE
    verifyControlsStates();

    // SET OUTPUT PRECISION
    IO::SetPrecision(_outputPrecision);

    // ANALYSIS SET
    AnalysisSet& analysisSet = _model->updAnalysisSet();
    if(analysisSet.getSize()<=0) {
        string msg = "AnalysisTool.run: ERROR- no analyses have been set.";
        throw Exception(msg,__FILE__,__LINE__);
    }

    // Call helper function to process analysis
    /*Array<double> bounds;
    bounds.append(_ti);
    bounds.append(_tf);
    const_cast<Storage &>(aStatesStore).interpolateAt(bounds);*/
    double ti,tf;
    int iInitial = _statesStore->findIndex(_ti);
    int iFinal = _statesStore->findIndex(_tf);
    _statesStore->getTime(iInitial,ti);
    _statesStore->getTime(iFinal,tf);

    // It is rediculous too start before the specified time! So check we aren't doing something stupid.
    //while(ti < _ti){
    //  _statesStore->getTime(++iInitial,ti);
    //}

    cout<<"Executing the analyses from "<<ti<<" to "<<tf<<"..."<<endl;
    run(s, *_model, iInitial, iFinal, *_statesStore, _solveForEquilibriumForAuxiliaryStates);
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position );
    } catch (const Exception& x) {
        x.print(cout);
        completed = false;
        IO::chDir(saveWorkingDirectory);
        throw Exception(x.what(),__FILE__,__LINE__);
    }

    // PRINT RESULTS
    // TODO: give option to write partial results if not completed
    if (completed && _printResultFiles)
        printResults(getName(),getResultsDir()); // this will create results directory if necessary

    IO::chDir(saveWorkingDirectory);

    return completed;
}

//=============================================================================
// HELPER
//=============================================================================
void AnalyzeTool::run(SimTK::State& s, Model &aModel, int iInitial, int iFinal, const Storage &aStatesStore, bool aSolveForEquilibrium)
{
    AnalysisSet& analysisSet = aModel.updAnalysisSet();

    for(int i=0;i<analysisSet.getSize();i++) {
        analysisSet.get(i).setStatesStore(aStatesStore);
    }

    // TODO: some sort of filtering or something to make derivatives smoother?
    GCVSplineSet statesSplineSet(5,&aStatesStore);

    // PERFORM THE ANALYSES
    double tPrev=0.0,t=0.0,dt=0.0;
    int ny = s.getNY();
    Array<double> dydt(0.0,ny);
    Array<double> yFromStorage(0.0,ny);

    const Array<string>& labels =  aStatesStore.getColumnLabels();
    int numOpenSimStates = labels.getSize()-1;

    SimTK::Vector stateData;
    stateData.resize(numOpenSimStates);

    for(int i=iInitial;i<=iFinal;i++) {
        tPrev = t;
        aStatesStore.getTime(i,s.updTime()); // time
        t = s.getTime();
        aModel.setAllControllersEnabled(true);

        aStatesStore.getData(i,numOpenSimStates,&stateData[0]); // states
        // Get data into local Vector and assign to State using common utility
        // to handle internal (non-OpenSim) states that may exist
        Array<std::string> stateNames = aStatesStore.getColumnLabels();
        for (int j=0; j<stateData.size(); ++j){
            // storage labels included time at index 0 so +1 to skip
            aModel.setStateVariable(s, stateNames[j+1], stateData[j]);
        }
       
        // Adjust configuration to match constraints and other goals
        aModel.assemble(s);

        // equilibrateMuscles before realization as it may affect forces
        if(aSolveForEquilibrium){
            try{// might not be able to equilibrate if model is in
                // a non-physical pose. For example, a pose where the 
                // muscle length is shorter than the tendon slack-length.
                // the muscle will throw an Exception in this case.
                aModel.equilibrateMuscles(s);
            }
            catch (const std::exception& e) {
                cout << "WARNING- AnalyzeTool::run() unable to equilibrate muscles ";
                cout << "at time = " << t <<"." << endl;
                cout << "Reason: " << e.what() << endl;
            }
        }
        // Make sure model is atleast ready to provide kinematics
        aModel.getMultibodySystem().realize(s, SimTK::Stage::Velocity);

        if(i==iInitial) {
            analysisSet.begin(s);
        } else if(i==iFinal) {
            analysisSet.end(s);
        // Step
        } else {
            analysisSet.step(s,i);
        }
    }
}
