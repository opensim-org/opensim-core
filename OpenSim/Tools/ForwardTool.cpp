/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ForwardTool.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include "ForwardTool.h"
#include <OpenSim/Common/IO.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "CorrectionController.h"

using namespace std;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ForwardTool::~ForwardTool()
{

    if(_yStore!=NULL) delete _yStore;

}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ForwardTool::ForwardTool() :
    AbstractTool(),
    _statesFileName(_statesFileNameProp.getValueStr()){
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
ForwardTool::ForwardTool(const string &aFileName,bool aUpdateFromXMLNode,bool aLoadModel) :
    AbstractTool(aFileName, false),
    _statesFileName(_statesFileNameProp.getValueStr()){
    setNull();

    if(aUpdateFromXMLNode) updateFromXMLDocument();
    if(aLoadModel) { 
        loadModel(aFileName); 
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
 * Copy constructors for all Tools only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Tool:
 *
 * 1) Construction based on XML file (@see Tool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Tool(const XMLDocument *aDocument).
 * This constructor explicitly requests construction based on an
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
ForwardTool::
ForwardTool(const ForwardTool &aTool) :
    AbstractTool(aTool),
    _statesFileName(_statesFileNameProp.getValueStr()){
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void ForwardTool::setNull()
{
    setupProperties();

    // BASIC
    _statesFileName = "";
    _printResultFiles = true;

    _replaceForceSet = false;   // default should be false for Forward.

    // INTERNAL WORK VARIABLES
    _yStore = NULL;

    // Start parsing log as empty. 
    _parsingLog="";
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ForwardTool::setupProperties()
{
    string comment;

    // BASIC

    comment = "Storage file (.sto) containing the initial states for the forward simulation. "
                 "This file often contains multiple rows of data, each row being a time-stamped array of states. "
                 "The first column contains the time.  The rest of the columns contain the states in the order "
                 "appropriate for the model. In a storage file, unlike a motion file (.mot), non-uniform time spacing "
                 "is allowed.  If the user-specified initial time for a simulation does not correspond exactly to "
                 "one of the time stamps in this file, interpolation is NOT used because it is usually necessary to "
                 "being a simulation from an exact set of states.  Instead, the closest earlier set of states is used. "
                 "Having a states file that contains the entire trajectory of a simulations allows for corrective "
                 "springs for perturbation analysis to be added.";
    _statesFileNameProp.setComment(comment);
    _statesFileNameProp.setName("states_file");
    _propertySet.append( &_statesFileNameProp );
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
ForwardTool& ForwardTool::
operator=(const ForwardTool &aTool)
{
   
    // BASE CLASS
    AbstractTool::operator=(aTool);

    // MEMBER VARIABLES
    // BASIC INPUT
    _statesFileName = aTool._statesFileName;

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
bool ForwardTool::run()
{
    log_warn("Running tool {}...", getName());
    // CHECK FOR A MODEL
    if(_model==NULL) {
        string msg = "A model has not been set.";
        log_error(msg);
        throw(Exception(msg,__FILE__,__LINE__));
    }

    // SET OUTPUT PRECISION
    IO::SetPrecision(_outputPrecision);

    // Do the maneuver to change then restore working directory 
    // so that the parsing code behaves properly if called from a different directory.
    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    /*bool externalLoads = */createExternalLoads(_externalLoadsFileName, *_model);

    // Re create the system with forces above and Realize the topology
    SimTK::State& s = _model->initSystem();
    _model->getMultibodySystem().realize(s, SimTK::Stage::Position);

    loadStatesStorage(_statesFileName, _yStore);

    // set the desired states for controllers  
    _model->updControllerSet().setDesiredStates( _yStore );

    // INITIAL AND FINAL TIMES AND STATES INDEX
    int startIndexForYStore = determineInitialTimeFromStatesStorage(_ti);

    // SETUP SIMULATION
    // Manager (now allocated on the heap so that getManager doesn't return stale pointer on stack
    Manager manager(*_model);
    setManager( manager );
    manager.setSessionName(getName());
    if (!_printResultFiles){
        manager.setWriteToStorage(false);
    }
    manager.setIntegratorInternalStepLimit(_maxSteps);
    manager.setIntegratorMaximumStepSize(_maxDT);
    manager.setIntegratorMinimumStepSize(_minDT);
    manager.setIntegratorAccuracy(_errorTolerance);
    // integ->setFineTolerance(_fineTolerance); No equivalent in SimTK

    // get values for state variables in rawData then assign by name to model
    int numStateVariables = _model->getNumStateVariables();
    Array<double> rawData = Array<double>(0.0, numStateVariables);
    // SET THE INITIAL STATES
    if(_yStore!=NULL) _yStore->getData(startIndexForYStore,numStateVariables,&rawData[0]);
    if(startIndexForYStore >= 0) {
        _yStore->getData(startIndexForYStore,numStateVariables,&rawData[0]);
    }
    if (_yStore!=NULL || startIndexForYStore >= 0){
        Array<std::string> stateNames = _model->getStateVariableNames();
        for (int i=0; i<numStateVariables; i++)
            _model->setStateVariableValue(s, stateNames[i], rawData[i]);
    }
    // SOLVE FOR EQUILIBRIUM FOR AUXILIARY STATES (E.G., MUSCLE FIBER LENGTHS)
    if(_solveForEquilibriumForAuxiliaryStates) {
        _model->equilibrateMuscles(s);  
    }


    bool completed = true;

    try {
        // INTEGRATE
        if (Logger::shouldLog(Logger::Level::Info)) {
            _model->printDetailedInfo(s, std::cout);
        }
        s.setTime(_ti);
        manager.initialize(s);
        manager.integrate(_tf);
    } catch(const std::exception& x) {
        log_error("ForwardTool::run() caught an exception: \n {}", x.what());
        completed = false;
        cwd.restore();
    }
    catch (...) { // e.g. may get InterruptedException
        log_error("ForwardTool::run() caught an exception.");
        completed = false;
        cwd.restore();
    }
    // PRINT RESULTS
    string fileName;
    if(_printResultFiles) printResultsInternal();

    cwd.restore();

    removeAnalysisSetFromModel();
    return completed;
}

void ForwardTool::printResults() {
    log_warn("ForwardTool::printResults() does nothing; use "
             "ForwardTool::setPrintResultFiles(true) before run().");
}
//=============================================================================
// PRINT RESULTS. 
// This method needs to be private since it shouldn't be called except from 
// within tool::run() otherwise will crash because Manager is out of scope
//=============================================================================
void ForwardTool::printResultsInternal()
{
    // Do the maneuver to change then restore working directory 
    // so that the parsing code behaves properly if called from a different directory.
    auto cwd = IO::CwdChanger::changeToParentOf(getDocumentFileName());

    AbstractTool::printResults(getName(),getResultsDir()); // this will create results directory if necessary
    if (_model) {
        _model->printControlStorage(getResultsDir() + "/" + getName() + "_controls.sto");
        getManager().getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");

        Storage statesDegrees(getManager().getStateStorage());
        _model->getSimbodyEngine().convertRadiansToDegrees(statesDegrees);
        statesDegrees.setWriteSIMMHeader(true);
        statesDegrees.print(getResultsDir() + "/" + getName() + "_states_degrees.mot");
    }
}



//=============================================================================
// UTILITY
//=============================================================================
int ForwardTool::determineInitialTimeFromStatesStorage(double &rTI)
{
    int index = -1;
    double ti;
    if(_yStore!=NULL) {
        index = _yStore->findIndex(rTI);
        if(index<0) {
            rTI = _yStore->getFirstTime();
            log_warn("The initial time set for the investigation precedes the "
                     "first time in the initial states file. Setting the "
                     "investigation to run at the first time in the initial "
                     "states file (ti = {}).", rTI);
            index = 0;
        } else {
            _yStore->getTime(index,ti);
            if(rTI!=ti) {
                rTI = ti;
                log_info("{}: The initial time for the investigation has been "
                         "set to {} to agree exactly with the time stamp of "
                         "the closest initial states in file '{}'.", 
                    getName(), rTI, _statesFileName);
            }
        }
    }
    return(index);
}

void ForwardTool::loadStatesStorage(std::string& statesFileName, Storage*& rYStore) const {
    // Initial states
    rYStore = NULL;
    if(_statesFileName!="") {
        log_info("Loading states from file {}.", _statesFileName);
        Storage temp(statesFileName);
        rYStore = new Storage();
        _model->formStateStorage(temp, *rYStore);

        log_info("Found {} state vectors with time stamps ranging from {} to "
                 "{}.", 
            rYStore->getSize(), rYStore->getFirstTime(), 
            rYStore->getLastTime());
    }
}

//=============================================================================
// CUBIC STEP FUNCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * A smooth step-down function using cubic polynomial.
 * x=0 for t<t0, x=1 for t>t1, and x=smooth step in between t0 and t1.
 *
 * @param t    Parameter at which to evaluate step function
 * @param t0   Parameter value at which step starts (result=0 to the left)
 * @param t1   Parameter value at which step ends (result=1 to the right)
 */
double ForwardTool::
Step(double t, double t0, double t1)
{
    double tn=(t-t0)/(t1-t0);
    if(tn<0) return 0;
    else if(tn>1) return 1;
    else return pow(tn,2)*(3-2*tn);
}
//=============================================================================
// EXPONENTIALS
//=============================================================================
//_____________________________________________________________________________
/**
 * A smooth step-up function using exponentials.
 *
 * @param tau  Rise and fall time constant.
 * @param to   Location of the midpoint of the step.
 * @param t    Independent variable
 */
double ForwardTool::
SigmaUp(double tau,double to,double t)
{
    return(  1.0 / (1.0 + exp(-(t-to)/tau)) );
}
//_____________________________________________________________________________
/**
 * A smooth step-down function using exponentials.
 *
 * @param tau  Rise and fall time constant.
 * @param to   Location of the midpoint of the step.
 * @param t    Independent variable
 */
double ForwardTool::
SigmaDn(double tau,double to,double t)
{
    return(1.0 -  1.0 / (1.0 + exp(-(t-to)/tau)) );
}
//_____________________________________________________________________________
/**
 *  Set the current integration manager
 *
 * @param m   pointer to integration manager  
 */

void ForwardTool::setManager( Manager& m ) {
   _manager = &m;
}
//_____________________________________________________________________________
/**
 *  Get the current integration manager
 *
 */
const Manager& ForwardTool::getManager() const {
   return( *_manager);
}
//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the tool to match current version
 */
/*virtual*/ void ForwardTool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    int documentVersion = versionNumber;
    bool neededSprings=false;
    std::string savedCwd;
    if(getDocument()) {
        savedCwd = IO::getCwd();
        IO::chDir(IO::getParentDirectory(getDocument()->getFileName()));
    }   
    if ( documentVersion < XMLDocument::getLatestVersion()){
            // Now check if we need to create a correction controller to replace springs
        if (documentVersion<10904){
            std::string propNames[]={
                "body1_linear_corrective_spring_active",
                "body1_torsional_corrective_spring_active",
                "body2_linear_corrective_spring_active",
                "body2_torsional_corrective_spring_active"
            };
            int i=0;
            while (!neededSprings && i<4){
                neededSprings = (aNode.element_begin(propNames[i++])!=aNode.element_end());
            }
            AbstractTool::updateFromXMLNode(aNode, versionNumber);
            if (neededSprings){
                CorrectionController* cc = new CorrectionController();
                cc->setKp(16.0);
                cc->setKv(8.0);
                _controllerSet.adoptAndAppend(cc);
                _parsingLog+= "This setup file contains corrective springs.\n";
                _parsingLog+= "Corrective springs are deprecated in OpenSim 2.0\n";
                _parsingLog+= "Instead, a Corrective Controller has been created.\n";

            }
        }
        else
            AbstractTool::updateFromXMLNode(aNode, versionNumber);
    }
    else
        AbstractTool::updateFromXMLNode(aNode, versionNumber);
    if(getDocument()) IO::chDir(savedCwd);
    //Object::updateFromXMLNode(aNode, versionNumber);
}
