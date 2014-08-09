/* -------------------------------------------------------------------------- *
 *                         OpenSim:  AbstractTool.cpp                         *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include "AbstractTool.h"
#include <OpenSim/Common/IO.h>

#include "PrescribedForce.h"
#include "ForceSet.h"
#include "BodySet.h"
#include "Model.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractTool::~AbstractTool()
{
    //if (_toolOwnsModel)
    //  delete _model;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractTool::AbstractTool():
    _modelFile(_modelFileProp.getValueStr()),
    _replaceForceSet(_replaceForceSetProp.getValueBool()),
    _forceSetFiles(_forceSetFilesProp.getValueStrArray()),
    _resultsDir(_resultsDirProp.getValueStr()),
    _outputPrecision(_outputPrecisionProp.getValueInt()),
    _ti(_tiProp.getValueDbl()),
    _tf(_tfProp.getValueDbl()),
    _solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
    _maxSteps(_maxStepsProp.getValueInt()),
    _maxDT(_maxDTProp.getValueDbl()),
    _minDT(_minDTProp.getValueDbl()),
    _errorTolerance(_errorToleranceProp.getValueDbl()),
    _analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
    _analysisSet((AnalysisSet&)_analysisSetProp.getValueObj()),
    _controllerSetProp(PropertyObj("Controllers", ControllerSet())),
    _controllerSet((ControllerSet&)_controllerSetProp.getValueObj()),
    _externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
    _toolOwnsModel(true)
{
    setNull();
}

/**
 * Construct from file, and an optional GuiModel
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
AbstractTool::AbstractTool(const string &aFileName, bool aUpdateFromXMLNode):
    Object(aFileName, false),
    _modelFile(_modelFileProp.getValueStr()),
    _replaceForceSet(_replaceForceSetProp.getValueBool()),
    _forceSetFiles(_forceSetFilesProp.getValueStrArray()),
    _resultsDir(_resultsDirProp.getValueStr()),
    _outputPrecision(_outputPrecisionProp.getValueInt()),
    _ti(_tiProp.getValueDbl()),
    _tf(_tfProp.getValueDbl()),
    _solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
    _maxSteps(_maxStepsProp.getValueInt()),
    _maxDT(_maxDTProp.getValueDbl()),
    _minDT(_minDTProp.getValueDbl()),
    _errorTolerance(_errorToleranceProp.getValueDbl()),
    _analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
    _analysisSet((AnalysisSet&)_analysisSetProp.getValueObj()),
    _controllerSetProp(PropertyObj("Controllers", ControllerSet())),
    _controllerSet((ControllerSet&)_controllerSetProp.getValueObj()),
    _externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
    _toolOwnsModel(true)
{
    _analysisSet.setMemoryOwner(false);
    setNull();
    if(aUpdateFromXMLNode) updateFromXMLDocument();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all SimulationTools only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for a AbstractTool:
 *
 * 1) Construction based on XML file (@see AbstractTool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by AbstractTool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * AbstractTool member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated AbstractTool member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the AbstractTool from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aTool Object to be copied.
 * @see AbstractTool(const XMLDocument *aDocument)
 * @see AbstractTool(const char *aFileName)
 * @see generateXMLDocument()
 */
AbstractTool::AbstractTool(const AbstractTool &aTool):
    Object(aTool),
    _modelFile(_modelFileProp.getValueStr()),
    _replaceForceSet(_replaceForceSetProp.getValueBool()),
    _forceSetFiles(_forceSetFilesProp.getValueStrArray()),
    _resultsDir(_resultsDirProp.getValueStr()),
    _outputPrecision(_outputPrecisionProp.getValueInt()),
    _ti(_tiProp.getValueDbl()),
    _tf(_tfProp.getValueDbl()),
    _solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
    _maxSteps(_maxStepsProp.getValueInt()),
    _maxDT(_maxDTProp.getValueDbl()),
    _minDT(_minDTProp.getValueDbl()),
    _errorTolerance(_errorToleranceProp.getValueDbl()),
    _analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
    _analysisSet((AnalysisSet&)_analysisSetProp.getValueObj()),
    _controllerSetProp(PropertyObj("Controllers", ControllerSet())),
    _controllerSet((ControllerSet&)_controllerSetProp.getValueObj()),
    _externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
    _toolOwnsModel(true)
{
    _analysisSet.setMemoryOwner(false);
    setNull();
    *this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void AbstractTool::
setNull()
{
    setupProperties();

    _model = NULL;
    _modelFile = "";
    _replaceForceSet = true;
    _resultsDir = "./";
    _outputPrecision = 8;
    _ti = 0.0;
    _tf = 1.0;
    _solveForEquilibriumForAuxiliaryStates = false;
    _maxSteps = 20000;
    _maxDT = 1.0;
    _minDT = 1.0e-8;
    _errorTolerance = 1.0e-5;
    _toolOwnsModel=true;
    _externalLoadsFileName = "";
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractTool::setupProperties()
{
    string comment;
    comment = "Name of the .osim file used to construct a model.";
    _modelFileProp.setComment(comment);
    _modelFileProp.setName("model_file");
    _propertySet.append( &_modelFileProp );


    comment = "Replace the model's force set with sets specified in <force_set_files>? "
                 "If false, the force set is appended to.";
    _replaceForceSetProp.setComment(comment);
    _replaceForceSetProp.setName("replace_force_set");
    _propertySet.append( &_replaceForceSetProp );

    comment = "List of xml files used to construct an force set for the model.";
    _forceSetFilesProp.setComment(comment);
    _forceSetFilesProp.setValue(Array<string>(""));
    _forceSetFilesProp.setName("force_set_files");
    _propertySet.append( &_forceSetFilesProp );

    comment = "Directory used for writing results.";
    _resultsDirProp.setComment(comment);
    _resultsDirProp.setName("results_directory");
    _propertySet.append( &_resultsDirProp );

    comment = "Output precision.  It is 8 by default.";
    _outputPrecisionProp.setComment(comment);
    _outputPrecisionProp.setName("output_precision");
    _propertySet.append( &_outputPrecisionProp );

    comment = "Initial time for the simulation.";
    _tiProp.setComment(comment);
    _tiProp.setName("initial_time");
    _propertySet.append( &_tiProp );

    comment = "Final time for the simulation.";
    _tfProp.setComment(comment);
    _tfProp.setName("final_time");
    _propertySet.append( &_tfProp );

    comment = "Flag indicating whether or not to compute equilibrium values for states other than the coordinates or speeds.  "
                "For example, equilibrium muscle fiber lengths or muscle forces.";
    _solveForEquilibriumForAuxiliaryStatesProp.setComment(comment);
    _solveForEquilibriumForAuxiliaryStatesProp.setName("solve_for_equilibrium_for_auxiliary_states");
    _propertySet.append( &_solveForEquilibriumForAuxiliaryStatesProp );

    comment = "Maximum number of integrator steps.";
    _maxStepsProp.setComment(comment);
    _maxStepsProp.setName("maximum_number_of_integrator_steps");
    _propertySet.append( &_maxStepsProp );

    comment = "Maximum integration step size.";
    _maxDTProp.setComment(comment);
    _maxDTProp.setName("maximum_integrator_step_size");
    _propertySet.append( &_maxDTProp );

    comment = "Minimum integration step size.";
    _minDTProp.setComment(comment);
    _minDTProp.setName("minimum_integrator_step_size");
    _propertySet.append( &_minDTProp );

    comment = "Integrator error tolerance. When the error is greater, the integrator step size is decreased.";
    _errorToleranceProp.setComment(comment);
    _errorToleranceProp.setName("integrator_error_tolerance");
    _propertySet.append( &_errorToleranceProp );

    comment = "Set of analyses to be run during the investigation.";
    _analysisSetProp.setComment(comment);
    _analysisSetProp.setName("Analyses");
    _propertySet.append( &_analysisSetProp );

    _controllerSetProp.setComment("Controller objects in the model.");
    _controllerSetProp.setName("ControllerSet");
    _propertySet.append(&_controllerSetProp);

    comment = "XML file (.xml) containing the forces applied to the model as ExternalLoads.";
    _externalLoadsFileNameProp.setComment(comment);
    _externalLoadsFileNameProp.setName("external_loads_file");
    _propertySet.append( &_externalLoadsFileNameProp );
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
AbstractTool& AbstractTool::
operator=(const AbstractTool &aTool)
{
    // BASE CLASS
    Object::operator=(aTool);

    // MEMEBER VARIABLES
    _model = aTool._model;

    _modelFile = aTool._modelFile;
    _replaceForceSet = aTool._replaceForceSet;
    _forceSetFiles = aTool._forceSetFiles;
    _resultsDir = aTool._resultsDir;

    _outputPrecision = aTool._outputPrecision;
    _ti = aTool._ti;
    _tf = aTool._tf;
    _solveForEquilibriumForAuxiliaryStates = aTool._solveForEquilibriumForAuxiliaryStates;
    _maxSteps = aTool._maxSteps;
    _maxDT = aTool._maxDT;
    _minDT = aTool._minDT;
    _errorTolerance = aTool._errorTolerance;
    _analysisSet = aTool._analysisSet;
    _toolOwnsModel = aTool._toolOwnsModel;

    _externalLoadsFileName = aTool._externalLoadsFileName;
    // CONTROLLER
    _controllerSet = aTool._controllerSet;
    
    return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model to be investigated.
 * NOTE: setup() should have been called on the model prior to calling this method
 */
void AbstractTool::
setModel(Model& aModel)
{
    _model = &aModel;
    //_toolOwnsModel=false;
    if(_model) {
       addAnalysisSetToModel();
       addControllerSetToModel();
    }
}
//_____________________________________________________________________________
/**
 * Get the model to be investigated.
 */
Model& AbstractTool::
getModel() const
{
    if (_model==NULL)
        throw Exception("AbstractTool::getModel(): Model has not been set");
    return(*_model);
}

//-----------------------------------------------------------------------------
// ANALYSIS SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the analysis set.
 */
AnalysisSet& AbstractTool::
getAnalysisSet() const
{
    return(_analysisSet);
}

//=============================================================================
// LOAD MODEL
//=============================================================================
//_____________________________________________________________________________
/**
 * Load and construct a model based on the property settings of
 * this investigation.
 */
void AbstractTool::
loadModel(const string &aToolSetupFileName, ForceSet *rOriginalForceSet )
{
    if (_modelFile != "") {
        string saveWorkingDirectory = IO::getCwd();
        string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
        IO::chDir(directoryOfSetupFile);

        cout<<"AbstractTool "<<getName()<<" loading model '"<<_modelFile<<"'"<<endl;

        Model *model = 0;

        try {
            model = new Model(_modelFile);
            if (rOriginalForceSet!=NULL)
                *rOriginalForceSet = model->getForceSet();
        } catch(...) { // Properly restore current directory if an exception is thrown
            IO::chDir(saveWorkingDirectory);
            throw;
        }
        _model = model;
        IO::chDir(saveWorkingDirectory);

    }
}

void AbstractTool::
updateModelForces(Model& model, const string &aToolSetupFileName, ForceSet *rOriginalForceSet )
{
    string saveWorkingDirectory = IO::getCwd();
    string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
    IO::chDir(directoryOfSetupFile);

    try {
        if(rOriginalForceSet) *rOriginalForceSet = model.getForceSet();

        // If replacing force set read in from model file, clear it here
        if (_replaceForceSet){
            // Can no longer just remove the model's forces.
            // If the model is connected, then the model will
            // maintain a list of subcomponents that refer to garbage.
            model.cleanup();
            model.updForceSet().setSize(0);
        }

        // Load force set(s)
        for(int i=0;i<_forceSetFiles.getSize();i++) {
            cout<<"Adding force object set from "<<_forceSetFiles[i]<<endl;
            ForceSet *forceSet=new ForceSet(model, _forceSetFiles[i]);
            model.updForceSet().append(*forceSet);
        }

    } catch (...) {
        IO::chDir(saveWorkingDirectory);
        throw;
    }

    IO::chDir(saveWorkingDirectory);
}
//_____________________________________________________________________________
/**
 * Adds Analysis objects from analysis set to model.
 *
 * NOTE: Makes copies of analyses.  Also, both this tool and the model have ownership of their analysis
 * objects, so making a copy is necessary so a single analysis won't be deleted twice.
 *
 * To avoid leaking when the tool is run from the GUI, pointers to the model's copy of the analyses
 * are kept around so that they can be removed at the end of tool execution.
 *  _analysisCopies is used to do this book keeping.
 */
void AbstractTool::
addAnalysisSetToModel()
{
    if (!_model) {
        string msg = "ERROR- A model has not been set.";
        cout<<endl<<msg<<endl;
        throw(Exception(msg,__FILE__,__LINE__));
    }

    int size = _analysisSet.getSize();
    _analysisCopies.setMemoryOwner(false);
    for(int i=0;i<size;i++) {
        if(!&_analysisSet.get(i)) continue;
        Analysis *analysis = _analysisSet.get(i).clone();
        _model->addAnalysis(analysis);
        _analysisCopies.adoptAndAppend(analysis);
    }
}
//_____________________________________________________________________________
/**
 * Adds Controller objects from Controller set to model.
 *
 * NOTE: Makes copies of Controller.  Also, both this tool and the model have ownership of their Controller
 * objects, so making a copy is necessary so a single analysis won't be deleted twice.
 *
 * To avoid leaking when the tool is run from the GUI, pointers to the model's copy of the Controller
 * are kept around so that they can be removed at the end of tool execution.
 *  _controllerCopies is used to do this book keeping.
 */
void AbstractTool::
addControllerSetToModel()
{
    if (!_model) {
        string msg = "ERROR- A model has not been set.";
        cout<<endl<<msg<<endl;
        throw(Exception(msg,__FILE__,__LINE__));
    }

    int size = _controllerSet.getSize();
    _controllerCopies.setMemoryOwner(false);
    for(int i=0;i<size;i++) {
        if(!&_controllerSet.get(i)) continue;
        Controller *controller = _controllerSet.get(i).clone();
        _model->addController(controller);
        _controllerCopies.adoptAndAppend(controller);
    }
}
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Remove Analysis objects that were added earlier from analysis set to model.
 *
 * NOTE: Pointers to the .
 */
void AbstractTool::
removeAnalysisSetFromModel()
{
    if (!_model) {
        return;
    }

    int size = _analysisCopies.getSize();
    for(int i=size-1;i>=0;i--) {
        Analysis& analysis = (Analysis&)_analysisCopies.get(i);
        _model->removeAnalysis(&analysis);
    }
}
void AbstractTool::
removeControllerSetFromModel()
{
    if (!_model) {
        return;
    }

    int size = _controllerCopies.getSize();
    for(int i=size-1;i>=0;i--) {
        Controller& controller = (Controller&)_controllerCopies.get(i);
        _model->removeController(&controller);
    }
}

//=============================================================================
// IO
//=============================================================================
//_____________________________________________________________________________
/**
 * Print the results of the analysis.
 *
 * @param aFileName File to which to print the data.
 * @param aDT Time interval between results (linear interpolation is used).
 * If not included as an argument or negative, all time steps are printed
 * without interpolation.
 * @param aExtension Extension for written files.
 */
void AbstractTool::
printResults(const string &aBaseName,const string &aDir,double aDT,
                 const string &aExtension)
{
    cout<<"Printing results of investigation "<<getName()<<" to "<<aDir<<"."<<endl;
    IO::makeDir(aDir);
    _model->updAnalysisSet().printResults(aBaseName,aDir,aDT,aExtension);
}


bool AbstractTool::createExternalLoads( const string& aExternalLoadsFileName, Model& aModel, const Storage *loadKinematics)
{
    if(aExternalLoadsFileName==""||aExternalLoadsFileName=="Unassigned") {
        cout<<"No external loads will be applied (external loads file not specified)."<<endl;
        return false;
    }

    // This is required so that the references to other files inside ExternalLoads file are interpretted 
    // as relative paths
    std::string savedCwd = IO::getCwd();
    IO::chDir(IO::getParentDirectory(aExternalLoadsFileName));
    // Create external forces
    try {
        _externalLoads = ExternalLoads(aModel, aExternalLoadsFileName);
    }
     catch (const Exception &ex) {
        // Important to catch exceptions here so we can restore current working directory...
        // And then we can rethrow the exception
        cout << "Error: failed to construct ExternalLoads from file " << aExternalLoadsFileName;
        cout << ". Please make sure the file exists and that it contains an ExternalLoads";
        cout << "object or create a fresh one." << endl;
        if(getDocument()) IO::chDir(savedCwd);
        throw(ex);
    }
    _externalLoads.setMemoryOwner(false);
    _externalLoads.invokeConnectToModel(aModel);

    string loadKinematicsFileName = _externalLoads.getExternalLoadsModelKinematicsFileName();
    
    const Storage *loadKinematicsForPointTransformation = NULL;
    
    //If the the Tool is already loading the storage allow it to pass it in for use rather than reloading and processing
    if(loadKinematics && loadKinematics->getName() == loadKinematicsFileName){
        loadKinematicsForPointTransformation = loadKinematics;
    }
    else{
        IO::TrimLeadingWhitespace(loadKinematicsFileName);
        Storage *temp = NULL;
        // fine if there are no kinematics as long as it was not assigned
        if(!(loadKinematicsFileName == "") && !(loadKinematicsFileName == "Unassigned")){
            temp = new Storage(loadKinematicsFileName);
            if(!temp){
                IO::chDir(savedCwd);
                throw Exception("DynamicsTool: could not find external loads kinematics file '"+loadKinematicsFileName+"'."); 
            }
        }
        // if loading the data, do whatever filtering operations are also specified
        if(temp && _externalLoads.getLowpassCutoffFrequencyForLoadKinematics() >= 0) {
            cout<<"\n\nLow-pass filtering coordinates data with a cutoff frequency of "<<_externalLoads.getLowpassCutoffFrequencyForLoadKinematics()<<"."<<endl;
            temp->pad(temp->getSize()/2);
            temp->lowpassIIR(_externalLoads.getLowpassCutoffFrequencyForLoadKinematics());
        }
        loadKinematicsForPointTransformation = temp;
    }
    
    // if load kinematics for performing re-expressing the point of application is provided
    // then perform the transformations
    if(loadKinematicsForPointTransformation){
        SimTK::State& s = aModel.initSystem();
        
        // Form complete storage so that the kinematics match the state labels/ordering
        Storage *qStore=NULL;
        Storage *uStore=NULL;
        aModel.getSimbodyEngine().formCompleteStorages(s, *loadKinematicsForPointTransformation,qStore,uStore);
        // qStore should be in radians
        if (qStore->isInDegrees()){
            aModel.getSimbodyEngine().convertDegreesToRadians(*qStore);
        }
        _externalLoads.transformPointsExpressedInGroundToAppliedBodies(*qStore, _ti, _tf);
        delete qStore;
        delete uStore;

        aModel.invalidateSystem();
    }
    
    // Add external loads to the set of all model forces
    for(int i=0; i<_externalLoads.getSize(); ++i){
        aModel.updForceSet().adoptAndAppend(&_externalLoads[i]);
    }

    if(!loadKinematics)
        delete loadKinematics;

    IO::chDir(savedCwd);
    return(true);
}


//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the tool to match current version
 */
/*virtual*/ void AbstractTool::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
    std::string controlsFileName ="";
    if ( versionNumber < XMLDocument::getLatestVersion()){
        // Replace names of properties
        if (versionNumber<10900){
            XMLDocument::renameChildNode(aNode, "replace_actuator_set", "replace_force_set");
        }
        if (versionNumber<10904){
            XMLDocument::renameChildNode(aNode, "actuator_set_files", "force_set_files");
            // Get name of controls_file of any and use it to build a ControlSetController later

            SimTK::Xml::element_iterator controlsFileNode = aNode.element_begin("controls_file");
            if (controlsFileNode!=aNode.element_end()){
                SimTK::String transcoded=controlsFileNode->getValueAs<SimTK::String>();
                if (transcoded.length()>0)
                    controlsFileName = transcoded;
            
                aNode.eraseNode(controlsFileNode);
                    }
                }
        if (versionNumber<20001){
            // if external loads .mot file has been speified, create 
            // an XML file corresponding to it and set it as new external loads file
            SimTK::Xml::element_iterator it = aNode.element_begin("external_loads_file");
            if (it != aNode.element_end()){
                string oldFile = it->getValueAs<string>();
            if (oldFile!="" && oldFile!="Unassigned"){
                if (oldFile.substr(oldFile.length()-4, 4)!=".xml"){
                    // get names of bodies for external loads and create an xml file for the forceSet 
                    string body1, body2;
                        body1 = aNode.element_begin("external_loads_body1")->getValueAs<string>();
                        body2 = aNode.element_begin("external_loads_body2")->getValueAs<string>();
                    string newFileName=createExternalLoadsFile(oldFile, body1, body2);
                        SimTK::Xml::element_iterator pNode = aNode.element_begin("external_loads_file");
                        if(pNode!=aNode.element_end()) {
                            pNode->setValue(newFileName);
                    }
                }
            }
        }
        }
        if (versionNumber<20201){
            // Move ExternalLoadKinematics and Filtering into ExternalLoads object
            // Get nodes for "external_loads_model_kinematics_file" and 
            // "lowpass_cutoff_frequency_for_load_kinematics" and if either is not null
            // AND "external_loads_file" is not null then move these two nodes under it
            // and change top level object type from ForceSet to ExternalLoads
            SimTK::Xml::element_iterator iter = aNode.element_begin("external_loads_file");

            if (iter!= aNode.element_end()){
                string fileName="";
                iter->getValueAs(fileName);
                if (fileName!="" && fileName != "Unassigned"){
                    string saveWorkingDirectory = IO::getCwd();
                    string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
                    IO::chDir(directoryOfSetupFile);
                    bool extLoadsFile=false;
                    try {
                        SimTK::Xml::Document doc(fileName);
                        doc.setIndentString("\t");
                        Xml::Element root = doc.getRootElement();
                        if (root.getElementTag()=="OpenSimDocument"){
                            int curVersion = root.getRequiredAttributeValueAs<int>("Version");
                            Xml::element_iterator rootIter(root.element_begin("ForceSet"));
                            if (rootIter!=root.element_end()){
                                rootIter->setElementTag("ExternalLoads");
                            }
                            Xml::element_iterator iter(root.element_begin("ExternalLoads"));
                            Xml::Element extLoadsElem = *iter;

                            SimTK::Xml::element_iterator kIter = aNode.element_begin("external_loads_model_kinematics_file");
                            if (kIter !=aNode.element_end()){
                                string kinFileName= "";
                                kIter->getValueAs(kinFileName);
                                aNode.removeNode(kIter);
                                // Make sure no node already exist
                                Xml::element_iterator iter2(extLoadsElem.element_begin("external_loads_model_kinematics_file"));
                                if (iter2 == extLoadsElem.element_end())
                                    iter->insertNodeAfter(iter->element_end(), Xml::Element("external_loads_model_kinematics_file", kinFileName));
                                else
                                    iter2->setValue(kinFileName);
                            }
                            SimTK::Xml::element_iterator fIter = aNode.element_begin("lowpass_cutoff_frequency_for_load_kinematics");
                            if (fIter !=aNode.element_end()){
                                SimTK::String freq;
                                fIter->getValueAs(freq);
                                Xml::element_iterator iter2(extLoadsElem.element_begin("lowpass_cutoff_frequency_for_load_kinematics"));
                                if (iter2 == extLoadsElem.element_end())
                                    iter->insertNodeAfter(iter->element_end(), Xml::Element("lowpass_cutoff_frequency_for_load_kinematics", freq));
                                else
                                    iter2->setValue(freq);
                            }
                            doc.writeToFile(fileName);
                        }
                    }
                    catch(...){
                        IO::chDir(saveWorkingDirectory);
                    }
                }
            }
        }
    }
    Object::updateFromXMLNode(aNode, 20303);
    // Create controllers and add them as needed.
    if (controlsFileName!="" && controlsFileName!="Unassigned"){
        // controls_file was specified, create a ControlSetController for it
        ControlSetController* csc = new ControlSetController();
        csc->setControlSetFileName(controlsFileName);
        _controllerSet.adoptAndAppend(csc);
    }
}
void AbstractTool::loadQStorage (const std::string& statesFileName, Storage& rQStore) const {
    // Initial states
    if(statesFileName!="") {
        cout<<"\nLoading q's from file "<<statesFileName<<"."<<endl;
        Storage temp(statesFileName);
        _model->formQStorage(temp, rQStore);

        cout<<"Found "<<rQStore.getSize()<<" q's with time stamps ranging"<<endl;
        cout<<"from "<<rQStore.getFirstTime()<<" to "<<rQStore.getLastTime()<<"."<<endl;
    }
}
//_____________________________________________________________________________
/**
 *  Interfaces to build controller from a file and add it to the tool
 *
 */
std::string AbstractTool::getControlsFileName() const
{
    int numControllers = _controllerSet.getSize();
    if (numControllers>=1){ // WE have potentially more than one, make sure there's controlset controller
        for(int i=0; i<numControllers; i++){
            OpenSim::Controller& controller= _controllerSet.get(i);
            if (dynamic_cast<OpenSim::ControlSetController *>(&controller)==0)
                continue;
            OpenSim::ControlSetController& dController = (OpenSim::ControlSetController&) _controllerSet.get(i);
            return (dController.getControlSetFileName());
        }
    }
    return ("Unassigned");
}
//_____________________________________________________________________________
/**
 * A Convenience method to add a ControlSetController from a file and add it
 * to the model
 */
void AbstractTool::setControlsFileName(const std::string& controlsFilename)
{
    if (controlsFilename=="" || controlsFilename=="Unassigned") return;

    int numControllers = _controllerSet.getSize();

    for(int i=0; i<numControllers;i++){
        OpenSim::Controller& controller= _controllerSet.get(i);
        if (dynamic_cast<OpenSim::ControlSetController *>(&controller)==0)
            continue;
        OpenSim::ControlSetController& dController = (OpenSim::ControlSetController&)_controllerSet[i];
        dController.setControlSetFileName(controlsFilename);
        return;
    }
    // Create a new controlsetController and add it to the tool
    ControlSetController* csc = new ControlSetController();
    csc->setControlSetFileName(controlsFilename);
    _controllerSet.adoptAndAppend(csc);
}
//_____________________________________________________________________________
/**
 * A Convenience method to verify that ColumnLabels are unique
 */
bool AbstractTool::verifyUniqueColumnLabels(const Storage& aStore) const
{
    const Array<string>& lbls = aStore.getColumnLabels();
    bool isUnique = true;
    for(int i=0; i< lbls.getSize() && isUnique; i++){
        isUnique= (lbls.findIndex(lbls[i])==i);
    }
    return isUnique;
}

//_____________________________________________________________________________
/**
 * A Convenience method to get the next unique
 */
std::string AbstractTool::getNextAvailableForceName(const std::string prefix) const
{
    int candidate=0;
    char pad[3];
    std::string candidateName;
    bool found = false;
    while (!found) {
        candidate++;
        sprintf(pad, "%d", candidate);
        candidateName = prefix +"_"+string(pad);
        if (_model) {
            if (_model->getForceSet().contains(candidateName))
                continue;
        }
        found = !(_externalLoads.contains(candidateName));
    };
    return candidateName;
}

std::string AbstractTool::createExternalLoadsFile(const std::string& oldFile, 
                                          const std::string& body1, 
                                          const std::string& body2)
{
    bool oldFileValid = !(oldFile=="" || oldFile=="Unassigned");

    std::string savedCwd;
    if(getDocument()) {
        savedCwd = IO::getCwd();
        IO::chDir(IO::getParentDirectory(getDocument()->getFileName()));
    }
    if (oldFileValid){
        if(!ifstream(oldFile.c_str(), ios_base::in).good()) {
            if(getDocument()) IO::chDir(savedCwd);
            string msg =
                "Object: ERR- Could not open file " + oldFile+ ". It may not exist or you don't have permission to read it.";
            throw Exception(msg,__FILE__,__LINE__);
        }
    }
    Storage dataFile(oldFile);
    const Array<string>& labels=dataFile.getColumnLabels();
    bool body1Valid = !(body1=="" || body1=="Unassigned");
    bool body2Valid = !(body2=="" || body2=="Unassigned");
    std::string forceLabels[9] = {"ground_force_vx", "ground_force_vy", "ground_force_vz", 
        "ground_force_px", "ground_force_py", "ground_force_pz", 
        "ground_torque_x", "ground_torque_y", "ground_torque_z"};
    // We'll create upto 2 PrescribedForces
    if (body1Valid && body2Valid){
        // Find first occurance of ground_force_vx
        int indices[9][2];
        for(int i=0; i<9; i++){
            indices[i][0]= labels.findIndex(forceLabels[i]);
            if (indices[i][0]==-1){ // Something went wrong, abort here 
                if(getDocument()) IO::chDir(savedCwd);
                string msg =
                    "Object: ERR- Could not find label "+forceLabels[i]+ "in file " + oldFile+ ". Aborting.";
                throw Exception(msg,__FILE__,__LINE__);
            }
            for(int j=indices[i][0]+1; j<labels.getSize(); j++){
                if (labels[j]==forceLabels[i]){
                    indices[i][1]=j;
                    break;
                }
            }
        }
        for(int f=0; f<2; f++){
            ExternalForce* xf = new ExternalForce();
            xf->setAppliedToBodyName((f==0)?body1:body2);
            char pad[3];
            sprintf(pad,"%d", f+1);
            std::string suffix = "ExternalForce_"+string(pad);
            xf->setName(suffix);
            _externalLoads.adoptAndAppend(xf);
        }
        _externalLoads.setDataFileName(oldFile);
        std::string newName=oldFile.substr(0, oldFile.length()-4)+".xml";
        _externalLoads.print(newName);
        if(getDocument()) IO::chDir(savedCwd);
        cout<<"\n\n- Created ForceSet file " << newName << "to apply forces from " << oldFile << ".\n\n";
        return newName;
    }
    else {
            if(getDocument()) IO::chDir(savedCwd);
            string msg =
                "Object: ERR- Only one body is specified in " + oldFile+ ".";
            throw Exception(msg,__FILE__,__LINE__);
    }
    if(getDocument()) IO::chDir(savedCwd);
}
