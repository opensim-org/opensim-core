// SimulationTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "SimulationTool.h"
#include "LoadModel.h"
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimulationTool::~SimulationTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimulationTool::SimulationTool():
	_modelLibrary(_modelLibraryProp.getValueStr()),
	_modelFile(_modelFileProp.getValueStr()),
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
	_contactForceSetFile(_contactForceSetFileProp.getValueStr()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_fineTolerance(_fineToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj())
{
	setType("SimulationTool");
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
SimulationTool::SimulationTool(const string &aFileName):
	Object(aFileName),
	_modelLibrary(_modelLibraryProp.getValueStr()),
	_modelFile(_modelFileProp.getValueStr()),
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
	_contactForceSetFile(_contactForceSetFileProp.getValueStr()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_fineTolerance(_fineToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj())
{
	setType("SimulationTool");
	setNull();
	updateFromXMLNode();
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	loadModel();

	IO::chDir(saveWorkingDirectory);
}
//_____________________________________________________________________________
/**
 * Construct from a DOMElement.
 */
SimulationTool::SimulationTool(DOMElement *aElement):
	Object(aElement),
	_modelLibrary(_modelLibraryProp.getValueStr()),
	_modelFile(_modelFileProp.getValueStr()),
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
	_contactForceSetFile(_contactForceSetFileProp.getValueStr()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_fineTolerance(_fineToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj())
{
	setType("SimulationTool");
	setNull();
	updateFromXMLNode();
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
 * There are three proper ways to generate an XML document for a SimulationTool:
 *
 * 1) Construction based on XML file (@see SimulationTool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by SimulationTool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * SimulationTool member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated SimulationTool member variable, are preserved.
 *
 * 3) A call to generateDocument().
 * This method generates an XML document for the SimulationTool from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aTool Object to be copied.
 * @see SimulationTool(const XMLDocument *aDocument)
 * @see SimulationTool(const char *aFileName)
 * @see generateDocument()
 */
SimulationTool::SimulationTool(const SimulationTool &aTool):
	Object(aTool),
	_modelLibrary(_modelLibraryProp.getValueStr()),
	_modelFile(_modelFileProp.getValueStr()),
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
	_contactForceSetFile(_contactForceSetFileProp.getValueStr()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_fineTolerance(_fineToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj())
{
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void SimulationTool::
setNull()
{
	setupProperties();

	_model = NULL;
	_modelLibrary = "";
	_modelFile = "";
	_replaceActuatorSet = true;
	_contactForceSetFile = "";
	_resultsDir = "./";
	_outputPrecision = 8;
	_ti = 0.0;
	_tf = 1.0;
	_maxSteps = 20000;
	_maxDT = 1.0;
	_errorTolerance = 1.0e-3;
	_fineTolerance = 1.0e-5;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimulationTool::setupProperties()
{
	string comment;

	comment = "Name of the model library to load. Do not include the library extension (e.g., .dll or .lib).";
	_modelLibraryProp.setComment(comment);
	_modelLibraryProp.setName("model_library");
	_propertySet.append( &_modelLibraryProp );

	comment = "Name of the .osim file used to construct a model.";
	_modelFileProp.setComment(comment);
	_modelFileProp.setName("model_file");
	_propertySet.append( &_modelFileProp );

	comment = "Replace the model's actuator set with sets specified in <actuator_set_files>? "
				 "If false, the actuator set is appended to.";
	_replaceActuatorSetProp.setComment(comment);
	_replaceActuatorSetProp.setName("replace_actuator_set");
	_propertySet.append( &_replaceActuatorSetProp );

	comment = "List of xml files used to construct an actuator set for the model.";
	_actuatorSetFilesProp.setComment(comment);
	_actuatorSetFilesProp.setValue(Array<string>(""));
	_actuatorSetFilesProp.setName("actuator_set_files");
	_propertySet.append( &_actuatorSetFilesProp );

	comment = "Name of the xml file used to construct a contact set for the model.";
	_contactForceSetFileProp.setComment(comment);
	_contactForceSetFileProp.setName("contact_force_set_file");
	_propertySet.append( &_contactForceSetFileProp );

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

	comment = "Maximum number of integrator steps.";
	_maxStepsProp.setComment(comment);
	_maxStepsProp.setName("maximum_number_of_integrator_steps");
	_propertySet.append( &_maxStepsProp );

	comment = "Maximum integration step size.";
	_maxDTProp.setComment(comment);
	_maxDTProp.setName("maximum_integrator_step_size");
	_propertySet.append( &_maxDTProp );

	comment = "Integrator error tolerance. When the error is greater, the integrator step size is decreased.";
	_errorToleranceProp.setComment(comment);
	_errorToleranceProp.setName("integrator_error_tolerance");
	_propertySet.append( &_errorToleranceProp );

	comment = "Integrator fine tolerance. When the error is less, the integrator step size is increased.";
	_fineToleranceProp.setComment(comment);
	_fineToleranceProp.setName("integrator_fine_tolerance");
	_propertySet.append( &_fineToleranceProp );

	comment = "Set of analyses to be run during the investigation.";
	_analysisSetProp.setComment(comment);
	_analysisSetProp.setName("Analyses");
	_propertySet.append( &_analysisSetProp );
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
SimulationTool& SimulationTool::
operator=(const SimulationTool &aTool)
{
	// BASE CLASS
	Object::operator=(aTool);

	// MEMEBER VARIABLES
	_model = aTool._model;

	_modelLibrary = aTool._modelLibrary;
	_modelFile = aTool._modelFile;
	_actuatorSetFiles = aTool._actuatorSetFiles;
	_contactForceSetFile = aTool._contactForceSetFile;
	_resultsDir = aTool._resultsDir;

	_outputPrecision = aTool._outputPrecision;
	_ti = aTool._ti;
	_tf = aTool._tf;
	_maxSteps = aTool._maxSteps;
	_errorTolerance = aTool._errorTolerance;
	_fineTolerance = aTool._fineTolerance;
	_analysisSet = aTool._analysisSet;

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
 */
void SimulationTool::
setModel(AbstractModel *aModel)
{
	_model = aModel;
	_analysisSet.setModel(_model);
}
//_____________________________________________________________________________
/**
 * Get the model to be investigated.
 */
AbstractModel* SimulationTool::
getModel() const
{
	return(_model);
}

//-----------------------------------------------------------------------------
// OUTPUT PRECISION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the output precision.
 */
void SimulationTool::
setOutputPrecision(int aOutputPrecision)
{
	_outputPrecision = aOutputPrecision;
}
//_____________________________________________________________________________
/**
 * Get the output precision.
 */
int SimulationTool::
getOutputPrecision() const
{
	return(_outputPrecision);
}

//-----------------------------------------------------------------------------
// ANALYSIS SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the analysis set.
 */
AnalysisSet& SimulationTool::
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
void SimulationTool::
loadModel()
{
	// If _modelLibrary is not specified, we do not try to load the model here and assume
	// the caller/user of this investigation will take care of setting it up.
	if (_modelLibrary != "") {
		cout<<"SimulationTool "<<getName()<<" loading a model:" << endl;
		cout<<"ModelLibrary = " << _modelLibrary << ", ModelFile = " << _modelFile << endl;

		AbstractModel *model = LoadModel(_modelLibrary, _modelFile);

		// If replacing actuator set read in from model file, clear it here
		if(_replaceActuatorSet) model->getActuatorSet()->setSize(0);

		// Load actuator set(s)
		for(int i=0;i<_actuatorSetFiles.getSize();i++) {
			cout<<"Adding actuator set from "<<_actuatorSetFiles[i]<<endl;
			ActuatorSet *actuatorSet=new ActuatorSet(_actuatorSetFiles[i]);
			model->getActuatorSet()->append(*actuatorSet);
		}

		model->setup();
		if(!model->getActuatorSet()->check())
			throw(Exception("ERROR ActuatorSet::check() failed",__FILE__,__LINE__));
		setModel(model);
	}
}
//_____________________________________________________________________________
/**
 * Adds Analysis objects from analysis set to model.
 */
void SimulationTool::
addAnalysisSetToModel()
{
	if (!_model) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	int size = _analysisSet.getSize();
	for(int i=0;i<size;i++) {
		Analysis *analysis = _analysisSet.get(i);
		if(analysis==NULL) continue;
		analysis->setModel(_model);
		_model->addAnalysis(analysis);
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
void SimulationTool::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	cout<<"Printing results of investigation "<<getName()<<" to "<<aDir<<".\n";
	IO::makeDir(aDir);
	_model->getAnalysisSet()->printResults(aBaseName,aDir,aDT,aExtension);
}
