// AbstractTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "AbstractTool.h"
#include <OpenSim/Common/IO.h>
#include "Model.h"



using namespace OpenSim;
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
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractTool::AbstractTool():
	_modelFile(_modelFileProp.getValueStr()),
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
   _replaceContactForceSet(_replaceContactForceSetProp.getValueBool()),
	_contactForceSetFiles(_contactForceSetFilesProp.getValueStrArray()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_fineTolerance(_fineToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj())
{
	setType("AbstractTool");
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
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
   _replaceContactForceSet(_replaceContactForceSetProp.getValueBool()),
	_contactForceSetFiles(_contactForceSetFilesProp.getValueStrArray()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_fineTolerance(_fineToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj())
{
	setType("AbstractTool");
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
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
	_replaceActuatorSet(_replaceActuatorSetProp.getValueBool()),
	_actuatorSetFiles(_actuatorSetFilesProp.getValueStrArray()),
   _replaceContactForceSet(_replaceContactForceSetProp.getValueBool()),
	_contactForceSetFiles(_contactForceSetFilesProp.getValueStrArray()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
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
void AbstractTool::
setNull()
{
	setupProperties();

	_model = NULL;
	_modelFile = "";
	_replaceActuatorSet = true;
   _replaceContactForceSet = true;
	_resultsDir = "./";
	_outputPrecision = 20;
	_ti = 0.0;
	_tf = 1.0;
	_solveForEquilibriumForAuxiliaryStates = false;
	_maxSteps = 20000;
	_maxDT = 1.0;
	_errorTolerance = 1.0e-3;
	_fineTolerance = 1.0e-5;
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

   comment = "Replace the model's contact force set with sets specified in "
             "<contact_force_set_files>? If false, contact force set is appended to.";
   _replaceContactForceSetProp.setComment(comment);
   _replaceContactForceSetProp.setName("replace_contact_force_set");
   _propertySet.append( &_replaceContactForceSetProp );

	comment = "List of xml files used to construct a contact set for the model.";
	_contactForceSetFilesProp.setComment(comment);
	_contactForceSetFilesProp.setName("contact_force_set_files");
	_propertySet.append( &_contactForceSetFilesProp );

	comment = "Directory used for writing results.";
	_resultsDirProp.setComment(comment);
	_resultsDirProp.setName("results_directory");
	_propertySet.append( &_resultsDirProp );

	comment = "Output precision.  It is 20 by default.";
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
AbstractTool& AbstractTool::
operator=(const AbstractTool &aTool)
{
	// BASE CLASS
	Object::operator=(aTool);

	// MEMEBER VARIABLES
	_model = aTool._model;

	_modelFile = aTool._modelFile;
	_actuatorSetFiles = aTool._actuatorSetFiles;
	_contactForceSetFiles = aTool._contactForceSetFiles;
	_resultsDir = aTool._resultsDir;

	_outputPrecision = aTool._outputPrecision;
	_ti = aTool._ti;
	_tf = aTool._tf;
	_solveForEquilibriumForAuxiliaryStates = aTool._solveForEquilibriumForAuxiliaryStates;
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
 * NOTE: setup() should have been called on the model prior to calling this method
 */
void AbstractTool::
setModel(Model *aModel)
{
	_model = aModel;
	if(_model) addAnalysisSetToModel();
}
//_____________________________________________________________________________
/**
 * Get the model to be investigated.
 */
Model* AbstractTool::
getModel() const
{
	return(_model);
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
loadModel(const string &aToolSetupFileName, ActuatorSet *rOriginalActuatorSet, ContactForceSet *rOriginalContactForceSet)
{
	if (_modelFile != "") {
		string saveWorkingDirectory = IO::getCwd();
		string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
		IO::chDir(directoryOfSetupFile);

		cout<<"AbstractTool "<<getName()<<" loading model '"<<_modelFile<<"'"<<endl;

		Model *model = 0;

		try {
			model = new Model(_modelFile);
		} catch(...) { // Properly restore current directory if an exception is thrown
			IO::chDir(saveWorkingDirectory);
			throw;
		}

		IO::chDir(saveWorkingDirectory);

		updateModelActuatorsAndContactForces(model, aToolSetupFileName, rOriginalActuatorSet, rOriginalContactForceSet);

		model->setup();
		if(!model->getActuatorSet()->check())
			throw(Exception("ERROR ActuatorSet::check() failed",__FILE__,__LINE__));

		setModel(model);
	}
}

void AbstractTool::
updateModelActuatorsAndContactForces(Model *model, const string &aToolSetupFileName, ActuatorSet *rOriginalActuatorSet, ContactForceSet *rOriginalContactForceSet)
{
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
	IO::chDir(directoryOfSetupFile);

	try {
		if(rOriginalActuatorSet) *rOriginalActuatorSet = *model->getActuatorSet();

		// If replacing actuator set read in from model file, clear it here
		if(_replaceActuatorSet) model->getActuatorSet()->setSize(0);

		// Load actuator set(s)
		for(int i=0;i<_actuatorSetFiles.getSize();i++) {
			cout<<"Adding actuator set from "<<_actuatorSetFiles[i]<<endl;
			ActuatorSet *actuatorSet=new ActuatorSet(_actuatorSetFiles[i]);
			model->getActuatorSet()->append(*actuatorSet);
		}

		if(rOriginalContactForceSet) *rOriginalContactForceSet = *model->getContactSet();

		// If replacing contact force set read in from model file, clear it here
		if(_replaceContactForceSet) model->getContactSet()->setSize(0);

		// Load contact force set
		for(int i=0;i<_contactForceSetFiles.getSize();i++) {
			cout<<"Adding contact force set from "<<_contactForceSetFiles[i]<<endl;
			ContactForceSet *contactForceSet=new ContactForceSet(_contactForceSetFiles[i]);
			model->getContactSet()->append(*contactForceSet); // uses append defined in ActuatorSet, parent class of ContactForceSet
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
void AbstractTool::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	cout<<"Printing results of investigation "<<getName()<<" to "<<aDir<<"."<<endl;
	IO::makeDir(aDir);
	_model->getAnalysisSet()->printResults(aBaseName,aDir,aDT,aExtension);
}
