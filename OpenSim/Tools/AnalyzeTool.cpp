// AnalyzeTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "AnalyzeTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Actuators/ForceApplier.h>
#include <OpenSim/Actuators/TorqueApplier.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "ForwardTool.h"

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
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_pseudoStatesFileName(_pseudoStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("AnalyzeTool");
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
AnalyzeTool::AnalyzeTool(const string &aFileName) :
	AbstractTool(aFileName, false),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_pseudoStatesFileName(_pseudoStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("AnalyzeTool");
	setNull();
	updateFromXMLNode();
	loadModel(aFileName);
	if(_model) addAnalysisSetToModel();
	loadControlsStatesPseudoStatesExternalLoadsFromFiles();
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
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_pseudoStatesFileName(_pseudoStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("AnalyzeTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* AnalyzeTool::
copy() const
{
	AnalyzeTool *object = new AnalyzeTool(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void AnalyzeTool::
setNull()
{
	setupProperties();

	_controlsFileName = "";
	_statesFileName = "";
	_pseudoStatesFileName = "";
	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = "";
	_externalLoadsBody2 = "";
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;

	_controlSet = NULL;
	_statesStore = NULL;
	_pseudoStore = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AnalyzeTool::setupProperties()
{
	string comment;

	comment = "XML file containing the controls (e.g., muscle excitations) for the forward simulation.";
	_controlsFileNameProp.setComment(comment);
	_controlsFileNameProp.setName("controls_file");
	_propertySet.append( &_controlsFileNameProp );

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

	comment = "Storage file (.sto) containing the time history of pseudo states for the model. "
				 "This file often contains multiple rows of data, each row being a time-stamped array of pseudo states. "
				 "The first column contains the time.  The rest of the columns contain the pseudo states in the order "
				 "appropriate for the model. In a storage file, unlike a motion file (.mot), non-uniform time spacing "
				 "is allowed.  The time stamps in the pseudo states file should agree with the time stamps in the "
				 "states file.";
	_pseudoStatesFileNameProp.setComment(comment);
	_pseudoStatesFileNameProp.setName("pseudo_states_file");
	_propertySet.append( &_pseudoStatesFileNameProp );

	comment = "Motion file (.mot) or storage file (.sto) containing any external loads applied to the model.";
	_externalLoadsFileNameProp.setComment(comment);
	_externalLoadsFileNameProp.setName("external_loads_file");
	_propertySet.append( &_externalLoadsFileNameProp );

	comment =	"Motion file (.mot) or storage file (.sto) containing the model kinematics "
					"corresponding to the external loads.";
	_externalLoadsModelKinematicsFileNameProp.setComment(comment);
	_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file");
	_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

	comment = "Name of the body to which the first set of external loads "
				 "should be applied (e.g., the name of the right foot).";
	_externalLoadsBody1Prop.setComment(comment);
	_externalLoadsBody1Prop.setName("external_loads_body1");
	_propertySet.append( &_externalLoadsBody1Prop );

	comment = "Name of the body to which the second set of external loads "
				 "should be applied (e.g., the name of the left foot).";
	_externalLoadsBody2Prop.setComment(comment);
	_externalLoadsBody2Prop.setName("external_loads_body2");
	_propertySet.append( &_externalLoadsBody2Prop );

	comment = "Low-pass cut-off frequency for filtering the model kinematics corresponding "
				 "to the external loads. A negative value results in no filtering. "
				 "The default value is -1.0, so no filtering.";
	_lowpassCutoffFrequencyForLoadKinematicsProp.setComment(comment);
	_lowpassCutoffFrequencyForLoadKinematicsProp.setName("lowpass_cutoff_frequency_for_load_kinematics");
	_propertySet.append( &_lowpassCutoffFrequencyForLoadKinematicsProp );
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
	_controlsFileName = aTool._controlsFileName;
	_statesFileName = aTool._statesFileName;
	_pseudoStatesFileName = aTool._pseudoStatesFileName;
	_externalLoadsFileName = aTool._externalLoadsFileName;
	_externalLoadsModelKinematicsFileName = aTool._externalLoadsModelKinematicsFileName;
	_externalLoadsBody1Prop = aTool._externalLoadsBody1Prop;
	_externalLoadsBody2Prop = aTool._externalLoadsBody2Prop;
	_lowpassCutoffFrequencyForLoadKinematics = aTool._lowpassCutoffFrequencyForLoadKinematics;
	_controlSet = aTool._controlSet;
	_statesStore = aTool._statesStore;
	_pseudoStore = aTool._pseudoStore;

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the control set.  If a control set is not set, the values of all
 * model controls are set to 0.0.  A copy of the control set is not made.
 *
 * @param Pointer to a control set.
 */
void AnalyzeTool::
setControlSet(ControlSet *aSet)
{
	_controlSet = aSet;
}
//_____________________________________________________________________________
/**
 * Get the control set.
 *
 * @return Pointer to the control set.
 */
ControlSet* AnalyzeTool::
getControlSet()
{
	return _controlSet;
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
setStatesStorage(Storage *aStore)
{
	_statesStore = aStore;
}
//_____________________________________________________________________________
/**
 * Get the states storage.
 *
 * @return Pointer to the states storage.
 */
Storage* AnalyzeTool::
getStatesStorage()
{
	return _statesStore;
}

//_____________________________________________________________________________
/**
 * Set the pseudo states storage.  The rows of a states storage consist of
 * time-stamped vectors of all the model pseudo states.  The pseudo states
 * storage must have the same number of rows occuring at the same time stamps
 * as the states storage (@see setStatesStorage()).
 *
 * @param Pointer to storage file containing the time history of model
 * pseudo states.
 */
void AnalyzeTool::
setPseudoStatesStorage(Storage *aStore)
{
	_pseudoStore = aStore;
}
//_____________________________________________________________________________
/**
 * Get the pseudo states storage.
 *
 * @return Pointer to the pseudo states storage.
 */
Storage* AnalyzeTool::
getPseudoStatesStorage()
{
	return _pseudoStore;
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Initialize the controls, states, pseudo states, and external loads from
 * files.  The file names are stored in the property set.  The file names
 * can either come from the XML setup file, or they can be set explicitly.
 * Either way, this method should be called to read all the needed information
 * in from file.
 */
void AnalyzeTool::
loadControlsStatesPseudoStatesExternalLoadsFromFiles()
{
	// States
	if(_statesFileName=="") {
		string msg = "AnalyzeTool.initializeFromFiles: A states file must be specified.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	cout<<"\nLoading states from file "<<_statesFileName<<".\n";
	_statesStore = new Storage(_statesFileName);
	cout<<"Found "<<_statesStore->getSize()<<" state vectors with time stamps ranging\n";
	cout<<"from "<<_statesStore->getFirstTime()<<" to "<<_statesStore->getLastTime()<<".\n";

	// Controls
	if(_controlsFileName!="") {
		cout<<"\n\nLoading controls from file "<<_controlsFileName<<".\n";
		_controlSet = new ControlSet(_controlsFileName);
	}

	// Pseudo States
	int nyp = _model->getNumPseudoStates();
	if(nyp > 0) {
		if(_pseudoStatesFileName=="") {
			string msg = "AnalyzeTool.initializeFromFiles: A pseudo states file must be specified.";
			throw Exception(msg,__FILE__,__LINE__);
		}
		cout<<"\nLoading states from file "<<_pseudoStatesFileName<<".\n";
		_pseudoStore = new Storage(_pseudoStatesFileName);
	}

	// External Loads
	ForwardTool::initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics);
}
//_____________________________________________________________________________
/**
 * Verify that the controls, states, pseudo states are consistent with the
 * model.
 */
void AnalyzeTool::
verifyControlsStatesPseudoStates()
{
	int nx = _model->getNumControls();
	int ny = _model->getNumStates();
	int np = _model->getNumPseudoStates();

	// DO WE HAVE STATES AND PSEUDO STATES?
	// States
	if(_statesStore==NULL) {
		string msg = "analyzeTool.verifyControlsStatesPseudoStates: ERROR- a storage object containing "
							"the time histories of states was not specified.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	// Pseudo States
	if((np!=0)&&(_pseudoStore==NULL)) {
		string msg = "analyzeTool.verifyControlsStatesPseudoStates: ERROR- a storage object containing "
							"the time histories of pseudo states was not specified.";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// ARE THE SIZES CORRECT?
	// Controls
	if(_controlSet!=NULL) {
		int NX = _controlSet->getSize();
		if(NX!=nx) {
			string msg = "AnalyzeTool.verifyControlsStatesPseudoStates: ERROR- Size of control set " + _controlsFileName;
			msg += " doesn't match number of controls in model " + _model->getName() + ".";
			throw Exception(msg,__FILE__,__LINE__);
		}
	}
	// States
	int NY = _statesStore->getSmallestNumberOfStates();
	if(NY!=ny) {
		string msg = "AnalyzeTool.verifyControlsStatesPseudoStates: ERROR- Number of states in " + _statesFileName;
		msg += " doesn't match number of states in model " + _model->getName() + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	// PseudoStates
	if(np==0) return;
	int NP = _pseudoStore->getSmallestNumberOfStates();
	if(NP!=np) {
		string msg = "AnalyzeTool.verifyControlsStatesPseudoStates: ERROR- Number of pseudo states in " + _pseudoStatesFileName;
		msg += " doesn't match number of pseudo states in model " + _model->getName() + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// IS THE TIME STEPPING THE SAME FOR STATES & PSEUDO STATES
	if(_pseudoStore->getSize()!=_statesStore->getSize()) {
		string msg = "AnalyzeTool.verifyControlsStatesPseudoStates: ERROR- the states storage and "
			"pseudo-states storage should have the same number of rows, but do not.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	if(_pseudoStore->getFirstTime()!=_statesStore->getFirstTime()) {
		string msg = "AnalyzeTool.verifyControlsStatesPseudoStates: ERROR- the time steps for the "
			"states and pseudo-states storage should be the same, but are not.";
		throw Exception(msg,__FILE__,__LINE__);
	}
}

//_____________________________________________________________________________
/**
 * Given an index into the states storage, get the corresponding time,
 * controls, states, and pseudo states.
 *
 * @param aIndex Index into the states storage.
 * @param rX Controls.
 * @param rY States.
 * @param rP Pseudo states.
 * @return Time
 */
double AnalyzeTool::
getControlsStatesPseudoStates(int aIndex,Array<double> &rX,Array<double> &rY,Array<double> &rP)
{
	// Time
	double t;
	_statesStore->getTime(aIndex,t);

	// Controls
	if(_controlSet!=NULL) {
		_controlSet->getControlValues(t,rX);
	}

	// States
	_statesStore->getData(aIndex,rY.getSize(),&rY[0]);

	// Pseudo States
	if(_pseudoStore!=NULL) {
		_pseudoStore->getData(aIndex,rP.getSize(),&rP[0]);
	}

	return t;
}


//=============================================================================
// RUN
//=============================================================================
//_____________________________________________________________________________
/**
 * Run the investigation.
 */
void AnalyzeTool::run()
{
	cout<<"Running investigation "<<getName()<<".\n";

	// CHECK FOR A MODEL
	if(_model==NULL) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	// VERIFY THE CONTROL SET, STATES, AND PSEUDO STATES ARE TENABLE
	verifyControlsStatesPseudoStates();

	// SET OUTPUT PRECISION
	IO::SetPrecision(_outputPrecision);

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	// COMPUTE INITIAL AND FINAL INDEX
	double ti,tf;
	int iInitial = _statesStore->findIndex(_ti);
	int iFinal = _statesStore->findIndex(_tf);
	_statesStore->getTime(iInitial,ti);
	_statesStore->getTime(iFinal,tf);
	cout<<"\n\nExecuting the analyses from "<<ti<<" to "<<tf<<"...\n";

	// ANALYSIS SET
	AnalysisSet *analysisSet = _model->getAnalysisSet();
	if(analysisSet==NULL) {
		string msg = "AnalysisTool.run: ERROR- no analyses have been set.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	if(analysisSet->getSize()<=0) {
		string msg = "AnalysisTool.run: ERROR- no analyses have been set.";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// PERFORM THE ANALYSES
	double tPrev=0.0,t=0.0,dt=0.0;
	int nx = _model->getNumControls();
	int ny = _model->getNumStates();
	int np = _model->getNumPseudoStates();
	Array<double> xPrev(0.0,nx),x(0.0,nx);
	Array<double> yPrev(0.0,nx),y(0.0,ny);
	Array<double> pPrev(0.0,np),p(0.0,np);
	for(int i=iInitial;i<=iFinal;i++) {
		
		// Data
		t = getControlsStatesPseudoStates(i,x,y,p);
		if((i==iInitial) && ((i+1)<=iFinal)) {
			_statesStore->getTime(i+1,dt);
			dt = dt - t;
		} else {
			dt = t - tPrev;
		}

		// Begin
		if(i==iInitial) {
			analysisSet->begin(iInitial,dt,t,&x[0],&y[0]);

		// End
		} else if(i==iFinal) {
			analysisSet->end(iFinal,dt,t,&x[0],&y[0]);

		// Step
		} else {
			analysisSet->step(&xPrev[0],&yPrev[0],i,dt,t,&x[0],&y[0]);
		}

		// ASSIGN PREV
		tPrev = t;
		xPrev = x;
		yPrev = y;
		pPrev = p;
	}

	// PRINT RESULTS
	printResults(getName(),getResultsDir()); // this will create results directory if necessary
	IO::chDir(saveWorkingDirectory);
}
