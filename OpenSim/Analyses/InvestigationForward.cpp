// InvestigationForward.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "InvestigationForward.h"
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/GCVSplineSet.h>
#include <OpenSim/Tools/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Analyses/ForceApplier.h>
#include <OpenSim/Analyses/TorqueApplier.h>
#include <OpenSim/Simulation/Manager/Manager.h>



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InvestigationForward::~InvestigationForward()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InvestigationForward::InvestigationForward() :
	Investigation(),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationForward");
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
InvestigationForward::InvestigationForward(const string &aFileName) :
	Investigation(aFileName),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationForward");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Construct from a DOMElement.
 */
InvestigationForward::InvestigationForward(DOMElement *aElement) :
	Investigation(aElement),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationForward");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Investigation's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Investigation:
 *
 * 1) Construction based on XML file (@see Investigation(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Investigation(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Investigation member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Investigation member variable, are preserved.
 *
 * 3) A call to generateDocument().
 * This method generates an XML document for the Investigation from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aInvestigation Object to be copied.
 * @see Investigation(const XMLDocument *aDocument)
 * @see Investigation(const char *aFileName)
 * @see generateDocument()
 */
InvestigationForward::
InvestigationForward(const InvestigationForward &aInvestigation) :
	Investigation(aInvestigation),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationForward");
	setNull();
	*this = aInvestigation;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* InvestigationForward::
copy() const
{
	InvestigationForward *object = new InvestigationForward(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Virtual copy constructor from DOMElement.
 */
Object* InvestigationForward::
copy(DOMElement *aElement) const
{
	InvestigationForward *object = new InvestigationForward(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InvestigationForward::
setNull()
{
	setupProperties();

	_controlsFileName = "";
	_initialStatesFileName = "";
	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = -1;
	_externalLoadsBody2 = -1;
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void InvestigationForward::setupProperties()
{
	// INPUT FILE NAMES
	_controlsFileNameProp.setName("controls_file_name");
	_propertySet.append( &_controlsFileNameProp );

	_initialStatesFileNameProp.setName("initial_states_file_name");
	_propertySet.append( &_initialStatesFileNameProp );

	_externalLoadsFileNameProp.setName("external_loads_file_name");
	_propertySet.append( &_externalLoadsFileNameProp );

	_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file_name");
	_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

	_externalLoadsBody1Prop.setName("external_loads_body1");
	_propertySet.append( &_externalLoadsBody1Prop );

	_externalLoadsBody2Prop.setName("external_loads_body2");
	_propertySet.append( &_externalLoadsBody2Prop );

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
InvestigationForward& InvestigationForward::
operator=(const InvestigationForward &aInvestigation)
{
	// BASE CLASS
	Investigation::operator=(aInvestigation);

	// MEMEBER VARIABLES
	_controlsFileName = aInvestigation._controlsFileName;
	_initialStatesFileName = aInvestigation._initialStatesFileName;
	_externalLoadsFileName = aInvestigation._externalLoadsFileName;
	_externalLoadsModelKinematicsFileName = aInvestigation._externalLoadsModelKinematicsFileName;
	_externalLoadsBody1Prop = aInvestigation._externalLoadsBody1Prop;
	_externalLoadsBody2Prop = aInvestigation._externalLoadsBody2Prop;
	_lowpassCutoffFrequencyForLoadKinematics = aInvestigation._lowpassCutoffFrequencyForLoadKinematics;

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
void InvestigationForward::run()
{
	cout<<"Running investigation "<<getName()<<".\n";

	// CHECK FOR A MODEL
	if(_model==NULL) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	// SET OUTPUT PRECISION
	IO::SetPrecision(_outputPrecision);

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd(0, 256);
	string directoryOfSetupFile = IO::getParentDirectory(getDocument()->getFileName());
	IO::chDir(directoryOfSetupFile.c_str());

	// INPUT
	// Controls
	ControlSet *controlSet=NULL;
	if(_controlsFileName!="") {
		cout<<"\n\nLoading controls from file "<<_controlsFileName<<".\n";
		controlSet = new ControlSet(_controlsFileName);
		cout<<"Found "<<controlSet->getSize()<<" controls.\n\n";
	}
	// Initial states
	Storage *yiStore = NULL;
	if(_initialStatesFileName!="") {
		cout<<"\nLoading initial states from file "<<_initialStatesFileName<<".\n";
		yiStore = new Storage(_initialStatesFileName.c_str());
		cout<<"Found "<<yiStore->getSize()<<" state vectors with time stamps ranging\n";
		cout<<"from "<<yiStore->getFirstTime()<<" to "<<yiStore->getLastTime()<<".\n";
	}

	// INITIAL AND FINAL TIMES
	// From initial states...
	int index;
	double ti,tf;
	if(yiStore!=NULL) {
		index = yiStore->findIndex(_ti);
		if(index<0) {
			_ti = yiStore->getFirstTime();
			cout<<"\n\nWARN- The initial time set for the investigation precedes the first time\n";
			cout<<"in the initial states file.  Setting the investigation to run at the first time\n";
			cout<<"in the initial states file (ti = "<<_ti<<").\n\n";
		} else {
			yiStore->getTime(index,ti);
			if(_ti!=ti) {
				_ti = ti;
				cout<<"\n"<<getName()<<": The initial time for the investigation has been set to "<<_ti<<endl;
				cout<<"to agree exactly with the time stamp of the closest initial states in file ";
				cout<<_initialStatesFileName<<".\n\n";
			}
		}
	}

	// Check controls...
	if(controlSet!=NULL) {
		int first = 0;
		Control *control = controlSet->get(first);
		ti = control->getFirstTime();
		tf = control->getLastTime();
		if(_ti<ti) {
			cout<<"\n"<<getName()<<": WARN- The controls read in from file "<<_controlsFileName<<" did not\n";
			cout<<"overlap the requested initial time of the simulation.  Controls are being extrapolated\n";
			cout<<"rather than interpolated.\n";
		}
		if(_tf>tf) {
			cout<<"\n"<<getName()<<": WARN- The controls read in from file "<<_controlsFileName<<" did not\n";
			cout<<"overlap the requested final time of the simulation.  Changing the final time of the\n";
			cout<<"forward integration from "<<_tf<<" to "<<tf<<".\n";
			_tf = tf;
		}
	}

	// ASSIGN NUMBERS OF THINGS
	int ny = _model->getNY();
	int nq = _model->getNQ();
	int nu = _model->getNU();
	int na = _model->getNA();
	int nb = _model->getNB();

	// ADD ANALYSES
	Analysis *analysis;
	int i, size = getAnalysisSet().getSize();
	for(i=0;i<size;i++) {
		analysis = getAnalysisSet().get(i);
		if(analysis==NULL) continue;
		analysis->setModel(_model);
		_model->addAnalysis(analysis);
	}

	// GROUND REACTION FORCES
	initializeExternalLoads();

	// SETUP SIMULATION
	// Manager
	ModelIntegrand integrand(_model);
	if(controlSet!=NULL) integrand.setControlSet(*controlSet);
	Manager manager(&integrand);
	manager.setSessionName(getName());
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);

	// Integrator settings
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(_maxSteps);
	integ->setMaxDT(_maxDT);
	integ->setTolerance(_errorTolerance);
	integ->setFineTolerance(_fineTolerance);

	// SET INITIAL AND FINAL TIME AND THE INITIAL STATES
	Array<double> yi(0.0,ny);
	if(yiStore!=NULL) yiStore->getData(index,ny,&yi[0]);
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);
	_model->setInitialStates(&yi[0]);

	// INTEGRATE
	cout<<"\n\nIntegrating from "<<_ti<<" to "<<_tf<<endl;
	manager.integrate();


	// PRINT RESULTS
	printResults(getName().c_str(),getResultsDir().c_str());
	IO::chDir(saveWorkingDirectory.c_str());
}

//=============================================================================
// EXTERNAL LOADS
//=============================================================================
//_____________________________________________________________________________
/**
 * Initialize the external loads applied to the model.  Currently, the loads
 * are assumed to be applied to the right and left foot.
 */
void InvestigationForward::
initializeExternalLoads()
{
	if(_externalLoadsFileName=="") {
		cout<<"\n\nWARNING- a file name for external loads was not specified.";
		cout<<" No loads will be applied.\n\n";
		return;
	}

	// LOAD MODEL KINEMATICS FOR EXTERNAL LOADS
	// To get the forces to be applied in the correct location, this file
	// should be from the IK solution, not from pass 2 of rra which alters
	// the kinematics.
	if(_externalLoadsModelKinematicsFileName=="") {
		cout<<"\n\nERROR- a external loads kinematics file was not specified.\n\n";
		return;
	}
	cout<<"\n\nLoading external loads kinematics from file "<<_externalLoadsModelKinematicsFileName<<" ...\n";
	Storage loadsKinStore(_externalLoadsModelKinematicsFileName.c_str());
	// Form complete storage objects for the q's and u's
	// This means filling in unspecified generalized coordinates and
	// setting constrained coordinates to their valid values.
	Storage *qStore=NULL;
	Storage *uStoreTmp=NULL;
	_model->formCompleteStorages(loadsKinStore,qStore,uStoreTmp);
	_model->convertDegreesToRadians(qStore);
	// Filter
	qStore->pad(60); 
	if(_lowpassCutoffFrequencyForLoadKinematics>=0) {
		int order = 50;
		cout<<"\n\nLow-pass filtering external load kinematics with a cutoff frequency of ";
		cout<<_lowpassCutoffFrequencyForLoadKinematics<<"...\n\n";
		qStore->lowpassFIR(order,_lowpassCutoffFrequencyForLoadKinematics);
	} else {
		cout<<"\n\nNote- not filtering the external loads model kinematics.\n\n";
	}
	// Spline
	cout<<"\nConstruction function set for tracking...\n\n";
	GCVSplineSet qSet(5,qStore);
	Storage *uStore = qSet.constructStorage(1);
	

	// LOAD COP, FORCE, AND TORQUE
	Storage kineticsStore(_externalLoadsFileName.c_str());
	int copSize = kineticsStore.getSize();
	if(copSize<=0) return;

	// Read the indices of all the ground reaction data columns.
	// We assume that the right foot's data appears before the left foot's data
	// when reading the kinetics file's columns from left to right.
	int rightForceX  = kineticsStore.getColumnIndex("ground_force_vx");
	if(rightForceX<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_force_vx not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightForceY  = kineticsStore.getColumnIndex("ground_force_vy");
	if(rightForceY<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_force_vy not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightForceZ  = kineticsStore.getColumnIndex("ground_force_vz");
	if(rightForceZ<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_force_vz not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftForceX   = kineticsStore.getColumnIndex("ground_force_vx", rightForceX + 2);
	if(leftForceX<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_force_vx not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftForceY   = kineticsStore.getColumnIndex("ground_force_vy", rightForceY + 2);
	if(leftForceY<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_force_vy not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftForceZ   = kineticsStore.getColumnIndex("ground_force_vz", rightForceZ + 2);
	if(leftForceZ<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_force_vz not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightCopX    = kineticsStore.getColumnIndex("ground_force_px");
	if(rightCopX<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_force_px not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightCopY    = kineticsStore.getColumnIndex("ground_force_py");
	if(rightCopY<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_force_py not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightCopZ    = kineticsStore.getColumnIndex("ground_force_pz");
	if(rightCopZ<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_force_pz not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftCopX     = kineticsStore.getColumnIndex("ground_force_px", rightCopX + 2);
	if(leftCopX<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_force_px not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftCopY     = kineticsStore.getColumnIndex("ground_force_py", rightCopY + 2);
	if(leftCopY<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_force_py not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftCopZ     = kineticsStore.getColumnIndex("ground_force_pz", rightCopZ + 2);
	if(leftCopZ<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_force_pz not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightTorqueX = kineticsStore.getColumnIndex("ground_torque_x");
	if(rightTorqueX<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_torque_x not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightTorqueY = kineticsStore.getColumnIndex("ground_torque_y");
	if(rightTorqueY<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_torque_y not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightTorqueZ = kineticsStore.getColumnIndex("ground_torque_z");
	if(rightTorqueZ<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for right ground_torque_z not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftTorqueX  = kineticsStore.getColumnIndex("ground_torque_x", rightTorqueX + 2);
	if(leftTorqueX<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_torque_x not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftTorqueY  = kineticsStore.getColumnIndex("ground_torque_y", rightTorqueY + 2);
	if(leftTorqueY<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_torque_y not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftTorqueZ  = kineticsStore.getColumnIndex("ground_torque_z", rightTorqueZ + 2);
	if(leftTorqueZ<0) {
		string msg = "InvestigationCMCGait.run: ERR- Column index for left ground_torque_z not found in ";
		msg += _externalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// BODY INDICES
	// Right
	int rightFoot = _model->getBodyIndex(_externalLoadsBody1);
	if(rightFoot<0) {
		string msg = "InvestigationCMCGait.run: ERR- The body to which the first set of external loads";
		msg+="should be applied (" + _externalLoadsBody1 + ") is not a segment in the model.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	// Left
	//int  leftFoot = _model->getBodyIndex("calcn_l");
	int  leftFoot = _model->getBodyIndex(_externalLoadsBody2);
	if(leftFoot<0) {
		string msg = "InvestigationCMCGait.run: ERR- The body to which the second set of external loads";
		msg+="should be applied (" + _externalLoadsBody2 + ") is not a segment in the model.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	// Ground
	int ground = _model->getGroundID();


	// CREATE FORCE AND TORQUE APPLIERS
	ForceApplier *rightForceApp, *leftForceApp;
	TorqueApplier *rightTorqueApp, *leftTorqueApp;
	rightForceApp = new ForceApplier(_model, ground, rightFoot, &kineticsStore,
												rightForceX, rightForceY, rightForceZ,
												rightCopX, rightCopY, rightCopZ,
												qStore, uStore);
	leftForceApp  = new ForceApplier(_model, ground, leftFoot, &kineticsStore,
												leftForceX, leftForceY, leftForceZ,
												leftCopX, leftCopY, leftCopZ,
												qStore, uStore);
	rightTorqueApp = new TorqueApplier(_model, ground, rightFoot, &kineticsStore,
												rightTorqueX, rightTorqueY, rightTorqueZ);
	leftTorqueApp  = new TorqueApplier(_model, ground, leftFoot, &kineticsStore,
												leftTorqueX, leftTorqueY, leftTorqueZ);

	// Add force and torque appliers as derivative callbacks for model.
	// Set input in global frame is true by default--we're just being
	// paranoid here by setting it to true for sure.
	rightForceApp->setInputForcesInGlobalFrame(true);
	leftForceApp->setInputForcesInGlobalFrame(true);
	rightTorqueApp->setInputTorquesInGlobalFrame(true);
	leftTorqueApp->setInputTorquesInGlobalFrame(true);
	_model->addDerivCallback(rightForceApp);
	_model->addDerivCallback(leftForceApp);
	_model->addDerivCallback(rightTorqueApp);
	_model->addDerivCallback(leftTorqueApp);
}


