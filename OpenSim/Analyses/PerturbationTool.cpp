// PerturbationTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "PerturbationTool.h"
#include "ForwardTool.h"
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/ActuatorSet.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Contact.h>
#include <OpenSim/Analyses/LinearSpring.h>
#include <OpenSim/Analyses/TorsionalSpring.h>
#include <OpenSim/Analyses/ActuatorPerturbationIndependent.h>
#include <OpenSim/Analyses/ForceApplier.h>



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PerturbationTool::~PerturbationTool()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PerturbationTool::PerturbationTool() :
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_actuatorsToPerturb(_actuatorsToPerturbProp.getValueStrArray()),
	_perturbGravity(_perturbGravityProp.getValueBool()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_qFileName(_qFileNameProp.getValueStr()),
	_uFileName(_uFileNameProp.getValueStr()),
	_yFileName(_yFileNameProp.getValueStr()),
	_rHeelStrike(_rHeelStrikeProp.getValueDbl()),
	_rFootFlat(_rFootFlatProp.getValueDbl()),
	_rHeelOff(_rHeelOffProp.getValueDbl()),
	_rToeOff(_rToeOffProp.getValueDbl()),
	_lHeelStrike(_lHeelStrikeProp.getValueDbl()),
	_lFootFlat(_lFootFlatProp.getValueDbl()),
	_lHeelOff(_lHeelOffProp.getValueDbl()),
	_lToeOff(_lToeOffProp.getValueDbl()),
	_tau(_tauProp.getValueDbl()),
	_tauRightStart(_tauRightStartProp.getValueDbl()),
	_tauRightEnd(_tauRightEndProp.getValueDbl()),
	_tauLeftStart(_tauLeftStartProp.getValueDbl()),
	_tauLeftEnd(_tauLeftEndProp.getValueDbl()),
	_springTransitionStartWeight(_springTransitionStartWeightProp.getValueDbl()),
	_springTransitionEndWeight(_springTransitionEndWeightProp.getValueDbl()),
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("PerturbationTool");
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
PerturbationTool::PerturbationTool(const string &aFileName):
	SimulationTool(aFileName),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_actuatorsToPerturb(_actuatorsToPerturbProp.getValueStrArray()),
	_perturbGravity(_perturbGravityProp.getValueBool()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_qFileName(_qFileNameProp.getValueStr()),
	_uFileName(_uFileNameProp.getValueStr()),
	_yFileName(_yFileNameProp.getValueStr()),
	_rHeelStrike(_rHeelStrikeProp.getValueDbl()),
	_rFootFlat(_rFootFlatProp.getValueDbl()),
	_rHeelOff(_rHeelOffProp.getValueDbl()),
	_rToeOff(_rToeOffProp.getValueDbl()),
	_lHeelStrike(_lHeelStrikeProp.getValueDbl()),
	_lFootFlat(_lFootFlatProp.getValueDbl()),
	_lHeelOff(_lHeelOffProp.getValueDbl()),
	_lToeOff(_lToeOffProp.getValueDbl()),
	_tau(_tauProp.getValueDbl()),
	_tauRightStart(_tauRightStartProp.getValueDbl()),
	_tauRightEnd(_tauRightEndProp.getValueDbl()),
	_tauLeftStart(_tauLeftStartProp.getValueDbl()),
	_tauLeftEnd(_tauLeftEndProp.getValueDbl()),
	_springTransitionStartWeight(_springTransitionStartWeightProp.getValueDbl()),
	_springTransitionEndWeight(_springTransitionEndWeightProp.getValueDbl()),
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("PerturbationTool");
	setNull();
	updateFromXMLNode();
	if (_model) addAnalysisSetToModel();
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
PerturbationTool::
PerturbationTool(const PerturbationTool &aTool):
	SimulationTool(aTool),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_actuatorsToPerturb(_actuatorsToPerturbProp.getValueStrArray()),
	_perturbGravity(_perturbGravityProp.getValueBool()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_qFileName(_qFileNameProp.getValueStr()),
	_uFileName(_uFileNameProp.getValueStr()),
	_yFileName(_yFileNameProp.getValueStr()),
	_rHeelStrike(_rHeelStrikeProp.getValueDbl()),
	_rFootFlat(_rFootFlatProp.getValueDbl()),
	_rHeelOff(_rHeelOffProp.getValueDbl()),
	_rToeOff(_rToeOffProp.getValueDbl()),
	_lHeelStrike(_lHeelStrikeProp.getValueDbl()),
	_lFootFlat(_lFootFlatProp.getValueDbl()),
	_lHeelOff(_lHeelOffProp.getValueDbl()),
	_lToeOff(_lToeOffProp.getValueDbl()),
	_tau(_tauProp.getValueDbl()),
	_tauRightStart(_tauRightStartProp.getValueDbl()),
	_tauRightEnd(_tauRightEndProp.getValueDbl()),
	_tauLeftStart(_tauLeftStartProp.getValueDbl()),
	_tauLeftEnd(_tauLeftEndProp.getValueDbl()),
	_springTransitionStartWeight(_springTransitionStartWeightProp.getValueDbl()),
	_springTransitionEndWeight(_springTransitionEndWeightProp.getValueDbl()),
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* PerturbationTool::
copy() const
{
	PerturbationTool *object = new PerturbationTool(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void PerturbationTool::
setNull()
{
	setupProperties();

	_actuatorsToPerturb.setSize(1);
	_actuatorsToPerturb[0] = "all";
	_perturbGravity = true;

	// FOOT CONTACT EVENTS
	_rHeelStrike = _rFootFlat =_rHeelOff = _rToeOff = 0.0;
	_lHeelStrike = _lFootFlat =_lHeelOff = _lToeOff = 0.0;

	// CORRECTIVE SPRING PARAMETERS
	_tau = 0.001;
	_tauRightStart = _tauRightEnd = _tauLeftStart = _tauLeftEnd = _tau;
	_springTransitionStartWeight = 0.05;
	_springTransitionEndWeight = 0.25;
	_kLin.setSize(3);
	_kLin[0] = _kLin[1] = _kLin[2] = 5000000.0;
	_bLin.setSize(3);
	_bLin[0] = _bLin[1] = _bLin[2] = 1500.0;
	_kTor.setSize(3);
	_kTor[0] = _kTor[1] = _kTor[2] = 100000.0;
	_bTor.setSize(3);
	_bTor[0] = _bTor[1] = _bTor[2] = 1000.0;

	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = "";
	_externalLoadsBody2 = "";
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PerturbationTool::setupProperties()
{
	string comment;

	// PERTURBATION PARAMETERS
	comment = "Time over which the model states are integrated following a perturbation. "
				 "To allow reaction forces to adjust to adjust to the perturbation, the recommended "
				 "window is from about 0.020 seconds to about 0.040 seconds, although it may differ "
				 "from model to model.";
	_pertWindowProp.setComment(comment);
	_pertWindowProp.setName("perturbation_time_window");
	_propertySet.append( &_pertWindowProp );

	comment = "Time increment between perturbation windows.  This can be smaller than "
				 "the perturbation time window size (e.g., 0.010 sec).";
	_pertIncrementProp.setComment(comment);
	_pertIncrementProp.setName("perturbation_time_increment");
	_propertySet.append( &_pertIncrementProp );

	comment = "Magnitude of perturbation. Perturbation results should be fairly insensitive to "
				 "the perturbation size.  Values between 0.10N and 10N should be fine.";
	_pertDFProp.setComment(comment);
	_pertDFProp.setName("perturbation_size");
	_propertySet.append( &_pertDFProp);

	_actuatorsToPerturbProp.setComment("List of actuator names to be perturbed (for which induced acceleration will be computed).  "
												  "Use 'all' to perturb all actuators.");
	_actuatorsToPerturbProp.setName("perturbed_actuators");
	_propertySet.append( &_actuatorsToPerturbProp );

	_perturbGravityProp.setComment("Whether or not to compute induced accelerations due to gravity perturbation.");
	_perturbGravityProp.setName("perturb_gravity");
	_propertySet.append( &_perturbGravityProp );

	// CORRECTIVE SPRING PARAMETERS
	comment = "Rise time for scaling functions.  This parameter determines how fast a corrective "
				 "spring is scaled into or out of effect around contact events.";
	_tauProp.setComment(comment);
	_tauProp.setName("scaling_rise_time");
	_propertySet.append( &_tauProp );

	comment = "Override scaling_rise_time for right foot flat (transition in).";
	_tauRightStartProp.setComment(comment);
	_tauRightStartProp.setName("scaling_rise_time_right_start");
	_propertySet.append( &_tauRightStartProp );

	comment = "Override scaling_rise_time for right heel off (transition out).";
	_tauRightEndProp.setComment(comment);
	_tauRightEndProp.setName("scaling_rise_time_right_end");
	_propertySet.append( &_tauRightEndProp );

	comment = "Override scaling_rise_time for left foot flat (transition in).";
	_tauLeftStartProp.setComment(comment);
	_tauLeftStartProp.setName("scaling_rise_time_left_start");
	_propertySet.append( &_tauLeftStartProp );

	comment = "Override scaling_rise_time for left heel off (transition out).";
	_tauLeftEndProp.setComment(comment);
	_tauLeftEndProp.setName("scaling_rise_time_left_end");
	_propertySet.append( &_tauLeftEndProp );

	comment = "Percent of body weight at which linear springs start to transition in.";
	_springTransitionStartWeightProp.setComment(comment);
	_springTransitionStartWeightProp.setName("spring_transition_start_weight");
	_propertySet.append( &_springTransitionStartWeightProp );

	comment = "Percent of body weight past which linear springs are fully activated.";
	_springTransitionEndWeightProp.setComment(comment);
	_springTransitionEndWeightProp.setName("spring_transition_end_weight");
	_propertySet.append( &_springTransitionEndWeightProp );

	_kLinProp.setComment("Stiffness for linear (translational) corrective springs");
	_kLinProp.setName("corrective_spring_linear_stiffness");
	_propertySet.append( &_kLinProp );

	_bLinProp.setComment("Damping for linear (translational) corrective springs");
	_bLinProp.setName("corrective_spring_linear_damping");
	_propertySet.append( &_bLinProp );

	_kTorProp.setComment("Stiffness for torsional corrective springs");
	_kTorProp.setName("corrective_spring_torsional_stiffness");
	_propertySet.append( &_kTorProp );

	_bTorProp.setComment("Damping for torsional corrective springs");
	_bTorProp.setName("corrective_spring_torsional_damping");
	_propertySet.append( &_bTorProp );


	// INPUT FILE NAMES
	comment = "XML file containing the controls (e.g., muscle excitations) for the forward simulation.";
	_controlsFileNameProp.setComment(comment);
	_controlsFileNameProp.setName("controls_file");
	_propertySet.append( &_controlsFileNameProp );

	comment = "Storage file (.sto) containing the generalized coordinates for the model. "
				 "These coordinates are used in conjuction with the center of pressure data "
				 "to place the corrective springs at the centers of pressure in the local frames "
				 "of each foot."; 
	_qFileNameProp.setComment(comment);
	_qFileNameProp.setName("coordinates_file");
	_propertySet.append( &_qFileNameProp );

	comment = "Storage file (.sto) containing the generalized speeds for the model. "
				 "These speeds are used to set the expected velocities of the centers of pressure "
				 "so that damping can be applied by the corrective springs if the trajectory "
				 "of the foot deviates from the expected trajectory."; 
	_uFileNameProp.setComment(comment);
	_uFileNameProp.setName("speeds_file");
	_propertySet.append( &_uFileNameProp );

	comment = "Storage file (.sto) containing the model states during the unperturbed simulation. "
				 "These states are used to set the initial states of the model for each perturbed simulation.";
	_yFileNameProp.setComment(comment);
	_yFileNameProp.setName("states_file");
	_propertySet.append( &_yFileNameProp );

	// FOOT CONTACT EVENT TIMES
	_rHeelStrikeProp.setComment("Time of right heel strike.  The linear corrective spring will increase its influence at this time.");
	_rHeelStrikeProp.setName("r_heel_strike");
	_propertySet.append( &_rHeelStrikeProp );

	_rFootFlatProp.setComment("Time of right foot flat.  The torsional corrective spring will increase its influence at this time.");
	_rFootFlatProp.setName("r_foot_flat");
	_propertySet.append( &_rFootFlatProp );

	_rHeelOffProp.setComment("Time of right heel off.  The torsional corrective spring will decrease its influence at this time.");
	_rHeelOffProp.setName("r_heel_off");
	_propertySet.append( &_rHeelOffProp );

	_rToeOffProp.setComment("Time of right toe off.  The linear corrective spring will decrease its influence at this time.");
	_rToeOffProp.setName("r_toe_off");
	_propertySet.append( &_rToeOffProp );

	_lHeelStrikeProp.setComment("Time of left heel strike.  The linear corrective spring will increase its influence at this time");
	_lHeelStrikeProp.setName("l_heel_strike");
	_propertySet.append( &_lHeelStrikeProp );

	_lFootFlatProp.setComment("Time of left foot flat.  The torsional corrective spring will increase its influence at this time");
	_lFootFlatProp.setName("l_foot_flat");
	_propertySet.append( &_lFootFlatProp );

	_lHeelOffProp.setComment("Time of left heel off.  The torsional corrective spring will decrease its influence at this time");
	_lHeelOffProp.setName("l_heel_off");
	_propertySet.append( &_lHeelOffProp );

	_lToeOffProp.setComment("Time of left toe off.  The linear corrective spring will decrease its influence at this time");
	_lToeOffProp.setName("l_toe_off");
	_propertySet.append( &_lToeOffProp );

	// EXTERNAL LOADS (e.g. GROUND REACTION FORCES)
	comment = "Motion file (.mot) or storage file (.sto) containing the external loads applied to the model.";
	_externalLoadsFileNameProp.setComment(comment);
	_externalLoadsFileNameProp.setName("external_loads_file");
	_propertySet.append( &_externalLoadsFileNameProp );

	comment = "Motion file (.mot) or storage file (.sto) containing the model kinematics corresponding to the external loads.";
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
PerturbationTool& PerturbationTool::
operator=(const PerturbationTool &aTool)
{
	// BASE CLASS
	SimulationTool::operator=(aTool);

	// MEMEBER VARIABLES

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
void PerturbationTool::run()
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
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	// Gather actuators to perturb
	ActuatorSet *as = _model->getActuatorSet();
	ArrayPtrs<AbstractActuator> actuators;
	actuators.setMemoryOwner(false);
	actuators.setSize(_actuatorsToPerturb.getSize());
	for(int i=0; i<_actuatorsToPerturb.getSize(); i++) {
		if(_actuatorsToPerturb[i] == "all") {
			actuators.setSize(as->getSize());
			for(int j=0;j<as->getSize();j++) actuators.set(j,as->get(j));
			break;
		}
		int index = as->getIndex(_actuatorsToPerturb[i]);
		if(index<0) throw Exception("PerturbationTool: ERR- Could not find actuator '"+_actuatorsToPerturb[i]+
										    "' (listed in "+_actuatorsToPerturbProp.getName()+") in model's actuator set.",__FILE__,__LINE__);
		actuators.set(i,as->get(index));
	}

	if(_actuatorsToPerturb.getSize() == 0 && !_perturbGravity)
		//throw Exception("PerturbationTool: ERR- Nothing to perturb (no actuators selected and gravity perturbation disabled).",__FILE__,__LINE__);
		cout << "PerturbationTool: WARNING- Nothing will be perturbed (no actuators to perturb and gravity perturbation is off)" << endl;

	// GROUND REACTION FORCES
	ForceApplier *rightGRFApp, *leftGRFApp;
	ForwardTool::initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics,&rightGRFApp,&leftGRFApp);

	// CONSTRUCT CORRECTIVE SPRINGS
	constructCorrectiveSprings(rightGRFApp,leftGRFApp);

	// INPUT
	// Controls
	cout<<endl<<endl<<"Loading controls from file "<<_controlsFileName<<".\n";
	ControlSet *controlSet = new ControlSet(_controlsFileName);
	cout<<"Found "<<controlSet->getSize()<<" controls.\n\n";
	// States
	Storage *yStore = new Storage(_yFileName);

	// Add actuation analysis -- needed in order to evaluate unperturbed forces
	// Actuation
	Actuation *actuation = new Actuation(_model);
	_model->addAnalysis(actuation);

	Kinematics *kin=0;
	BodyKinematics *bodyKin=0;
	AnalysisSet &ans=getAnalysisSet();
	for(int i=0;i<ans.getSize();i++) {
		if(!ans.get(i)->getOn()) continue;
		if(dynamic_cast<Kinematics*>(ans.get(i))) kin=dynamic_cast<Kinematics*>(ans.get(i));
		else if(dynamic_cast<BodyKinematics*>(ans.get(i))) bodyKin=dynamic_cast<BodyKinematics*>(ans.get(i));
	}

	// SETUP SIMULATION
	// Manager
	ModelIntegrand integrand(_model);
	integrand.setControlSet(*controlSet);
	Manager manager(&integrand);
	manager.setSessionName(getName());
	// Initial and final times
	// If the times lie outside the range for which control values are
	// available, the initial and final times are altered.
	int first = 0;
	ControlLinear *control = (ControlLinear*)controlSet->get(first);
	if(control==NULL) {
		cout<<"\n\nError- There are no controls.\n\n";
		exit(-1);
	}
	double ti = control->getFirstTime();
	double tf = control->getLastTime();
	// Check initial time.
	if(_ti<ti) {
		cout<<"\n\nControls not available at "<<_ti<<".  ";
		cout<<"Changing initial time to "<<ti<<".";
		_ti = ti;
	}
	// Check final time.
	if(tf<_tf) {
		cout<<"\n\nControls not available at "<<_tf<<".  ";
		cout<<"Changing final time to "<<tf<<".";
		_tf = tf;
	}
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);
	cout<<"\n\nPerforming perturbations over the range ti=";
	cout<<_ti<<" to tf="<<_tf<<endl<<endl;

	// Integrator settings
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(_maxSteps);
	integ->setMaxDT(_maxDT);
	integ->setTolerance(_errorTolerance);
	integ->setFineTolerance(_fineTolerance);

	// Pertubation callback
	ActuatorPerturbationIndependent *perturbation = 
		new ActuatorPerturbationIndependent(_model);
	_model->addDerivCallback(perturbation);

	int gravity_axis = 1;
	double original_gravity[3];
	int nperturb = actuators.getSize() + (_perturbGravity ? 1 : 0);

	if(_perturbGravity) _model->getGravity(original_gravity);

	string columnLabels = "time";
	for(int i=0; i<actuators.getSize(); i++)
		columnLabels += "\t" + actuators.get(i)->getName();
	if(_perturbGravity)
		columnLabels += "\tgravity";

	// Figure out which columns are being recorded in the Kinematics and BodyKinematics analyses...
	// but make sure to ignore the first (time) column
	int ncoords = kin ? kin->getPositionStorage()->getColumnLabelsArray().getSize()-1 : 0;
	int nbodycoords = bodyKin ? bodyKin->getPositionStorage()->getColumnLabelsArray().getSize()-1 : 0;
	int nvalues = ncoords + nbodycoords;
	if(nvalues==0) 
		throw Exception("PerturbationTool.run: ERROR- No (active) analyses found -- no perturbation to compute.",__FILE__,__LINE__);

	Array<string> values_name("",nvalues);
	for(int i=0;i<ncoords;i++)
		values_name[i] = kin->getPositionStorage()->getColumnLabelsArray()[i+1];
	for(int i=0;i<nbodycoords;i++)
		values_name[ncoords+i] = bodyKin->getPositionStorage()->getColumnLabelsArray()[i+1];

	cout << "PERTURBED DATA:" << endl;
	if(actuators.getSize()) {
		cout << "Actuators:";
		for(int i=0; i<actuators.getSize(); i++) cout << (i>0?", ":"") << actuators.get(i)->getName();
		cout << endl;
	}
	if(_perturbGravity) cout << "Gravity perturbation on" << endl;
	cout << endl;

	cout << "MEASURED INDUCED ACCELERATIONS:" << endl;
	if(ncoords) {
		cout << "Kinematics: ";
		for(int i=0;i<ncoords;i++) {
			values_name[i] = kin->getPositionStorage()->getColumnLabelsArray()[i+1];
			cout << (i>0?", ":"") << values_name[i];
		}
		cout << endl;
	}
	if(nbodycoords) {
		cout << "BodyKinematics: ";
		for(int i=0;i<nbodycoords;i++) {
			values_name[ncoords+i] = bodyKin->getPositionStorage()->getColumnLabelsArray()[i+1];
			cout << (i>0?", ":"") << values_name[ncoords+i];
		}
		cout << endl;
	}

	// The first ncoords values in the values_* arrays refer to generalized coordinates that are being measured; the
	// remaining nbodycoords values refer to body coordinates that are being measured.
	Array<double> values_unperturbed(0.0,nvalues);
	ArrayPtrs<Array<double> > values_perturbed, values_dAdF;
	values_perturbed.setSize(nvalues);
	values_dAdF.setSize(nvalues);
	ArrayPtrs<Storage> values_perturbedStorage, values_dAdFStorage, values_deltaAStorage;
	values_perturbedStorage.setSize(nvalues);
	values_dAdFStorage.setSize(nvalues);
	values_deltaAStorage.setSize(nvalues);
	for(int i=0; i<nvalues; i++) {
		values_perturbed.set(i,new Array<double>(0.0,nperturb));
		values_dAdF.set(i,new Array<double>(0.0,nperturb));
		values_perturbedStorage.set(i,new Storage);
		values_dAdFStorage.set(i,new Storage);
		values_deltaAStorage.set(i,new Storage);
		values_perturbedStorage[i]->setName(values_name[i]+"_perturbed");
		values_perturbedStorage[i]->setColumnLabels(columnLabels.c_str());
		values_dAdFStorage[i]->setName(values_name[i]+"_dAdF");
		values_dAdFStorage[i]->setColumnLabels(columnLabels.c_str());
		values_deltaAStorage[i]->setName(values_name[i]+"_deltaA");
		values_deltaAStorage[i]->setColumnLabels(columnLabels.c_str());
	}

	Storage unperturbedAccelStorage;
	unperturbedAccelStorage.setName("unperturbedAccel");
	columnLabels = "time";
	for(int i=0;i<nvalues;i++) columnLabels += "\t" + values_name[i];
	unperturbedAccelStorage.setColumnLabels(columnLabels.c_str());

	Array<double> deltaA(0,nperturb);

	// From now on we'll only need the last state vectors recorded in these analyses, so we
	// set their step interval to a large number to avoid them computing and writing their
	// data at the (many) individual integration steps.
	if(kin) kin->setStepInterval(1000000);
	if(bodyKin) bodyKin->setStepInterval(1000000);
	actuation->setStepInterval(1000000);

	IO::makeDir(getResultsDir());

	//********************************************************************
	// LOOP
	//********************************************************************
	double lastPertTime = _tf - _pertWindow;
	for(double t=_ti;t<=lastPertTime;t+=_pertIncrement) {
		// SET INITIAL AND FINAL TIME AND THE INITIAL STATES
		int index = yStore->findIndex(t);
		double tiPert;
		yStore->getTime(index,tiPert);
		double tfPert = tiPert + _pertWindow;
		manager.setInitialTime(tiPert);
		manager.setFinalTime(tfPert);
		const Array<double> &yi = yStore->getStateVector(index)->getData();
		_model->setInitialStates(&yi[0]);

		// RESET ANALYSES
		if(kin) {
			kin->getPositionStorage()->reset();
			kin->getVelocityStorage()->reset();
			kin->getAccelerationStorage()->reset();
		}
		if(bodyKin) {
			bodyKin->getPositionStorage()->reset();
			bodyKin->getVelocityStorage()->reset();
			bodyKin->getAccelerationStorage()->reset();
		}
		actuation->getForceStorage()->reset();

		// INTEGRATE (1)
		integ->setUseSpecifiedDT(false);
		perturbation->setOn(false);
		cout<<"\n\nUnperturbed integration (1) from "<<tiPert<<" to "<<tfPert<<endl;
		manager.integrate();
		
		// record unperturbed accelerations:
		Array<double> unperturbedAccel(0.0, nvalues);
		if(kin) {
			const Array<double> &posStart = kin->getPositionStorage()->getStateVector(0)->getData();
			const Array<double> &velStart = kin->getVelocityStorage()->getStateVector(0)->getData();
			const Array<double> &posEnd = kin->getPositionStorage()->getLastStateVector()->getData();
			double dt=kin->getPositionStorage()->getLastTime() - kin->getPositionStorage()->getFirstTime();
			if(dt>1e-8) for(int i=0;i<ncoords;i++) unperturbedAccel[i] = 2*(posEnd[i]-posStart[i]-dt*velStart[i])/(dt*dt);
		}
		if(bodyKin) {
			const Array<double> &posStart = bodyKin->getPositionStorage()->getStateVector(0)->getData();
			const Array<double> &velStart = bodyKin->getVelocityStorage()->getStateVector(0)->getData();
			const Array<double> &posEnd = bodyKin->getPositionStorage()->getLastStateVector()->getData();
			double dt=bodyKin->getPositionStorage()->getLastTime() - bodyKin->getPositionStorage()->getFirstTime();
			if(dt>1e-8) for(int i=0;i<nbodycoords;i++) unperturbedAccel[ncoords+i] = 2*(posEnd[i]-posStart[i]-dt*velStart[i])/(dt*dt);
		}
		unperturbedAccelStorage.append(tiPert, nvalues, &unperturbedAccel[0]);
		char fileName[Object::NAME_LENGTH];
		sprintf(fileName,"%s/%s_unperturbedAccel_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),getName().c_str(),_pertWindow,_pertDF);
		unperturbedAccelStorage.print(fileName);

		// If we are not perturbing any actuators/gravity, we must only be computing unperturbed accelerations...
		// so can skip rest of this loop body.
		if(nperturb==0) continue;

		// INTEGRATE (2) - Record unperturbed muscle forces
		integ->setUseSpecifiedDT(true);
		perturbation->setOn(true);
		perturbation->getUnperturbedForceStorage()->reset(); 
		perturbation->setRecordUnperturbedForces(true);
		cout<<"\nUnperturbed integration 2 to record forces and kinematics\n";
		manager.integrate();
		perturbation->setRecordUnperturbedForces(false);

		// Get unperturbed values (concatenate generalized coordinates and body coordinates)
		if(kin) {
			const Array<double> &unperturbedCoordinates = kin->getPositionStorage()->getLastStateVector()->getData(); // at end of timestep
			for(int i=0;i<ncoords;i++) values_unperturbed[i] = unperturbedCoordinates[i];
		}
		if(bodyKin) {
			const Array<double> &unperturbedBodyCoordinates = bodyKin->getPositionStorage()->getLastStateVector()->getData(); // at end of timestep
			for(int i=0;i<nbodycoords;i++) values_unperturbed[ncoords+i] = unperturbedBodyCoordinates[i];
		}

		// Unperturbed forces
		Array<double> unperturbedForces = actuation->getForceStorage()->getStateVector(0)->getData(); // at BEGINNING of timestep
		// include unperturbed gravity value if doing gravity perturbation
		if(_perturbGravity) unperturbedForces.append(original_gravity[gravity_axis]);

		// Loop over actuators/gravity to be perturbed
		for (int m=0;m<nperturb;m++)	{
			_model->getDerivCallbackSet()->resetCallbacks();
			perturbation->reset(); 
			if(m<actuators.getSize()) {
				AbstractActuator *act = actuators.get(m);
				// Set up pertubation callback
				cout<<"\nPerturbation of muscle "<<act->getName()<<" ("<<m<<") in loop"<<endl;
				perturbation->setActuator(act); 
				perturbation->setPerturbation(ActuatorPerturbationIndependent::DELTA,+_pertDF);
			} else {
				cout<<"\nGravity perturbation"<<endl;
				perturbation->setActuator(0); 
				double grav[3];
				for(int i=0;i<3;i++) grav[i]=original_gravity[i];
				grav[gravity_axis] += _pertDF;
				_model->setGravity(grav);
			}

			// Integrate
 			manager.integrate();

			if(m<actuators.getSize()) {
				cout << "actuator:\t"<<m<<"\tforce:\t"<<unperturbedForces[m]<<endl;
			} else {
				cout << "gravity original:\t"<<unperturbedForces[m]<<endl;
				// undo gravity perturbation
				_model->setGravity(original_gravity);
			}

			// Perturbed generalized coordinate values (concatenate into values_perturbed array)
			if(kin) {
				const Array<double> &perturbedCoordinates = kin->getPositionStorage()->getLastStateVector()->getData();
				for(int i=0;i<ncoords;i++) (*values_perturbed[i])[m] = perturbedCoordinates[i];
			}
			if(bodyKin) {
				const Array<double> &perturbedBodyCoordinates = bodyKin->getPositionStorage()->getLastStateVector()->getData();
				for(int i=0;i<nbodycoords;i++) (*values_perturbed[ncoords+i])[m] = perturbedBodyCoordinates[i];
			}

			// COMPUTE DERIVATIVES
			for(int i=0;i<nvalues;i++) {
				double perturbed = (*values_perturbed[i])[m], unperturbed = values_unperturbed[i];
				double dAdF = 2*(perturbed - unperturbed)/(_pertDF*_pertWindow*_pertWindow);
				(*values_perturbed[i])[m] = perturbed;
				(*values_dAdF[i])[m] = dAdF;

				cout << values_name[i] << ": perturbed="<<perturbed<<"  unperturbed="<<unperturbed<<"  diff="<<perturbed-unperturbed
					  <<"  dAdF="<<dAdF<<"  deltaA="<<unperturbedForces[m]*dAdF<<endl;
			}
		} //end muscle loop

		// Append to storage objects
		for(int i=0;i<nvalues;i++) {
			const Array<double> &perturbed = *values_perturbed[i];
			const Array<double> &dAdF = *values_dAdF[i];
			values_perturbedStorage[i]->append(tiPert,nperturb,&perturbed[0]);
			values_dAdFStorage[i]->append(tiPert,nperturb,&dAdF[0]);
			for(int m=0;m<nperturb;m++) deltaA[m] = unperturbedForces[m] * dAdF[m];
			values_deltaAStorage[i]->append(tiPert,nperturb,&deltaA[0]);
		}
	
		// Print results
		for(int i=0;i<nvalues;i++) {
			sprintf(fileName,"%s/%s_%s_perturbed_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),getName().c_str(),values_name[i].c_str(),_pertWindow,_pertDF);
			values_perturbedStorage[i]->print(fileName);
			sprintf(fileName,"%s/%s_%s_dAdF_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),getName().c_str(),values_name[i].c_str(),_pertWindow,_pertDF);
			values_dAdFStorage[i]->print(fileName);
			sprintf(fileName,"%s/%s_%s_deltaA_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),getName().c_str(),values_name[i].c_str(),_pertWindow,_pertDF);
			values_deltaAStorage[i]->print(fileName);
		}
	} // end time loop
	//***************************************************************************
	IO::chDir(saveWorkingDirectory);
}


//_____________________________________________________________________________
/**
 * Construct the corrective springs.
 */
void PerturbationTool::
constructCorrectiveSprings(ForceApplier *aRightGRFApp, ForceApplier *aLeftGRFApp)
{
	// Qs and Us
	cout<<"\n\nLoading generalized coordinates and speeds from files "
	    <<_qFileName<<" and "<<_uFileName<<".\n";
	Storage qStore(_qFileName);
	Storage uStore(_uFileName);

	// CONVERT Qs AND Us TO RADIANS AND QUATERNIONS
	_model->getDynamicsEngine().convertDegreesToRadians(&qStore);
	_model->getDynamicsEngine().convertAnglesToQuaternions(&qStore);
	_model->getDynamicsEngine().convertDegreesToRadians(&uStore);

	// SCALING FUNCTIONS FOR SPRINGS
	double dtScale=0.001;
	double tiScale = qStore.getFirstTime();
	double tfScale = qStore.getLastTime();
	Array<double> timeScale(0.0);
	Array<double> rLinearScale(0.0),rTorsionalScale(0.0);
	Array<double> lLinearScale(0.0),lTorsionalScale(0.0);

	double tauRStart = _tauRightStartProp.getUseDefault() ? _tau : _tauRightStart;
	double tauREnd = _tauRightEndProp.getUseDefault() ? _tau : _tauRightEnd;
	double tauLStart = _tauLeftStartProp.getUseDefault() ? _tau : _tauLeftStart;
	double tauLEnd = _tauLeftEndProp.getUseDefault() ? _tau : _tauLeftEnd;

	cout << "Spring parameters:" << endl;
	cout << "\tSpring transition weights: " << _springTransitionStartWeight << " " << _springTransitionEndWeight << endl;
	cout << "\tTau values: right_start = " << tauRStart << ", right_end = " << tauREnd << ", left_start = " << tauLStart << ", left_end = " << tauLEnd << endl;

	for(double tScale=tiScale;tScale<=tfScale;tScale+=dtScale) {
		// time
		timeScale.append(tScale);
		double value1,value2;

		// Use vertical GRF to transition linear springs
		double r_grf[3], l_grf[3];
		aRightGRFApp->getForceFunction()->evaluate(&tScale,r_grf);
		aLeftGRFApp->getForceFunction()->evaluate(&tScale,l_grf);
		double r_grf_y_norm = rdMath::Clamp(r_grf[1]/(r_grf[1]+l_grf[1]),0.,1.);
		double l_grf_y_norm = 1-r_grf_y_norm;
		rLinearScale.append(rdMath::Step(r_grf_y_norm,_springTransitionStartWeight,_springTransitionEndWeight));
		lLinearScale.append(rdMath::Step(l_grf_y_norm,_springTransitionStartWeight,_springTransitionEndWeight));

		// Use different tau's to transition torsional springs
		// rTorsional
		value1 = rdMath::SigmaUp(tauRStart,_rFootFlat,tScale);
		value2 = rdMath::SigmaDn(tauREnd,_rHeelOff,tScale);
		rTorsionalScale.append(value1+value2-1.0);
		// lTorsional
		value1 = rdMath::SigmaUp(tauLStart,_lFootFlat,tScale);
		value2 = rdMath::SigmaDn(tauLEnd,_lHeelOff,tScale);
		lTorsionalScale.append(value1+value2-1.0);
	}
	// Create Splines
	GCVSpline *rScaleTranslationalSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&rLinearScale[0]);
	rScaleTranslationalSpline->setName("Right_Translational");
	GCVSpline *rScaleTorsionalSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&rTorsionalScale[0]);
	rScaleTorsionalSpline->setName("Right_Torsional");
	GCVSpline *lScaleTranslationalSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&lLinearScale[0]);
	lScaleTranslationalSpline->setName("Left_Translational");
	GCVSpline *lScaleTorsionalSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&lTorsionalScale[0]);
	lScaleTorsionalSpline->setName("Left_Torsional");

	// TODO: this assumes that external loads body 1 is right foot, and external loads body 2 is left foot
	if(_externalLoadsBody1.size() && _externalLoadsBody1[_externalLoadsBody1.size()-1]=='l' || 
		_externalLoadsBody2.size() && _externalLoadsBody2[_externalLoadsBody2.size()-1]=='r')
		throw Exception("PerturbationTool: ERR- External loads body 1 should be right foot and external loads body 2 should be left foot",__FILE__,__LINE__);

	AbstractBody *rightFootBody = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody1);
	if(!rightFootBody) throw Exception("PerturbationTool: ERR- Could not find right foot body '"+_externalLoadsBody1+"'",__FILE__,__LINE__);
	AbstractBody *leftFootBody = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody2);
	if(!leftFootBody) throw Exception("PerturbationTool: ERR- Could not find left foot body '"+_externalLoadsBody2+"'",__FILE__,__LINE__);

	// LINEAR
	// right
	LinearSpring *rLin = new LinearSpring(_model,rightFootBody);
	// Use the same foot-frame COP positions as the GRF force applier
	rLin->setPointFunction((VectorFunction*)aRightGRFApp->getPointFunction()->copy());
	rLin->computeTargetFunctions(&qStore,&uStore);
	rLin->setKValue(&_kLin[0]);
	rLin->setBValue(&_bLin[0]);
	rLin->setScaleFunction(rScaleTranslationalSpline);
	_model->addDerivCallback(rLin);
	// left linear
	LinearSpring *lLin = new LinearSpring(_model,leftFootBody);
	// Use the same foot-frame COP positions as the GRF force applier
	lLin->setPointFunction((VectorFunction*)aLeftGRFApp->getPointFunction()->copy());
	lLin->computeTargetFunctions(&qStore,&uStore);
	lLin->setKValue(&_kLin[0]);
	lLin->setBValue(&_bLin[0]);
	lLin->setScaleFunction(lScaleTranslationalSpline);
	_model->addDerivCallback(lLin);

	// TORSIONAL
	// right
	TorsionalSpring *rTrq = new TorsionalSpring(_model,rightFootBody);
	rTrq->computeTargetFunctions(&qStore,&uStore);
	rTrq->setKValue(&_kTor[0]);
	rTrq->setBValue(&_bTor[0]);
	rTrq->setScaleFunction(rScaleTorsionalSpline);
	_model->addDerivCallback(rTrq);
	// left
	TorsionalSpring *lTrq = new TorsionalSpring(_model,leftFootBody);
	lTrq->computeTargetFunctions(&qStore,&uStore);
	lTrq->setKValue(&_kTor[0]);
	lTrq->setBValue(&_bTor[0]);
	lTrq->setScaleFunction(lScaleTorsionalSpline);
	_model->addDerivCallback(lTrq);


	string labels = "time\tr_scale_linear\tl_scale_linear\tr_scale_torsional\tl_scale_torsional\tr_stance\tl_stance\tr_footflat\tl_footflat";
	Storage debugStorage;
	debugStorage.setColumnLabels(labels.c_str());
	for(double tScale=tiScale;tScale<=tfScale;tScale+=dtScale) {
		double values[8] = {rScaleTranslationalSpline->evaluate(0,tScale), 
								  lScaleTranslationalSpline->evaluate(0,tScale),
								  rScaleTorsionalSpline->evaluate(0,tScale),
								  lScaleTorsionalSpline->evaluate(0,tScale),
								  (_rHeelStrike<=tScale && tScale<=_rToeOff)?1:0,
								  (_lHeelStrike<=tScale && tScale<=_lToeOff)?1:0,
								  (_rFootFlat<=tScale && tScale<=_rHeelOff)?1:0,
								  (_lFootFlat<=tScale && tScale<=_lHeelOff)?1:0};
		debugStorage.append(tScale,8,values);
	}
	debugStorage.print("spring_scales.sto");
	cout << "Wrote out spring_scales.sto for debugging spring scaling" << endl;
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
void PerturbationTool::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	cout<<"PerturbationTool.printResults: ";
	cout<<"Printing results of investigation "<<getName()<<".\n";
}
