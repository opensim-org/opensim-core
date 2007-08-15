// ForwardTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "ForwardTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Mtx.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Common/InterruptedException.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Actuators/ForceApplier.h>
#include <OpenSim/Actuators/TorqueApplier.h>
#include <OpenSim/Actuators/LinearSpring.h>
#include <OpenSim/Actuators/TorsionalSpring.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
ForwardTool::~ForwardTool()
{
	delete _integrand;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ForwardTool::ForwardTool() :
	AbstractTool(),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_rLinSpringOn(_rLinSpringOnProp.getValueBool()),
	_rTorSpringOn(_rTorSpringOnProp.getValueBool()),
	_lLinSpringOn(_lLinSpringOnProp.getValueBool()),
	_lTorSpringOn(_lTorSpringOnProp.getValueBool()),
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
	_springTransitionStartForce(_springTransitionStartForceProp.getValueDbl()),
	_springTransitionEndForce(_springTransitionEndForceProp.getValueDbl()),
	_forceThreshold(_forceThresholdProp.getValueDbl()),
	_torqueThreshold(_torqueThresholdProp.getValueDbl()),
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_outputDetailedResults(_outputDetailedResultsProp.getValueBool())
{
	setType("ForwardTool");
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
ForwardTool::ForwardTool(const string &aFileName, bool aLoadModel) :
	AbstractTool(aFileName, false),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_rLinSpringOn(_rLinSpringOnProp.getValueBool()),
	_rTorSpringOn(_rTorSpringOnProp.getValueBool()),
	_lLinSpringOn(_lLinSpringOnProp.getValueBool()),
	_lTorSpringOn(_lTorSpringOnProp.getValueBool()),
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
	_springTransitionStartForce(_springTransitionStartForceProp.getValueDbl()),
	_springTransitionEndForce(_springTransitionEndForceProp.getValueDbl()),
	_forceThreshold(_forceThresholdProp.getValueDbl()),
	_torqueThreshold(_torqueThresholdProp.getValueDbl()),
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_outputDetailedResults(_outputDetailedResultsProp.getValueBool())
{
	setType("ForwardTool");
	setNull();
	updateFromXMLNode();
	if(aLoadModel) loadModel(aFileName);
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
ForwardTool::
ForwardTool(const ForwardTool &aTool) :
	AbstractTool(aTool),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_rLinSpringOn(_rLinSpringOnProp.getValueBool()),
	_rTorSpringOn(_rTorSpringOnProp.getValueBool()),
	_lLinSpringOn(_lLinSpringOnProp.getValueBool()),
	_lTorSpringOn(_lTorSpringOnProp.getValueBool()),
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
	_springTransitionStartForce(_springTransitionStartForceProp.getValueDbl()),
	_springTransitionEndForce(_springTransitionEndForceProp.getValueDbl()),
	_forceThreshold(_forceThresholdProp.getValueDbl()),
	_torqueThreshold(_torqueThresholdProp.getValueDbl()),
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_outputDetailedResults(_outputDetailedResultsProp.getValueBool())
{
	setType("ForwardTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* ForwardTool::
copy() const
{
	ForwardTool *object = new ForwardTool(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void ForwardTool::
setNull()
{
	setupProperties();

	// BASIC
	_controlsFileName = "";
	_statesFileName = "";
	_useSpecifiedDt = false;
	_printResultFiles = true;

	// EXTERNAL LOADS
	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = "";
	_externalLoadsBody2 = "";
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;

	// CORRECTIVE SPING ON FLAGS
	_rLinSpringOn = _rTorSpringOn = false;
	_lLinSpringOn = _lTorSpringOn = false;

	// FOOT CONTACT EVENTS
	_rHeelStrike = _rFootFlat =_rHeelOff = _rToeOff = 0.0;
	_lHeelStrike = _lFootFlat =_lHeelOff = _lToeOff = 0.0;

	// CORRECTIVE SPRING PARAMETERS
	_tau = 0.001;
	_tauRightStart = _tauRightEnd = _tauLeftStart = _tauLeftEnd = _tau;
	_springTransitionStartForce = 1.0;
	_springTransitionEndForce = 50.0;
	_forceThreshold = 0.0;
	_torqueThreshold = 0.0;
	_kLin.setSize(3);
	_kLin[0] = _kLin[1] = _kLin[2] = 5000000.0;
	_bLin.setSize(3);
	_bLin[0] = _bLin[1] = _bLin[2] = 1500.0;
	_kTor.setSize(3);
	_kTor[0] = _kTor[1] = _kTor[2] = 100000.0;
	_bTor.setSize(3);
	_bTor[0] = _bTor[1] = _bTor[2] = 1000.0;
	_outputDetailedResults = false;

	// INTERNAL WORK VARIABLES
	_integrand = NULL;
	_yStore = NULL;	
	_rLin = NULL;
	_lLin = NULL;
	_rTor = NULL;
	_lTor = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ForwardTool::setupProperties()
{
	string comment;

	// BASIC
	comment = "XML file containing the controls (e.g., muscle excitations) for the forward simulation.";
	_controlsFileNameProp.setComment(comment);
	_controlsFileNameProp.setName("controls_file");
	_propertySet.append( &_controlsFileNameProp );

	comment = "Storage file (.sto) containing the initial states for the forward simulation. "
				 "This file often contains multiple rows of data, each row being a time-stamped array of states. "
				 "The first column contains the time.  The rest of the columns contain the states in the order "
				 "appropriate for the model. In a storage file, unlike a motion file (.mot), non-uniform time spacing "
				 "is allowed.  If the user-specified initial time for a simulation does not correspond exactly to "
				 "one of the time stamps in this file, inerpolation is NOT used because it is usually necessary to "
				 "being a simulation from an exact set of states.  Instead, the closest earlier set of states is used. "
				 "Having a states file that contains the entire trajectory of a simulations allows for corrective "
				 "springs for perturbation analysis to be added.";
	_statesFileNameProp.setComment(comment);
	_statesFileNameProp.setName("states_file");
	_propertySet.append( &_statesFileNameProp );

	comment = "Flag (true or false) indicating whether or not the integrator should "
				 "use a particular time stepping.  If true, the time stepping is extracted "
				 "from the initial states file.  In this situation, therefore, the initial "
				 "states file must contain all the time steps in a simulation and be written out "
				 "to high precision (usually 20 decimal places).  Setting this flag to true can "
				 "be useful when reproducing a previous forward simulation with as little drift "
				 "as possible.  If this flag is false, the integrator is left to determine its own "
				 "time stepping.";
	_useSpecifiedDtProp.setComment(comment);
	_useSpecifiedDtProp.setName("use_specified_dt");
	_propertySet.append( &_useSpecifiedDtProp );

	// EXTERNAL LOADS
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


	// FOOT CONTACT EVENT TIMES
	_rLinSpringOnProp.setComment("True-false flag indicating whether or not to turn on a linear corrective spring for external load body 1.");
	_rLinSpringOnProp.setName("body1_linear_corrective_spring_on");
	_propertySet.append(&_rLinSpringOnProp);

	_rTorSpringOnProp.setComment("True-false flag indicating whether or not to turn on a torsional corrective spring for external load body 1.");
	_rTorSpringOnProp.setName("body1_torsional_corrective_spring_on");
	_propertySet.append(&_rTorSpringOnProp);

	_lLinSpringOnProp.setComment("True-false flag indicating whether or not to turn on a linear corrective spring for external load body 2.");
	_lLinSpringOnProp.setName("body2_linear_corrective_spring_on");
	_propertySet.append(&_lLinSpringOnProp);

	_lTorSpringOnProp.setComment("True-false flag indicating whether or not to turn on a torsional corrective spring for external load body 2.");
	_lTorSpringOnProp.setName("body2_torsional_corrective_spring_on");
	_propertySet.append(&_lTorSpringOnProp);

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

	comment = "Force magnitude at which linear springs start to transition in.";
	_springTransitionStartForceProp.setComment(comment);
	_springTransitionStartForceProp.setName("spring_transition_start_force");
	_propertySet.append( &_springTransitionStartForceProp );

	comment = "Force magnitude past which linear springs are fully activated.";
	_springTransitionEndForceProp.setComment(comment);
	_springTransitionEndForceProp.setName("spring_transition_end_force");
	_propertySet.append( &_springTransitionEndForceProp );

	comment ="Force magnitude below which the linear corrective springs exert no force. "
				"Setting this parameter to a small positive number will make it possible to "
				"open-loop simulation for a longer period of time with less drift.";
	_forceThresholdProp.setComment(comment);
	_forceThresholdProp.setName("spring_force_threshold");
	_propertySet.append( &_forceThresholdProp );

	comment ="Torque magnitude below which the torsional corrective springs exert no force. "
				"Setting this parameter to a small positive number will make it possible to "
				"open-loop simulation for a longer period of time with less drift.";
	_torqueThresholdProp.setComment(comment);
	_torqueThresholdProp.setName("spring_torque_threshold");
	_propertySet.append( &_torqueThresholdProp );

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

	comment = "Record and output corrective spring forces, amoung other quantities.";
	_outputDetailedResultsProp.setComment(comment);
	_outputDetailedResultsProp.setName("output_detailed_results");
	_propertySet.append( &_outputDetailedResultsProp );
}

//_____________________________________________________________________________
/**
 * Construct the corrective springs.
void ForwardTool::
constructCorrectiveSprings(ForceApplier *aRightGRFApp, ForceApplier *aLeftGRFApp)
{

	AbstractBody *rightFootBody = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody1);
	if(!rightFootBody) throw Exception("PerturbationTool: ERR- Could not find right foot body '"+_externalLoadsBody1+"'",__FILE__,__LINE__);
	AbstractBody *leftFootBody = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody2);
	if(!leftFootBody) throw Exception("PerturbationTool: ERR- Could not find left foot body '"+_externalLoadsBody2+"'",__FILE__,__LINE__);

	// Extract the Qs and Us from the states.
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	Storage *qStore,*uStore;
	//ExtractConfiguration(*_yStore,nq,nu,qStore,uStore);

	// SCALING FUNCTIONS FOR SPRINGS
	double dtScale=0.001;
	double tiScale = qStore->getFirstTime();
	double tfScale = qStore->getLastTime();
	Array<double> timeScale(0.0);
	Array<double> rLinearScale(0.0),rTorsionalScale(0.0);
	Array<double> lLinearScale(0.0),lTorsionalScale(0.0);

	double tauRStart = _tauRightStartProp.getUseDefault() ? _tau : _tauRightStart;
	double tauREnd = _tauRightEndProp.getUseDefault() ? _tau : _tauRightEnd;
	double tauLStart = _tauLeftStartProp.getUseDefault() ? _tau : _tauLeftStart;
	double tauLEnd = _tauLeftEndProp.getUseDefault() ? _tau : _tauLeftEnd;

	cout << "Spring parameters:" << endl;
	cout << "\tSpring transition weights: " << _springTransitionStartForce << " " << _springTransitionEndForce << endl;
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
		rLinearScale.append(rdMath::Step(r_grf_y_norm,_springTransitionStartForce,_springTransitionEndForce));
		lLinearScale.append(rdMath::Step(l_grf_y_norm,_springTransitionStartForce,_springTransitionEndForce));

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
	//if(_externalLoadsBody1.size() && _externalLoadsBody1[_externalLoadsBody1.size()-1]=='l' || 
	//	_externalLoadsBody2.size() && _externalLoadsBody2[_externalLoadsBody2.size()-1]=='r')
	//	throw Exception("PerturbationTool: ERR- External loads body 1 should be right foot and external loads body 2 should be left foot",__FILE__,__LINE__);

	// LINEAR
	// right
	if(_rLinSpringOn) {
		_rLin = new LinearSpring(_model,rightFootBody);
		// Use the same foot-frame COP positions as the GRF force applier
		_rLin->setPointFunction((VectorFunction*)aRightGRFApp->getPointFunction()->copy());
		_rLin->computeTargetFunctions(qStore,uStore);
		_rLin->setKValue(&_kLin[0]);
		_rLin->setBValue(&_bLin[0]);
		_rLin->setScaleFunction(rScaleTranslationalSpline);
		if(_outputDetailedResults) _rLin->setRecordAppliedLoads(true);
		_model->addDerivCallback(_rLin);
	}
	// left linear
	if(_lLinSpringOn) {
		_lLin = new LinearSpring(_model,leftFootBody);
		// Use the same foot-frame COP positions as the GRF force applier
		_lLin->setPointFunction((VectorFunction*)aLeftGRFApp->getPointFunction()->copy());
		_lLin->computeTargetFunctions(qStore,uStore);
		_lLin->setKValue(&_kLin[0]);
		_lLin->setBValue(&_bLin[0]);
		_lLin->setScaleFunction(lScaleTranslationalSpline);
		if(_outputDetailedResults) _lLin->setRecordAppliedLoads(true);
		_model->addDerivCallback(_lLin);
	}

	// TORSIONAL
	// right
	if(_rTorSpringOn) {
		_rTor = new TorsionalSpring(_model,rightFootBody);
		_rTor->computeTargetFunctions(qStore,uStore);
		_rTor->setKValue(&_kTor[0]);
		_rTor->setBValue(&_bTor[0]);
		_rTor->setScaleFunction(rScaleTorsionalSpline);
		if(_outputDetailedResults) _rTor->setRecordAppliedLoads(true);
		_model->addDerivCallback(_rTor);
	}
	// left
	if(_lTorSpringOn) {
		_lTor = new TorsionalSpring(_model,leftFootBody);
		_lTor->computeTargetFunctions(qStore,uStore);
		_lTor->setKValue(&_kTor[0]);
		_lTor->setBValue(&_bTor[0]);
		_lTor->setScaleFunction(lScaleTorsionalSpline);
		if(_outputDetailedResults) _lTor->setRecordAppliedLoads(true);
		_model->addDerivCallback(_lTor);
	}

	Array<string> labels;
	labels.append("time"); 
	labels.append("r_scale_linear"); 
	labels.append("l_scale_linear");
	labels.append("r_scale_torsional"); 
	labels.append("l_scale_torsional");
	labels.append("r_stance");
	labels.append("l_stance");
	labels.append("r_footflat");
	labels.append("l_footflat");
	Storage debugStorage;
	debugStorage.setColumnLabels(labels);
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
*/

//_____________________________________________________________________________
/**
 * Add a linear corrective spring.
 */
LinearSpring* ForwardTool::
addLinearCorrectiveSpring(const Storage &aQStore,const Storage &aUStore,const ForceApplier &aAppliedForce)
{
	double dtScale=0.001;
	double tiScale = aQStore.getFirstTime();
	double tfScale = aQStore.getLastTime();
	Array<double> timeScale(0.0);
	Array<double> linearScale(0.0);

	cout<<"Linear corrective spring parameters:"<<endl;
	cout<<"\tSpring transition forces: "<<_springTransitionStartForce<<" "<<_springTransitionEndForce<<endl;

	// Create scale function
	for(double tScale=tiScale;tScale<=tfScale;tScale+=dtScale) {
		timeScale.append(tScale);
		double force[3];
		aAppliedForce.getForceFunction()->evaluate(&tScale,force);
		linearScale.append(rdMath::Step(Mtx::Magnitude(3,force),_springTransitionStartForce,_springTransitionEndForce));
	}
	GCVSpline *scaleSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&linearScale[0]);
	scaleSpline->setName("ScaleForLinearCorrectiveSpring");

	// Create linear spring
	LinearSpring *spring = new LinearSpring(_model,aAppliedForce.getBody());
	spring->setPointFunction((VectorFunction*)aAppliedForce.getPointFunction()->copy());
	spring->computeTargetFunctions(aQStore,aUStore);
	spring->setKValue(&_kLin[0]);
	spring->setBValue(&_bLin[0]);
	spring->setThreshold(_forceThreshold);
	spring->setScaleFunction(scaleSpline);
	if(_outputDetailedResults) spring->setRecordAppliedLoads(true);
	_model->addDerivCallback(spring);

	return(spring);
}
//_____________________________________________________________________________
/**
 * Add a torsional corrective spring.
 */
TorsionalSpring* ForwardTool::
addTorsionalCorrectiveSpring(const Storage &aQStore,const Storage &aUStore,AbstractBody *aBody,
									  double aTauOn,double aTimeOn,double aTauOff,double aTimeOff)
{
	// SCALING FUNCTIONS FOR SPRINGS
	double dtScale=0.001;
	double tiScale = aQStore.getFirstTime();
	double tfScale = aQStore.getLastTime();
	Array<double> timeScale(0.0);
	Array<double> torsionalScale(0.0);

	cout<<"Torsional corrective spring parameters:" << endl;
	cout<<"\tTau values: on="<<aTauOn<<", off="<<aTauOff<<endl;

	// Create scaling functions
	for(double tScale=tiScale;tScale<=tfScale;tScale+=dtScale) {
		timeScale.append(tScale);
		double value1 = rdMath::SigmaUp(aTauOn,aTimeOn,tScale);
		double value2 = rdMath::SigmaDn(aTauOff,aTimeOff,tScale);
		torsionalScale.append(value1+value2-1.0);
	}
	GCVSpline *scaleSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&torsionalScale[0]);
	scaleSpline->setName("ScaleForTorsionalCorrectiveSpring");

	TorsionalSpring *spring = new TorsionalSpring(_model,aBody);
	spring->computeTargetFunctions(aQStore,aUStore);
	spring->setKValue(&_kTor[0]);
	spring->setBValue(&_bTor[0]);
	spring->setThreshold(_torqueThreshold);
	spring->setScaleFunction(scaleSpline);
	if(_outputDetailedResults) spring->setRecordAppliedLoads(true);
	_model->addDerivCallback(spring);

	return(spring);
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

	// MEMEBER VARIABLES
	// BASIC INPUT
	_controlsFileName = aTool._controlsFileName;
	_statesFileName = aTool._statesFileName;
	_useSpecifiedDt = aTool._useSpecifiedDt;

	// EXTERNAL LOADS
	_externalLoadsFileName = aTool._externalLoadsFileName;
	_externalLoadsModelKinematicsFileName = aTool._externalLoadsModelKinematicsFileName;
	_externalLoadsBody1Prop = aTool._externalLoadsBody1Prop;
	_externalLoadsBody2Prop = aTool._externalLoadsBody2Prop;
	_lowpassCutoffFrequencyForLoadKinematics = aTool._lowpassCutoffFrequencyForLoadKinematics;
	_outputDetailedResults = aTool._outputDetailedResults;

	// FOOT CONTACT
	_rLinSpringOn = aTool._rLinSpringOn;
	_rTorSpringOn = aTool._rTorSpringOn;
	_lLinSpringOn = aTool._lLinSpringOn;
	_lTorSpringOn = aTool._lTorSpringOn;
	_rHeelStrike = aTool._rHeelStrike;
	_rFootFlat = aTool._rFootFlat;
	_rHeelOff = aTool._rHeelOff;
	_rToeOff = aTool._rToeOff;
	_lHeelStrike = aTool._lHeelStrike;
	_lFootFlat = aTool._lFootFlat;
	_lHeelOff = aTool._lHeelOff;
	_lToeOff = aTool._lToeOff;

	// CORRECTIVE SPRING PARAMETERS
	_tau = aTool._tau;
	_tauRightStart = aTool._tauRightStart;
	_tauRightEnd = aTool._tauRightEnd;
	_tauLeftStart = aTool._tauLeftStart;
	_tauLeftEnd = aTool._tauLeftEnd;
	_springTransitionStartForce = aTool._springTransitionStartForce;
	_springTransitionEndForce = aTool._springTransitionEndForce;
	_forceThreshold = aTool._forceThreshold;
	_torqueThreshold = aTool._torqueThreshold;
	_kLin = aTool._kLin;
	_bLin = aTool._bLin;
	_kTor = aTool._kTor;
	_bTor = aTool._bTor;

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
	cout<<"Running investigation "<<getName()<<"."<<endl;

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

	// INPUT
	// Controls
	ControlSet *controlSet=NULL;
	if(_controlsFileName!="") {
		cout<<"\n\nLoading controls from file "<<_controlsFileName<<"."<<endl;
		controlSet = new ControlSet(_controlsFileName);
		cout<<"Found "<<controlSet->getSize()<<" controls."<<endl<<endl;
	}
	// Initial states
	if(_statesFileName!="") {
		cout<<"\nLoading states from file "<<_statesFileName<<"."<<endl;
		_yStore = new Storage(_statesFileName);
		cout<<"Found "<<_yStore->getSize()<<" state vectors with time stamps ranging"<<endl;
		cout<<"from "<<_yStore->getFirstTime()<<" to "<<_yStore->getLastTime()<<"."<<endl;
	}

	// INITIAL AND FINAL TIMES
	// From initial states...
	int index=-1;
	double ti,tf;
	if(_yStore!=NULL) {
		index = _yStore->findIndex(_ti);
		if(index<0) {
			_ti = _yStore->getFirstTime();
			cout<<"\n\nWARN- The initial time set for the investigation precedes the first time\n";
			cout<<"in the initial states file.  Setting the investigation to run at the first time\n";
			cout<<"in the initial states file (ti = "<<_ti<<").\n\n";
		} else {
			_yStore->getTime(index,ti);
			if(_ti!=ti) {
				_ti = ti;
				cout<<"\n"<<getName()<<": The initial time for the investigation has been set to "<<_ti<<endl;
				cout<<"to agree exactly with the time stamp of the closest initial states in file ";
				cout<<_statesFileName<<".\n\n";
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
	int ny = _model->getNumStates();

	// GROUND REACTION FORCES
	ForceApplier *body1Force,*body2Force;
	initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics,&body1Force,&body2Force);

	// CORRECTIVE SPRINGS
	if(_yStore!=NULL) {
		Storage qStore,uStore;
		_model->getDynamicsEngine().extractConfiguration(*_yStore,qStore,uStore);
		AbstractBody *body1 = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody1);
		AbstractBody *body2 = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody2);
		// Body1 Linear
		if(_rLinSpringOn) {
			_rLin = addLinearCorrectiveSpring(qStore,uStore,*body1Force);
		}
		// Body1 Torsional
		if(_rTorSpringOn) {
			double tauOn = _tauRightStartProp.getUseDefault() ? _tau : _tauRightStart;
			double tauOff = _tauRightEndProp.getUseDefault() ? _tau : _tauRightEnd;
			_rTor = addTorsionalCorrectiveSpring(qStore,uStore,body1,tauOn,_rFootFlat,tauOff,_rHeelOff);
		}
		// Body2 Linear
		if(_lLinSpringOn) {
			_lLin = addLinearCorrectiveSpring(qStore,uStore,*body2Force);
		}
		// Body2 Torsional
		if(_lTorSpringOn) {
			double tauOn = _tauLeftStartProp.getUseDefault() ? _tau : _tauLeftStart;
			double tauOff = _tauLeftEndProp.getUseDefault() ? _tau : _tauLeftEnd;
			_lTor = addTorsionalCorrectiveSpring(qStore,uStore,body2,tauOn,_lFootFlat,tauOff,_lHeelOff);
		}
	}

	// SETUP SIMULATION
	// Manager
	delete _integrand;
	_integrand = new ModelIntegrand(_model);
	if(controlSet!=NULL) _integrand->setControlSet(*controlSet);
	Manager manager(_integrand);
	manager.setSessionName(getName());
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);

	// Integrator settings
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(_maxSteps);
	integ->setMaxDT(_maxDT);
	integ->setTolerance(_errorTolerance);
	integ->setFineTolerance(_fineTolerance);

	if(_useSpecifiedDt) {
		if(_yStore) {
			std::cout << "Using dt specified in initial states file (" << _statesFileName << ")" << std::endl;
			double *tArray = new double[_yStore->getSize()];
			double *dtArray = new double[_yStore->getSize()-1];
			_yStore->getTimeColumn(tArray);
			for(int i=0;i<_yStore->getSize()-1;i++) dtArray[i]=tArray[i+1]-tArray[i];
			integ->setUseSpecifiedDT(true);
			integ->setDTArray(_yStore->getSize()-1, dtArray, tArray[0]);
			delete[] tArray;
			delete[] dtArray;
		}
		else {
			std::cout << "WARNING: Ignoring 'use_specified_dt' property because no initial states file is specified" << std::endl;
		}
	}

	// SET INITIAL AND FINAL TIME AND THE INITIAL STATES
	Array<double> yi(0.0,ny);
	if(_yStore!=NULL) _yStore->getData(index,ny,&yi[0]);
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);
	if (index >= 0) {
		Array<double> yi(0.0,ny);
		_yStore->getData(index,ny,&yi[0]);
		_model->setInitialStates(&yi[0]);
	}

	// SOLVE FOR EQUILIBRIUM FOR AUXILIARY STATES (E.G., MUSCLE FIBER LENGTHS)
	if(_solveForEquilibriumForAuxiliaryStates) {
		cout<<"\n\n------ Before Equilibrium -------------\n";
		cout<<yi;
		cout<<"\n\n-----------------------------------\n";
		_model->computeEquilibriumForAuxiliaryStates(&yi[0]);
		cout<<"\n\n------ After Equlilibrium  --------\n";
		cout<<yi;
		cout<<"\n\n-----------------------------------\n";
		_model->setInitialStates(&yi[0]);
	}

	bool completed = true;

	try {
		// INTEGRATE
		cout<<"\n\nIntegrating from "<<_ti<<" to "<<_tf<<endl;
		manager.integrate();
	} catch (...) { // e.g. may get InterruptedException
		completed = false;
	}

	// PRINT RESULTS
	char fileName[Object::NAME_LENGTH];
	if(_printResultFiles) printResults();
	if(_outputDetailedResults) {
		// Spring forces
		if(_rLin) {
			sprintf(fileName,"%s/%s_detailed_appliedForce_rLin.sto",getResultsDir().c_str(),getName().c_str());
			_rLin->getAppliedForceStorage()->print(fileName);
		}
		if(_lLin) {
			sprintf(fileName,"%s/%s_detailed_appliedForce_lLin.sto",getResultsDir().c_str(),getName().c_str());
			_lLin->getAppliedForceStorage()->print(fileName);
		}
		if(_rTor) {
			sprintf(fileName,"%s/%s_detailed_appliedTorque_rTor.sto",getResultsDir().c_str(),getName().c_str());
			_rTor->getAppliedTorqueStorage()->print(fileName);
		}
		if(_lTor) {
			sprintf(fileName,"%s/%s_detailed_appliedTorque_lTor.sto",getResultsDir().c_str(),getName().c_str());
			_lTor->getAppliedTorqueStorage()->print(fileName);
		}
	}


	IO::chDir(saveWorkingDirectory);

	return completed;
}
//=============================================================================
// PRINT RESULTS
//=============================================================================
void ForwardTool::
printResults() 
{
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	AbstractTool::printResults(getName(),getResultsDir()); // this will create results directory if necessary
	if(_integrand) {
		_integrand->getControlStorage()->print(getResultsDir() + "/" + getName() + "_controls.sto");
		_integrand->getStateStorage()->print(getResultsDir() + "/" + getName() + "_states.sto");
		_integrand->getPseudoStateStorage()->print(getResultsDir() + "/" + getName() + "_pseudo.sto");

		Storage statesDegrees(*_integrand->getStateStorage());
		_model->getDynamicsEngine().convertRadiansToDegrees(statesDegrees);
		statesDegrees.setWriteSIMMHeader(true);
		statesDegrees.print(getResultsDir() + "/" + getName() + "_states_degrees.mot");
	}

	IO::chDir(saveWorkingDirectory);
}

Storage *ForwardTool::
getStateStorage() 
{
	return _integrand ? _integrand->getStateStorage() : 0;
}
//=============================================================================
// EXTERNAL LOADS
//=============================================================================
//_____________________________________________________________________________
/**
 * Initialize the external loads applied to the model.  Currently, the loads
 * are assumed to be applied to the right and left foot.
 */
void ForwardTool::
initializeExternalLoads(Model *aModel, const string &aExternalLoadsFileName,
								const string &aExternalLoadsModelKinematicsFileName,
								const string &aExternalLoadsBody1,
								const string &aExternalLoadsBody2,
								double aLowpassCutoffFrequencyForLoadKinematics,
								ForceApplier **rRightForceApp,
								ForceApplier **rLeftForceApp,
								TorqueApplier **rRightTorqueApp,
								TorqueApplier **rLeftTorqueApp)
{
	if(aExternalLoadsFileName=="") {
		cout<<"\n\nWARNING- a file name for external loads was not specified.";
		cout<<" No loads will be applied.\n\n";
		return;
	}

	// LOAD MODEL KINEMATICS FOR EXTERNAL LOADS
	// To get the forces to be applied in the correct location, this file
	// should be from the IK solution, not from pass 2 of rra which alters
	// the kinematics.
	if(aExternalLoadsModelKinematicsFileName=="") {
		cout<<"\n\nERROR- a external loads kinematics file was not specified.\n\n";
		return;
	}
	cout<<"\n\nLoading external loads kinematics from file "<<aExternalLoadsModelKinematicsFileName<<" ...\n";
	Storage loadsKinStore(aExternalLoadsModelKinematicsFileName);
	// Form complete storage objects for the q's and u's
	// This means filling in unspecified generalized coordinates and
	// setting constrained coordinates to their valid values.
	Storage *qStore=NULL;
	Storage *uStoreTmp=NULL;
	aModel->getDynamicsEngine().formCompleteStorages(loadsKinStore,qStore,uStoreTmp);
	aModel->getDynamicsEngine().convertDegreesToRadians(*qStore);
	// Filter
	qStore->pad(60); 
	if(aLowpassCutoffFrequencyForLoadKinematics>=0) {
		int order = 50;
		cout<<"\n\nLow-pass filtering external load kinematics with a cutoff frequency of ";
		cout<<aLowpassCutoffFrequencyForLoadKinematics<<"...\n\n";
		qStore->lowpassFIR(order,aLowpassCutoffFrequencyForLoadKinematics);
	} else {
		cout<<"\n\nNote- not filtering the external loads model kinematics.\n\n";
	}
	// Spline
	GCVSplineSet qSet(5,qStore);
	Storage *uStore = qSet.constructStorage(1);

	// LOAD COP, FORCE, AND TORQUE
	Storage kineticsStore(aExternalLoadsFileName);
	int copSize = kineticsStore.getSize();
	if(copSize<=0) return;

	// Read the indices of all the ground reaction data columns.
	// We assume that the right foot's data appears before the left foot's data
	// when reading the kinetics file's columns from left to right.
	int rightForceX  = kineticsStore.getStateIndex("ground_force_vx");
	if(rightForceX<0) {
		string msg = "ForwardTool.run: ERR- Column index for right ground_force_vx not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightForceY  = kineticsStore.getStateIndex("ground_force_vy");
	if(rightForceY<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_force_vy not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightForceZ  = kineticsStore.getStateIndex("ground_force_vz");
	if(rightForceZ<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_force_vz not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftForceX   = kineticsStore.getStateIndex("ground_force_vx", rightForceX + 2);
	if(leftForceX<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_force_vx not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftForceY   = kineticsStore.getStateIndex("ground_force_vy", rightForceY + 2);
	if(leftForceY<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_force_vy not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftForceZ   = kineticsStore.getStateIndex("ground_force_vz", rightForceZ + 2);
	if(leftForceZ<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_force_vz not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightCopX    = kineticsStore.getStateIndex("ground_force_px");
	if(rightCopX<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_force_px not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightCopY    = kineticsStore.getStateIndex("ground_force_py");
	if(rightCopY<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_force_py not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightCopZ    = kineticsStore.getStateIndex("ground_force_pz");
	if(rightCopZ<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_force_pz not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftCopX     = kineticsStore.getStateIndex("ground_force_px", rightCopX + 2);
	if(leftCopX<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_force_px not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftCopY     = kineticsStore.getStateIndex("ground_force_py", rightCopY + 2);
	if(leftCopY<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_force_py not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftCopZ     = kineticsStore.getStateIndex("ground_force_pz", rightCopZ + 2);
	if(leftCopZ<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_force_pz not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightTorqueX = kineticsStore.getStateIndex("ground_torque_x");
	if(rightTorqueX<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_torque_x not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightTorqueY = kineticsStore.getStateIndex("ground_torque_y");
	if(rightTorqueY<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_torque_y not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int rightTorqueZ = kineticsStore.getStateIndex("ground_torque_z");
	if(rightTorqueZ<0) {
		string msg = "FowardTool.run: ERR- Column index for right ground_torque_z not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftTorqueX  = kineticsStore.getStateIndex("ground_torque_x", rightTorqueX + 2);
	if(leftTorqueX<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_torque_x not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftTorqueY  = kineticsStore.getStateIndex("ground_torque_y", rightTorqueY + 2);
	if(leftTorqueY<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_torque_y not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}
	int leftTorqueZ  = kineticsStore.getStateIndex("ground_torque_z", rightTorqueZ + 2);
	if(leftTorqueZ<0) {
		string msg = "FowardTool.run: ERR- Column index for left ground_torque_z not found in ";
		msg += aExternalLoadsFileName + ".";
		throw Exception(msg,__FILE__,__LINE__);
	}

	// BODY INDICES
	// Right
	AbstractBody *body1 = aModel->getDynamicsEngine().getBodySet()->get(aExternalLoadsBody1);
	if(body1<0) {
		string msg = "FowardTool.run: ERR- The body to which the first set of external loads";
		msg+="should be applied (" + aExternalLoadsBody1 + ") is not a segment in the model.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	// Left
	//int  leftFoot = aModel->getBodyIndex("calcn_l");
	AbstractBody *body2 = aModel->getDynamicsEngine().getBodySet()->get(aExternalLoadsBody2);
	if(body2<0) {
		string msg = "FowardTool.run: ERR- The body to which the second set of external loads";
		msg+="should be applied (" + aExternalLoadsBody2 + ") is not a segment in the model.";
		throw Exception(msg,__FILE__,__LINE__);
	}
	// Ground
	AbstractBody &ground = aModel->getDynamicsEngine().getGroundBody();


	// CREATE FORCE AND TORQUE APPLIERS
	ForceApplier *rightForceApp, *leftForceApp;
	TorqueApplier *rightTorqueApp, *leftTorqueApp;
	rightForceApp = new ForceApplier(aModel, &ground, body1, &kineticsStore,
												rightForceX, rightForceY, rightForceZ,
												rightCopX, rightCopY, rightCopZ,
												qStore, uStore);
	leftForceApp  = new ForceApplier(aModel, &ground, body2, &kineticsStore,
												leftForceX, leftForceY, leftForceZ,
												leftCopX, leftCopY, leftCopZ,
												qStore, uStore);
	rightTorqueApp = new TorqueApplier(aModel, &ground, body1, &kineticsStore,
												rightTorqueX, rightTorqueY, rightTorqueZ);
	leftTorqueApp  = new TorqueApplier(aModel, &ground, body2, &kineticsStore,
												leftTorqueX, leftTorqueY, leftTorqueZ);

	// Add force and torque appliers as derivative callbacks for model.
	// Set input in global frame is true by default--we're just being
	// paranoid here by setting it to true for sure.
	rightForceApp->setInputForcesInGlobalFrame(true);
	leftForceApp->setInputForcesInGlobalFrame(true);
	rightTorqueApp->setInputTorquesInGlobalFrame(true);
	leftTorqueApp->setInputTorquesInGlobalFrame(true);
	aModel->addDerivCallback(rightForceApp);
	aModel->addDerivCallback(leftForceApp);
	aModel->addDerivCallback(rightTorqueApp);
	aModel->addDerivCallback(leftTorqueApp);

	if(rRightForceApp) *rRightForceApp=rightForceApp;
	if(rLeftForceApp) *rLeftForceApp=leftForceApp;
	if(rRightTorqueApp) *rRightTorqueApp=rightTorqueApp;
	if(rLeftTorqueApp) *rLeftTorqueApp=leftTorqueApp;
}
