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
	if(_yStore!=NULL) delete _yStore;
	if(_ypStore!=NULL) delete _ypStore;
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
	_pseudoFileName(_pseudoFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_body1LinSpringActive(_body1LinSpringActiveProp.getValueBool()),
	_body1TorSpringActive(_body1TorSpringActiveProp.getValueBool()),
	_body2LinSpringActive(_body2LinSpringActiveProp.getValueBool()),
	_body2TorSpringActive(_body2TorSpringActiveProp.getValueBool()),
	_body1TorSpringTimeOn(_body1TorSpringTimeOnProp.getValueDbl()),
	_body1TorSpringTimeOff(_body1TorSpringTimeOffProp.getValueDbl()),
	_body2TorSpringTimeOn(_body2TorSpringTimeOnProp.getValueDbl()),
	_body2TorSpringTimeOff(_body2TorSpringTimeOffProp.getValueDbl()),
	_tau(_tauProp.getValueDbl()),
	_tauBody1On(_tauBody1OnProp.getValueDbl()),
	_tauBody1Off(_tauBody1OffProp.getValueDbl()),
	_tauBody2On(_tauBody2OnProp.getValueDbl()),
	_tauBody2Off(_tauBody2OffProp.getValueDbl()),
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
ForwardTool::ForwardTool(const string &aFileName,bool aUpdateFromXMLNode,bool aLoadModel) :
	AbstractTool(aFileName, false),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_pseudoFileName(_pseudoFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_body1LinSpringActive(_body1LinSpringActiveProp.getValueBool()),
	_body1TorSpringActive(_body1TorSpringActiveProp.getValueBool()),
	_body2LinSpringActive(_body2LinSpringActiveProp.getValueBool()),
	_body2TorSpringActive(_body2TorSpringActiveProp.getValueBool()),
	_body1TorSpringTimeOn(_body1TorSpringTimeOnProp.getValueDbl()),
	_body1TorSpringTimeOff(_body1TorSpringTimeOffProp.getValueDbl()),
	_body2TorSpringTimeOn(_body2TorSpringTimeOnProp.getValueDbl()),
	_body2TorSpringTimeOff(_body2TorSpringTimeOffProp.getValueDbl()),
	_tau(_tauProp.getValueDbl()),
	_tauBody1On(_tauBody1OnProp.getValueDbl()),
	_tauBody1Off(_tauBody1OffProp.getValueDbl()),
	_tauBody2On(_tauBody2OnProp.getValueDbl()),
	_tauBody2Off(_tauBody2OffProp.getValueDbl()),
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
	if(aUpdateFromXMLNode) updateFromXMLNode();
	if(aLoadModel) { loadModel(aFileName); setToolOwnsModel(true); }
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
	_pseudoFileName(_pseudoFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_body1LinSpringActive(_body1LinSpringActiveProp.getValueBool()),
	_body1TorSpringActive(_body1TorSpringActiveProp.getValueBool()),
	_body2LinSpringActive(_body2LinSpringActiveProp.getValueBool()),
	_body2TorSpringActive(_body2TorSpringActiveProp.getValueBool()),
	_body1TorSpringTimeOn(_body1TorSpringTimeOnProp.getValueDbl()),
	_body1TorSpringTimeOff(_body1TorSpringTimeOffProp.getValueDbl()),
	_body2TorSpringTimeOn(_body2TorSpringTimeOnProp.getValueDbl()),
	_body2TorSpringTimeOff(_body2TorSpringTimeOffProp.getValueDbl()),
	_tau(_tauProp.getValueDbl()),
	_tauBody1On(_tauBody1OnProp.getValueDbl()),
	_tauBody1Off(_tauBody1OffProp.getValueDbl()),
	_tauBody2On(_tauBody2OnProp.getValueDbl()),
	_tauBody2Off(_tauBody2OffProp.getValueDbl()),
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
	_pseudoFileName = "";
	_useSpecifiedDt = false;
	_printResultFiles = true;

	// EXTERNAL LOADS
	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = "";
	_externalLoadsBody2 = "";
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;

	// CORRECTIVE SPING ON FLAGS
	_body1LinSpringActive = _body1TorSpringActive = false;
	_body2LinSpringActive = _body2TorSpringActive = false;

	// FOOT CONTACT EVENTS
	_body1TorSpringTimeOn = _body1TorSpringTimeOff = 0.0;
	_body2TorSpringTimeOn = _body2TorSpringTimeOff = 0.0;

	// CORRECTIVE SPRING PARAMETERS
	_tau = 0.001;
	_tauBody1On = _tauBody1Off = _tauBody2On = _tauBody2Off = _tau;
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
	_ypStore = NULL;
	_body1Lin = NULL;
	_body2Lin = NULL;
	_body1Tor = NULL;
	_body2Tor = NULL;
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

	comment = "Storage file (.sto) containing the initial pseudo states for the forward simulation. "
				 "Pseudostates are quantities that are not integrated, but never-the-less are dependent "
				 "on the time history of simulation.  Examples, are the spring zeros for contact elements. "
				 "This file often contains multiple rows of data, each row being a time-stamped array of pseudostates. "
				 "The first column contains the time.  The rest of the columns contain the states in the order "
				 "appropriate for the model. In a storage file, unlike a motion file (.mot), non-uniform time spacing "
				 "is allowed.  If the user-specified initial time for a simulation does not correspond exactly to "
				 "one of the time stamps in this file, inerpolation is NOT used because it is usually necessary to "
				 "being a simulation from an exact set of states.  Instead, the closest earlier set of pseudostates is used. "
				 "Having a pseudostates file that contains the entire trajectory of a simulations allows for corrective "
				 "springs for perturbation analysis to be added. The time stamps in a pseudostates file should "
				 "match the time stamps in its companion states file.";
	_pseudoFileNameProp.setComment(comment);
	_pseudoFileNameProp.setName("pseudo_states_file");
	_propertySet.append( &_pseudoFileNameProp );

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


	// CONTACT ON-OFF PROPERTIES
	// Body1 Linear
	_body1LinSpringActiveProp.setComment("True-false flag indicating whether or not to turn on a linear corrective spring for external load body 1.");
	_body1LinSpringActiveProp.setName("body1_linear_corrective_spring_active");
	_propertySet.append(&_body1LinSpringActiveProp);

	// Body1 Torsional
	_body1TorSpringActiveProp.setComment("True-false flag indicating whether or not to turn on a torsional corrective spring for external load body 1.");
	_body1TorSpringActiveProp.setName("body1_torsional_corrective_spring_active");
	_propertySet.append(&_body1TorSpringActiveProp);

	_body1TorSpringTimeOnProp.setComment("Time at which the torsional spring comes on for body1 (if it is active). By default, this time is 0.0.");
	_body1TorSpringTimeOnProp.setName("body1_torsional_corrective_spring_time_on");
	_propertySet.append( &_body1TorSpringTimeOnProp );

	_body1TorSpringTimeOffProp.setComment("Time at which the torsional spring turns off for body1 (if it is active). By default, this time is 0.0.");
	_body1TorSpringTimeOffProp.setName("body1_torsional_corrective_spring_time_off");
	_propertySet.append( &_body1TorSpringTimeOffProp );

	// Body2 Linear
	_body2LinSpringActiveProp.setComment("True-false flag indicating whether or not to turn on a linear corrective spring for external load body 2.");
	_body2LinSpringActiveProp.setName("body2_linear_corrective_spring_active");
	_propertySet.append(&_body2LinSpringActiveProp);

	// Body2 Torsional
	_body2TorSpringActiveProp.setComment("True-false flag indicating whether or not to turn on a torsional corrective spring for external load body 2.");
	_body2TorSpringActiveProp.setName("body2_torsional_corrective_spring_active");
	_propertySet.append(&_body2TorSpringActiveProp);

	_body2TorSpringTimeOnProp.setComment("Time at which the torsional spring comes on for body2 (if it is active). By default, this time is 0.0.");
	_body2TorSpringTimeOnProp.setName("body2_torsional_corrective_spring_time_on");
	_propertySet.append( &_body2TorSpringTimeOnProp );

	_body2TorSpringTimeOffProp.setComment("Time at which the torsional spring turns off for body2 (if it is active). By default, this time is 0.0.");
	_body2TorSpringTimeOffProp.setName("body2_torsional_corrective_spring_time_off");
	_propertySet.append( &_body2TorSpringTimeOffProp );


	// CORRECTIVE SPRING PARAMETERS
	comment = "Force magnitude at which linear springs start to transition in.";
	_springTransitionStartForceProp.setComment(comment);
	_springTransitionStartForceProp.setName("linear_spring_transition_start_force");
	_propertySet.append( &_springTransitionStartForceProp );

	comment = "Force magnitude past which linear springs are fully activated.";
	_springTransitionEndForceProp.setComment(comment);
	_springTransitionEndForceProp.setName("spring_transition_end_force");
	_propertySet.append( &_springTransitionEndForceProp );

	comment = "Rise time for scaling functions for the torsional corrective springs. "
				 "This parameter determines how fast a torsional corrective spring is scaled on and off.";
	_tauProp.setComment(comment);
	_tauProp.setName("torsional_spring_scaling_rise_time");
	_propertySet.append( &_tauProp );

	comment = "Override scaling rise time for the on transition of the body1 torsional corrective spring.";
	_tauBody1OnProp.setComment(comment);
	_tauBody1OnProp.setName("body1_scaling_rise_time_on");
	_propertySet.append( &_tauBody1OnProp );

	comment = "Override scaling rise time for the off transition out of the body1 torsional corrective spring.";
	_tauBody1OffProp.setComment(comment);
	_tauBody1OffProp.setName("body1_scaling_rise_time_off");
	_propertySet.append( &_tauBody1OffProp );

	comment = "Override scaling rise time for the on transition of the body2 torsional corrective spring.";
	_tauBody2OnProp.setComment(comment);
	_tauBody2OnProp.setName("body2_scaling_rise_time_on");
	_propertySet.append( &_tauBody2OnProp );

	comment = "Override scaling rise time for the off transition of the body2 torsional corrective spring.";
	_tauBody2OffProp.setComment(comment);
	_tauBody2OffProp.setName("body2_scaling_rise_time_off");
	_propertySet.append( &_tauBody2OffProp );

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

	double tauRStart = _tauBody1OnProp.getUseDefault() ? _tau : _tauBody1On;
	double tauREnd = _tauBody1OffProp.getUseDefault() ? _tau : _tauBody1Off;
	double tauLStart = _tauBody2OnProp.getUseDefault() ? _tau : _tauBody2On;
	double tauLEnd = _tauBody2OffProp.getUseDefault() ? _tau : _tauBody2Off;

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
		value2 = rdMath::SigmaDn(tauREnd,_body1TorSpringTimeOff,tScale);
		rTorsionalScale.append(value1+value2-1.0);
		// lTorsional
		value1 = rdMath::SigmaUp(tauLStart,_body2TorSpringTimeOn,tScale);
		value2 = rdMath::SigmaDn(tauLEnd,_body2TorSpringTimeOff,tScale);
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
	if(_body1LinSpringActive) {
		_body1Lin = new LinearSpring(_model,rightFootBody);
		// Use the same foot-frame COP positions as the GRF force applier
		_body1Lin->setPointFunction((VectorFunction*)aRightGRFApp->getPointFunction()->copy());
		_body1Lin->computeTargetFunctions(qStore,uStore);
		_body1Lin->setKValue(&_kLin[0]);
		_body1Lin->setBValue(&_bLin[0]);
		_body1Lin->setScaleFunction(rScaleTranslationalSpline);
		if(_outputDetailedResults) _body1Lin->setRecordAppliedLoads(true);
		_model->addDerivCallback(_body1Lin);
	}
	// left linear
	if(_body2LinSpringActive) {
		_body2Lin = new LinearSpring(_model,leftFootBody);
		// Use the same foot-frame COP positions as the GRF force applier
		_body2Lin->setPointFunction((VectorFunction*)aLeftGRFApp->getPointFunction()->copy());
		_body2Lin->computeTargetFunctions(qStore,uStore);
		_body2Lin->setKValue(&_kLin[0]);
		_body2Lin->setBValue(&_bLin[0]);
		_body2Lin->setScaleFunction(lScaleTranslationalSpline);
		if(_outputDetailedResults) _body2Lin->setRecordAppliedLoads(true);
		_model->addDerivCallback(_body2Lin);
	}

	// TORSIONAL
	// right
	if(_body1TorSpringActive) {
		_body1Tor = new TorsionalSpring(_model,rightFootBody);
		_body1Tor->computeTargetFunctions(qStore,uStore);
		_body1Tor->setKValue(&_kTor[0]);
		_body1Tor->setBValue(&_bTor[0]);
		_body1Tor->setScaleFunction(rScaleTorsionalSpline);
		if(_outputDetailedResults) _body1Tor->setRecordAppliedLoads(true);
		_model->addDerivCallback(_body1Tor);
	}
	// left
	if(_body2TorSpringActiveProp) {
		_body2Tor = new TorsionalSpring(_model,leftFootBody);
		_body2Tor->computeTargetFunctions(qStore,uStore);
		_body2Tor->setKValue(&_kTor[0]);
		_body2Tor->setBValue(&_bTor[0]);
		_body2Tor->setScaleFunction(lScaleTorsionalSpline);
		if(_outputDetailedResults) _body2Tor->setRecordAppliedLoads(true);
		_model->addDerivCallback(_body2Tor);
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
								  (_rFootFlat<=tScale && tScale<=_body1TorSpringTimeOff)?1:0,
								  (_body2TorSpringTimeOn<=tScale && tScale<=_body2TorSpringTimeOff)?1:0};
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
	_pseudoFileName = aTool._pseudoFileName;
	_useSpecifiedDt = aTool._useSpecifiedDt;

	// EXTERNAL LOADS
	_externalLoadsFileName = aTool._externalLoadsFileName;
	_externalLoadsModelKinematicsFileName = aTool._externalLoadsModelKinematicsFileName;
	_externalLoadsBody1Prop = aTool._externalLoadsBody1Prop;
	_externalLoadsBody2Prop = aTool._externalLoadsBody2Prop;
	_lowpassCutoffFrequencyForLoadKinematics = aTool._lowpassCutoffFrequencyForLoadKinematics;
	_outputDetailedResults = aTool._outputDetailedResults;

	// FOOT CONTACT
	_body1LinSpringActive = aTool._body1LinSpringActive;
	_body1TorSpringActive = aTool._body1TorSpringActive;
	_body1TorSpringTimeOn = aTool._body1TorSpringTimeOn;
	_body1TorSpringTimeOff = aTool._body1TorSpringTimeOff;
	_body2LinSpringActive = aTool._body2LinSpringActive;
	_body2TorSpringActive = aTool._body2TorSpringActive;
	_body2TorSpringTimeOn = aTool._body2TorSpringTimeOn;
	_body2TorSpringTimeOff = aTool._body2TorSpringTimeOff;

	// CORRECTIVE SPRING PARAMETERS
	_tau = aTool._tau;
	_tauBody1On = aTool._tauBody1On;
	_tauBody1Off = aTool._tauBody1Off;
	_tauBody2On = aTool._tauBody2On;
	_tauBody2Off = aTool._tauBody2Off;
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
	cout<<"Running tool "<<getName()<<"."<<endl;

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
	ControlSet *controlSet = NULL;
	inputControlsStatesAndPseudoStates(controlSet,_yStore,_ypStore);

	// INITIAL AND FINAL TIMES AND STATES INDEX
	int startIndexForYStore = determineInitialTimeFromStatesStorage(_ti);

	// PSEUDO STATES INDEX
	bool interpolatePseudoStates;
	int startIndexForYPStore = determinePseudoStatesIndex(_ti,interpolatePseudoStates);

	// CHECK CONTROLS
	checkControls(controlSet);

	// ASSIGN NUMBERS OF THINGS
	int ny = _model->getNumStates();
	int nyp = _model->getNumPseudoStates();

	// GROUND REACTION FORCES
	ForceApplier *body1Force,*body2Force;
	initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics,&body1Force,&body2Force);

	// CORRECTIVE SPRINGS
	addCorrectiveSprings(body1Force,body2Force);

	// SETUP SIMULATION
	// Manager
	delete _integrand;
	_integrand = new ModelIntegrand(_model);
	if(controlSet!=NULL) {_integrand->setControlSet(*controlSet); delete controlSet; }
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

	// SET INITIAL AND FINAL TIME
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);

	// SET THE INITIAL STATES and PSEUDO STATES
	Array<double> yi(0.0,ny);
	if(_yStore!=NULL) _yStore->getData(startIndexForYStore,ny,&yi[0]);
	if(startIndexForYStore >= 0) {
		Array<double> yi(0.0,ny);
		_yStore->getData(startIndexForYStore,ny,&yi[0]);
		_model->setInitialStates(&yi[0]);
	}
	if(startIndexForYPStore >= 0) {
		Array<double> ypi(0.0,nyp);
		if(!interpolatePseudoStates) {
			_ypStore->getData(startIndexForYPStore,nyp,&ypi[0]);
		} else {
			_ypStore->getData(_ti,nyp,&ypi[0]);
		}
		_model->setInitialPseudoStates(&ypi[0]);
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
		if(_body1Lin) {
			sprintf(fileName,"%s/%s_detailed_appliedForce_body1.sto",getResultsDir().c_str(),getName().c_str());
			_body1Lin->getAppliedForceStorage()->print(fileName);
		}
		if(_body2Lin) {
			sprintf(fileName,"%s/%s_detailed_appliedForce_body2.sto",getResultsDir().c_str(),getName().c_str());
			_body2Lin->getAppliedForceStorage()->print(fileName);
		}
		if(_body1Tor) {
			sprintf(fileName,"%s/%s_detailed_appliedTorque_body1.sto",getResultsDir().c_str(),getName().c_str());
			_body1Tor->getAppliedTorqueStorage()->print(fileName);
		}
		if(_body2Tor) {
			sprintf(fileName,"%s/%s_detailed_appliedTorque_body2.sto",getResultsDir().c_str(),getName().c_str());
			_body2Tor->getAppliedTorqueStorage()->print(fileName);
		}
	}


	IO::chDir(saveWorkingDirectory);

	// Since the Tool added the DerivCallbacks to the model, it is responsible for removing them as well
	_model->removeAllDerivCallbacks();
	removeAnalysisSetFromModel();
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
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Add corrective springs.
 *
 * @return Index into the pseudo states storage for the initial pseudo
 * states for the simulation.  A return of -1 indicates no valid pseudo states.
 */
void ForwardTool::
addCorrectiveSprings(const ForceApplier *aBody1Force,const ForceApplier *aBody2Force)
{
	if(_yStore!=NULL && !( _externalLoadsBody1=="") && !( _externalLoadsBody2=="")) {
		Storage qStore,uStore;
		_model->getDynamicsEngine().extractConfiguration(*_yStore,qStore,uStore);
		AbstractBody *body1 = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody1);
		AbstractBody *body2 = _model->getDynamicsEngine().getBodySet()->get(_externalLoadsBody2);

		// Body1 Linear
		if(_body1LinSpringActive) {
			_body1Lin = addLinearCorrectiveSpring(qStore,uStore,*aBody1Force);
		}
		// Body1 Torsional
		if(_body1TorSpringActive) {
			double tauOn = _tauBody1OnProp.getUseDefault() ? _tau : _tauBody1On;
			double tauOff = _tauBody1OffProp.getUseDefault() ? _tau : _tauBody1Off;
			_body1Tor = addTorsionalCorrectiveSpring(qStore,uStore,body1,tauOn,_body1TorSpringTimeOn,tauOff,_body1TorSpringTimeOff);
		}

		// Body2 Linear
		if(_body2LinSpringActive) {
			_body2Lin = addLinearCorrectiveSpring(qStore,uStore,*aBody2Force);
		}
		// Body2 Torsional
		if(_body2TorSpringActive) {
			double tauOn = _tauBody2OnProp.getUseDefault() ? _tau : _tauBody2On;
			double tauOff = _tauBody2OffProp.getUseDefault() ? _tau : _tauBody2Off;
			_body2Tor = addTorsionalCorrectiveSpring(qStore,uStore,body2,tauOn,_body2TorSpringTimeOn,tauOff,_body2TorSpringTimeOff);
		}
	}
}
//_____________________________________________________________________________
/**
 * Check the controls.  This method just prints out warning messages.  It
 * doesn't change anything important.
 */
void ForwardTool::
checkControls(const ControlSet *aControlSet)
{
	if(aControlSet==NULL) {
		cout<<"\n"<<getName()<<": WARN- No controls were specified.\n";	
		return;
	}

	int first = 0;
	Control *control = aControlSet->get(first);
	double ti = control->getFirstTime();
	double tf = control->getLastTime();
	if(_ti<ti) {
		cout<<"\n"<<getName()<<": WARN- The controls read in from file "<<_controlsFileName<<" did not\n";
		cout<<"overlap the requested initial time of the simulation.  Controls are being extrapolated.\n";
	}
	if(_tf>tf) {
		cout<<"\n"<<getName()<<": WARN- The controls read in from file "<<_controlsFileName<<" did not\n";
		cout<<"overlap the requested final time of the simulation.  Changing the final time of the\n";
		cout<<"forward integration from "<<_tf<<" to "<<tf<<".\n";
		_tf = tf;
	}
}
//_____________________________________________________________________________
/**
 * Determine the index into the pseudo states storage for the initial
 * pseudo states.
 *
 * @return Index into the pseudo states storage for the initial pseudo
 * states for the simulation.  A return of -1 indicates no valid pseudo states.
 */
int ForwardTool::
determinePseudoStatesIndex(double aTI,bool &interpolatePseudoStates)
{
	int index = -1;
	double ti;
	if(_ypStore!=NULL) {
		index = _ypStore->findIndex(aTI);
		if(index<0) {
			cout<<"\n\nWARN- The pseudo states file does not contain a time range that overlaps\n";
			cout<<"the time range of the states file.  RUNNING without pseudo sates.\n\n";
		} else {
			_ypStore->getTime(index,ti);
			if(ti!=aTI) {
				cout<<"\n\nWARN- The time stamp in the pseudo states file does not match the time stamp\n";
				cout<<"in the states file.  Interpolating to get the pseudo states.\n\n";
				interpolatePseudoStates = true;
			}
		}
	}
	return index;
}
//_____________________________________________________________________________
/**
 * Determine initial time for a simulation and find the index into the
 * states storage for that time.
 *
 * @param rTI Requested initial time for the simulation.  If the time does not
 * match a time in the states storage exactly, the initial time is altered so
 * that there is an exact match.
 * @return Index into the states storage corresponding to the initial states
 * for the simulation.  A return of -1 indicates no valid states.
 */
int ForwardTool::
determineInitialTimeFromStatesStorage(double &rTI)
{
	int index = 1;
	double ti;
	if(_yStore!=NULL) {
		index = _yStore->findIndex(rTI);
		if(index<0) {
			rTI = _yStore->getFirstTime();
			cout<<"\n\nWARN- The initial time set for the investigation precedes the first time\n";
			cout<<"in the initial states file.  Setting the investigation to run at the first time\n";
			cout<<"in the initial states file (ti = "<<rTI<<").\n\n";
			index = 0;
		} else {
			_yStore->getTime(index,ti);
			if(rTI!=ti) {
				rTI = ti;
				cout<<"\n"<<getName()<<": The initial time for the investigation has been set to "<<rTI<<endl;
				cout<<"to agree exactly with the time stamp of the closest initial states in file ";
				cout<<_statesFileName<<".\n\n";
			}
		}
	}

	return(index);
}

//_____________________________________________________________________________
/**
 * Read in the controls, states, and pseudo states.
 *
 * @param rContolSet Control set for the simulation.
 * @param rYStore Storage containing the initial states for the simulation.
 * @param rYPStore Storage containing the initial pseudo states for the simulation.
 */
void ForwardTool::
inputControlsStatesAndPseudoStates(ControlSet*& rControlSet,Storage*& rYStore,Storage*& rYPStore)
{
	// Controls
	rControlSet = NULL;
	if(_controlsFileName!="") {
		cout<<"\n\nLoading controls from file "<<_controlsFileName<<"."<<endl;
		rControlSet = new ControlSet(_controlsFileName);
		cout<<"Found "<<rControlSet->getSize()<<" controls."<<endl<<endl;
	}

	// Initial states
	rYStore = NULL;
	if(_statesFileName!="") {
		cout<<"\nLoading states from file "<<_statesFileName<<"."<<endl;
		rYStore = new Storage(_statesFileName);
		cout<<"Found "<<rYStore->getSize()<<" state vectors with time stamps ranging"<<endl;
		cout<<"from "<<rYStore->getFirstTime()<<" to "<<rYStore->getLastTime()<<"."<<endl;
	}

	// Initial pseudo states
	rYPStore = NULL;
	if(_pseudoFileName!="") {
		cout<<"\nLoading pseudo states from file "<<_pseudoFileName<<"."<<endl;
		rYPStore = new Storage(_pseudoFileName);
		cout<<"Found "<<rYPStore->getSize()<<" rows with time stamps ranging"<<endl;
		cout<<"from "<<rYPStore->getFirstTime()<<" to "<<rYPStore->getLastTime()<<"."<<endl;
	}
}

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
		cout<<"No external loads will be applied (external loads file not specified)."<<endl;
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
		cout<<"Low-pass filtering external load kinematics with a cutoff frequency of "
		    <<aLowpassCutoffFrequencyForLoadKinematics<<"..."<<endl;
		qStore->lowpassFIR(order,aLowpassCutoffFrequencyForLoadKinematics);
	} else {
		cout<<"Note- not filtering the external loads model kinematics."<<endl;
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
