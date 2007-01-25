// InvestigationPerturbation.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "InvestigationPerturbation.h"
#include "InvestigationForward.h"
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



using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InvestigationPerturbation::~InvestigationPerturbation()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InvestigationPerturbation::InvestigationPerturbation() :
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_copFileName(_copFileNameProp.getValueStr()),
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
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationPerturbation");
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
InvestigationPerturbation::InvestigationPerturbation(const string &aFileName):
	Investigation(aFileName),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_copFileName(_copFileNameProp.getValueStr()),
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
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationPerturbation");
	setNull();
	updateFromXMLNode();
	if (_model) addAnalysisSetToModel();
}
//_____________________________________________________________________________
/**
 * Construct from a DOMElement.
 */
InvestigationPerturbation::InvestigationPerturbation(DOMElement *aElement):
	Investigation(aElement),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_copFileName(_copFileNameProp.getValueStr()),
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
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setType("InvestigationPerturbation");
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
InvestigationPerturbation::
InvestigationPerturbation(const InvestigationPerturbation &aInvestigation):
	Investigation(aInvestigation),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_controlsFileName(_controlsFileNameProp.getValueStr()),
	_copFileName(_copFileNameProp.getValueStr()),
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
	_kLin(_kLinProp.getValueDblArray()),
	_bLin(_bLinProp.getValueDblArray()),
	_kTor(_kTorProp.getValueDblArray()),
	_bTor(_bTorProp.getValueDblArray()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl())
{
	setNull();
	*this = aInvestigation;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 */
Object* InvestigationPerturbation::
copy() const
{
	InvestigationPerturbation *object = new InvestigationPerturbation(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Virtual copy constructor from DOMElement.
 */
Object* InvestigationPerturbation::
copy(DOMElement *aElement) const
{
	InvestigationPerturbation *object = new InvestigationPerturbation(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InvestigationPerturbation::
setNull()
{
	setupProperties();

	// FOOT CONTACT EVENTS
	_rHeelStrike = _rFootFlat =_rHeelOff = _rToeOff = 0.0;
	_lHeelStrike = _lFootFlat =_lHeelOff = _lToeOff = 0.0;

	// CORRECTIVE SPRING PARAMETERS
	_tau = 0.001;
	_kLin.setSize(3);
	_kLin[0] = _kLin[1] = _kLin[2] = 5000000.0;
	_bLin.setSize(3);
	_bLin[0] = _bLin[1] = _bLin[2] = 1500.0;
	_kTor.setSize(3);
	_kTor[0] = _kTor[1] = _kTor[2] = 100000.0;
	_bTor.setSize(3);
	_bTor[0] = _bTor[1] = _bTor[2] = 1000.0;

	_adjustedCOMBody = "";
	_adjustedCOMFileName = "";
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
void InvestigationPerturbation::setupProperties()
{
	// PERTURBATION PARAMETERS
	_pertWindowProp.setName("perturbation_time_window");
	_propertySet.append( &_pertWindowProp );

	_pertIncrementProp.setName("perturbation_time_increment");
	_propertySet.append( &_pertIncrementProp );

	_pertDFProp.setName("perturbation_size");
	_propertySet.append( &_pertDFProp);


	// CORRECTIVE SPRING PARAMETERS
	_tauProp.setName("scaling_rise_time");
	_propertySet.append( &_tauProp );

	_kLinProp.setName("corrective_spring_linear_stiffness");
	_propertySet.append( &_kLinProp );

	_bLinProp.setName("corrective_spring_linear_damping");
	_propertySet.append( &_bLinProp );

	_kTorProp.setName("corrective_spring_torsional_stiffness");
	_propertySet.append( &_kTorProp );

	_bTorProp.setName("corrective_spring_torsional_damping");
	_propertySet.append( &_bTorProp );


	// INPUT FILE NAMES
	_controlsFileNameProp.setName("controls_file_name");
	_propertySet.append( &_controlsFileNameProp );

	_copFileNameProp.setName("cop_file_name");
	_propertySet.append( &_copFileNameProp );

	_qFileNameProp.setName("coordinates_file_name");
	_propertySet.append( &_qFileNameProp );

	_uFileNameProp.setName("speeds_file_name");
	_propertySet.append( &_uFileNameProp );

	_yFileNameProp.setName("states_file_name");
	_propertySet.append( &_yFileNameProp );


	// FOOT CONTACT EVENT TIMES
	_rHeelStrikeProp.setName("r_heel_strike");
	_propertySet.append( &_rHeelStrikeProp );

	_rFootFlatProp.setName("r_foot_flat");
	_propertySet.append( &_rFootFlatProp );

	_rHeelOffProp.setName("r_heel_off");
	_propertySet.append( &_rHeelOffProp );

	_rToeOffProp.setName("r_toe_off");
	_propertySet.append( &_rToeOffProp );

	_lHeelStrikeProp.setName("l_heel_strike");
	_propertySet.append( &_lHeelStrikeProp );

	_lFootFlatProp.setName("l_foot_flat");
	_propertySet.append( &_lFootFlatProp );

	_lHeelOffProp.setName("l_heel_off");
	_propertySet.append( &_lHeelOffProp );

	_lToeOffProp.setName("l_toe_off");
	_propertySet.append( &_lToeOffProp );

	// EXTERNAL LOADS (e.g. GROUND REACTION FORCES)
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

	_adjustedCOMBodyProp.setComment("Name of the body whose center of mass is adjusted.");
	_adjustedCOMBodyProp.setName("adjusted_com_body");
	_propertySet.append( &_adjustedCOMBodyProp );

	_adjustedCOMFileNameProp.setComment("Name of the file specifying a change to the center of mass of a body."
												   " This adjustment is made to remove dc offset in the residuals.");
	_adjustedCOMFileNameProp.setName("adjusted_com_file_name");
	_propertySet.append( &_adjustedCOMFileNameProp );
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
InvestigationPerturbation& InvestigationPerturbation::
operator=(const InvestigationPerturbation &aInvestigation)
{
	// BASE CLASS
	Investigation::operator=(aInvestigation);

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
void InvestigationPerturbation::run()
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

	// ALTER COM ?
	InvestigationForward::adjustCOM(_model,_adjustedCOMFileName,_adjustedCOMBody);

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocument()->getFileName());
	IO::chDir(directoryOfSetupFile);

	// INPUT
	// Controls
	cout<<endl<<endl<<"Loading controls from file "<<_controlsFileName<<".\n";
	ControlSet *controlSet = new ControlSet(_controlsFileName);
	cout<<"Found "<<controlSet->getSize()<<" controls.\n\n";
	// States
	Storage *yStore = new Storage(_yFileName);

	// ASSIGN NUMBERS OF THINGS
	int na = _model->getNumActuators();
	int nb = _model->getNumBodies();
	int numBodyKinCols = 6*nb + 3;
	int indexCOMX = numBodyKinCols - 3;
	int indexCOMY = numBodyKinCols - 2;
	int indexCOMZ = numBodyKinCols - 1;

	// GROUND REACTION FORCES
	InvestigationForward::initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics);

	// CONSTRUCT CORRECTIVE SPRINGS
	constructCorrectiveSprings();

	// ADD ANALYSES
	// Body kinematics
	BodyKinematics *kin = new BodyKinematics(_model);
	_model->addAnalysis(kin);
	kin->getPositionStorage()->setWriteSIMMHeader(true);		
	kin->setOn(true);
	// Actuation
	Actuation *actuation = new Actuation(_model);
	_model->addAnalysis(actuation);
	actuation->setOn(true);

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

	bool gravity_perturbation = true;
	int gravity_axis = 1;
	double original_gravity[3];
	int nperturb = na + (gravity_perturbation ? 1 : 0);

	if(gravity_perturbation) _model->getGravity(original_gravity);

	// RESULT VARIABLES
	Array<double> PFXBody(0.0,nperturb),PFYBody(0.0,nperturb),PFZBody(0.0,nperturb);
	Array<double> daXdf(0.0,nperturb),deltaAX(0.0,nperturb);
	Array<double> daYdf(0.0,nperturb),deltaAY(0.0,nperturb);
	Array<double> daZdf(0.0,nperturb),deltaAZ(0.0,nperturb);
	for(int i=0;i<nperturb;i++)	{
		PFXBody[i] = PFYBody[i] = PFZBody[i] = 0.0;
		daXdf[i] = daYdf[i] = daZdf[i] = 0.0;
		deltaAX[i] = deltaAY[i] = deltaAZ[i] = 0.0;
	}

	// Storage objects for results
	Storage daXdfStore,deltaAXStore,PFXBodyStore;
	Storage daYdfStore,deltaAYStore,PFYBodyStore;
	Storage daZdfStore,deltaAZStore,PFZBodyStore;
	string columnLabels = "time";
	ActuatorSet *as = _model->getActuatorSet();
	for(int i=0; i<as->getSize(); i++)
	{
		columnLabels += "\t" + as->get(i)->getName();
	}
	if(gravity_perturbation)
		columnLabels += "\tgravity";

	daXdfStore.setName("daXdf");
	daXdfStore.setColumnLabels(columnLabels.c_str());
	deltaAXStore.setName("deltaAX");
	deltaAXStore.setColumnLabels(columnLabels.c_str());
	PFXBodyStore.setName("PFXBody");
	PFXBodyStore.setColumnLabels(columnLabels.c_str());
	daYdfStore.setName("daYdf");
	daYdfStore.setColumnLabels(columnLabels.c_str());
	deltaAYStore.setName("deltaAY");
	deltaAYStore.setColumnLabels(columnLabels.c_str());
	PFYBodyStore.setName("PFYBody");
	PFYBodyStore.setColumnLabels(columnLabels.c_str());
	daZdfStore.setName("daZdf");
	daZdfStore.setColumnLabels(columnLabels.c_str());
	deltaAZStore.setName("deltaAZ");
	deltaAZStore.setColumnLabels(columnLabels.c_str());
	PFZBodyStore.setName("PFZBody");
	PFZBodyStore.setColumnLabels(columnLabels.c_str());

	//********************************************************************
	// Run an unperturbed forward simulation to compute and write out
	// unperturbed data
	//********************************************************************
#if 0
	{
		int index = yStore->findIndex(_ti);
		double tiPert;
		yStore->getTime(index,tiPert);
		double tfPert = _tf;
		manager.setInitialTime(tiPert);
		manager.setFinalTime(tfPert);
		const Array<double> &yi = yStore->getStateVector(index)->getData();
		_model->setInitialStates(&yi[0]);
		perturbation->setOn(false);
		cout<<"\nFull unperturbed integration from "<<tiPert<<" to "<<tfPert<<endl;
		manager.integrate();
		IO::makeDir(getResultsDir());
		_model->getAnalysisSet()->printResults(getName()+"_unpert",getResultsDir());
	}
#endif

	// From now on we'll only need the last state vectors recorded in these analyses, so we
	// set their step interval to a large number to avoid them computing and writing their
	// data at the (many) individual integration steps.
	kin->setStepInterval(1000000);
	actuation->setStepInterval(1000000);

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
		kin->getPositionStorage()->reset();
		kin->getVelocityStorage()->reset();
		kin->getAccelerationStorage()->reset();
		actuation->getForceStorage()->reset();

		// INTEGRATE (1)
		integ->setUseSpecifiedDT(false);
		perturbation->setOn(false);
		cout<<"\n\nUnperturbed integration (1) from "<<tiPert<<" to "<<tfPert<<endl;
		manager.integrate();
		
		// INTEGRATE (2) - Record unperturbed muscle forces
		integ->setUseSpecifiedDT(true);
		perturbation->setOn(true);
		perturbation->getUnperturbedForceStorage()->reset(); 
		perturbation->setRecordUnperturbedForces(true);
		cout<<"\nUnperturbed integration 2 to record forces and kinematics\n";
		manager.integrate();
		perturbation->setRecordUnperturbedForces(false);

		// Get unperturbed kinmatics
		const Array<double> &rowUnperturbedKin = kin->getPositionStorage()->getLastStateVector()->getData();
		double PXBody = rowUnperturbedKin[indexCOMX];
		double PYBody = rowUnperturbedKin[indexCOMY];
		double PZBody = rowUnperturbedKin[indexCOMZ];

		// Get unperturbed forces -- make a copy of the array
		Array<double> rowUnperturbedForces = actuation->getForceStorage()->getStateVector(0)->getData();

		// Loop over muscles
		for (int m=0;m<nperturb;m++)	{
			_model->getDerivCallbackSet()->resetCallbacks();
			perturbation->reset(); 
			if(m<na) {
				AbstractActuator *act = as->get(m);
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

			// Get perturbed kinematics
			const Array<double> &lastRowPerturbedKin = kin->getPositionStorage()->getLastStateVector()->getData();
			PFXBody[m] = lastRowPerturbedKin[indexCOMX];
			PFYBody[m] = lastRowPerturbedKin[indexCOMY];
			PFZBody[m] = lastRowPerturbedKin[indexCOMZ];

			// Compute derivatives
			daXdf[m] = 2*(PFXBody[m]-PXBody)/(_pertDF*_pertWindow*_pertWindow);
			daYdf[m] = 2*(PFYBody[m]-PYBody)/(_pertDF*_pertWindow*_pertWindow);
			daZdf[m] = 2*(PFZBody[m]-PZBody)/(_pertDF*_pertWindow*_pertWindow);

			if(m<na) {
				cout << "muscle:\t"<<m<<"\tforce:\t"<<rowUnperturbedForces[m]<<endl;
				deltaAX[m] = rowUnperturbedForces[m] *daXdf[m];
				deltaAY[m] = rowUnperturbedForces[m] *daYdf[m];
				deltaAZ[m] = rowUnperturbedForces[m] *daZdf[m];
			} else {
				cout << "gravity original:\t"<<original_gravity[gravity_axis]<<endl;
				deltaAX[m] = original_gravity[gravity_axis] *daXdf[m];
				deltaAY[m] = original_gravity[gravity_axis] *daYdf[m];
				deltaAZ[m] = original_gravity[gravity_axis] *daZdf[m];
				// undo gravity perturbation
				_model->setGravity(original_gravity);
			}

			cout << "PFXBody:\t"<<PFXBody[m]<<"\tPXBody:\t"<<PXBody<<"\tdifference:\t"<<PFXBody[m]-PXBody<<endl;
			cout << "daXdf:\t"<<daXdf[m]<<"\tdeltaAX:\t"<<deltaAX[m]<<endl;
			cout << "PFYBody:\t"<<PFYBody[m]<<"\tPYBody:\t"<<PYBody<<"\tdifference:\t"<<PFYBody[m]-PYBody<<endl;
			cout << "daYdf:\t"<<daYdf[m]<<"\tdeltaAY:\t"<<deltaAY[m]<<endl;
		} //end muscle loop

		// Append to storage objects
		daXdfStore.append(tiPert,nperturb,&daXdf[0]);
		deltaAXStore.append(tiPert,nperturb,&deltaAX[0]);
		PFXBodyStore.append(tiPert,nperturb,&PFXBody[0]);
		daYdfStore.append(tiPert,nperturb,&daYdf[0]);
		deltaAYStore.append(tiPert,nperturb,&deltaAY[0]);
		PFYBodyStore.append(tiPert,nperturb,&PFYBody[0]);
		daZdfStore.append(tiPert,nperturb,&daZdf[0]);
		deltaAZStore.append(tiPert,nperturb,&deltaAZ[0]);
		PFZBodyStore.append(tiPert,nperturb,&PFZBody[0]);

		// Print results
		IO::makeDir(getResultsDir());
		char fileName[Object::NAME_LENGTH];
		sprintf(fileName,"%s/daXdf_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		daXdfStore.print(fileName);
		sprintf(fileName,"%s/deltaAX_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		deltaAXStore.print(fileName);
		sprintf(fileName,"%s/PFXBody_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		PFXBodyStore.print(fileName);
		sprintf(fileName,"%s/daYdf_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		daYdfStore.print(fileName);
		sprintf(fileName,"%s/deltaAY_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		deltaAYStore.print(fileName);
		sprintf(fileName,"%s/PFYBody_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		PFYBodyStore.print(fileName);
		sprintf(fileName,"%s/daZdf_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		daZdfStore.print(fileName);
		sprintf(fileName,"%s/deltaAZ_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		deltaAZStore.print(fileName);
		sprintf(fileName,"%s/PFZBody_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),_pertWindow,_pertDF);
		PFZBodyStore.print(fileName);

	} // end time loop
	//***************************************************************************
	IO::chDir(saveWorkingDirectory);

}


//_____________________________________________________________________________
/**
 * Construct the corrective springs.
 */
void InvestigationPerturbation::
constructCorrectiveSprings()
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
	double tScale,dtScale=0.001;
	double tiScale = qStore.getFirstTime();
	double tfScale = qStore.getLastTime();
	double value1,value2;
	Array<double> timeScale(0.0);
	Array<double> rLinearScale(0.0),rTorsionalScale(0.0);
	Array<double> lLinearScale(0.0),lTorsionalScale(0.0);
	for(tScale=tiScale;tScale<=tfScale;tScale+=dtScale) {
		// time
		timeScale.append(tScale);
		// rLinear
		value1 = rdMath::SigmaUp(_tau,_rHeelStrike,tScale);
		value2 = rdMath::SigmaDn(_tau,_rToeOff,tScale);
		rLinearScale.append(value1+value2-1.0);
		// rTorsional
		value1 = rdMath::SigmaUp(_tau,_rFootFlat,tScale);
		value2 = rdMath::SigmaDn(_tau,_rHeelOff,tScale);
		rTorsionalScale.append(value1+value2-1.0);
		// lLinear
		value1 = rdMath::SigmaUp(_tau,_lHeelStrike,tScale);
		value2 = rdMath::SigmaDn(_tau,_lToeOff,tScale);
		lLinearScale.append(value1+value2-1.0);
		// lTorsional
		value1 = rdMath::SigmaUp(_tau,_lFootFlat,tScale);
		value2 = rdMath::SigmaDn(_tau,_lHeelOff,tScale);
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

	// Center of pressure
	cout<<"\n\nLoading center of pressure data from file "<<_copFileName<<".\n";
	Storage copStore(_copFileName);

	// CONSTRUCT SPRINGS
	VectorGCVSplineR1R3 *cop;
	int size = copStore.getSize();
	double *t=0,*x=0,*y=0,*z=0;
	copStore.getTimeColumn(t);

	// Look for center of pressure columns in the storage file.  We handle two types of column labels: ground_force_p{x,y,z}_{r,l} (i.e. explicitly
	// suffixing with right and left sides, and ground_force_p{x,y,z} which doesn't explicitly give side and instead we assume the first occurence
	// is the right side and the second occurence is the left side.
	int rightCopX, rightCopY, rightCopZ, leftCopX, leftCopY, leftCopZ;
	rightCopX = copStore.getColumnIndex("ground_force_px_r");
	if(rightCopX<0) {
		rightCopX = copStore.getColumnIndex("ground_force_px");
		rightCopY = copStore.getColumnIndex("ground_force_py");
		rightCopZ = copStore.getColumnIndex("ground_force_pz");
		leftCopX = copStore.getColumnIndex("ground_force_px", rightCopX + 2);
		leftCopY = copStore.getColumnIndex("ground_force_py", rightCopY + 2);
		leftCopZ = copStore.getColumnIndex("ground_force_pz", rightCopZ + 2);
		if(rightCopX<0 || rightCopY<0 || rightCopZ<0 || leftCopX<0 || leftCopY<0 || leftCopZ<0) {
			string msg = "InvestigationPerturbation: ERR- One or more of the COP columns ground_force_p{x,y,z} not found in "+_copFileName+".";
			throw Exception(msg,__FILE__,__LINE__);
		}
	} 
	else {
		rightCopY = copStore.getColumnIndex("ground_force_py_r");
		rightCopZ = copStore.getColumnIndex("ground_force_pz_r");
		leftCopX = copStore.getColumnIndex("ground_force_px_l");
		leftCopY = copStore.getColumnIndex("ground_force_py_l");
		leftCopZ = copStore.getColumnIndex("ground_force_pz_l");
		if(rightCopX<0 || rightCopY<0 || rightCopZ<0 || leftCopX<0 || leftCopY<0 || leftCopZ<0) {
			string msg = "InvestigationPerturbation: ERR- One or more of the COP columns ground_force_p{x,y,z}_{r,l} not found in "+_copFileName+".";
			throw Exception(msg,__FILE__,__LINE__);
		}
	}

	// LINEAR
	// right
	copStore.getDataColumn(rightCopX,x);
	copStore.getDataColumn(rightCopY,y);
	copStore.getDataColumn(rightCopZ,z);
	cop = new VectorGCVSplineR1R3(5,size,t,x,y,z);
	LinearSpring *rLin = new LinearSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_r"));
	rLin->computePointAndTargetFunctions(&qStore,&uStore,*cop);
	rLin->setKValue(&_kLin[0]);
	rLin->setBValue(&_bLin[0]);
	rLin->setScaleFunction(rScaleTranslationalSpline);
	_model->addDerivCallback(rLin);
	delete cop;

	// left linear
	copStore.getDataColumn(leftCopX,x);
	copStore.getDataColumn(leftCopY,y);
	copStore.getDataColumn(leftCopZ,z);
	cop = new VectorGCVSplineR1R3(5,size,t,x,y,z);
	LinearSpring *lLin = new LinearSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_l"));
	lLin->computePointAndTargetFunctions(&qStore,&uStore,*cop);
	lLin->setKValue(&_kLin[0]);
	lLin->setBValue(&_bLin[0]);
	lLin->setScaleFunction(lScaleTranslationalSpline);
	_model->addDerivCallback(lLin);
	delete cop;
	delete[] t; delete[] x; delete[] y; delete[] z;

	// TORSIONAL
	// right
	TorsionalSpring *rTrq = new TorsionalSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_r"));
	rTrq->computeTargetFunctions(&qStore,&uStore);
	rTrq->setKValue(&_kTor[0]);
	rTrq->setBValue(&_bTor[0]);
	rTrq->setScaleFunction(rScaleTorsionalSpline);
	_model->addDerivCallback(rTrq);
	// left
	TorsionalSpring *lTrq = new TorsionalSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_l"));
	lTrq->computeTargetFunctions(&qStore,&uStore);
	lTrq->setKValue(&_kTor[0]);
	lTrq->setBValue(&_bTor[0]);
	lTrq->setScaleFunction(lScaleTorsionalSpline);
	_model->addDerivCallback(lTrq);
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
void InvestigationPerturbation::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	cout<<"InvestigationPerturbation.printResults: ";
	cout<<"Printing results of investigation "<<getName()<<".\n";
}
