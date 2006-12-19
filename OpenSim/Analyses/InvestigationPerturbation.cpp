// InvestigationPerturbation.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "InvestigationPerturbation.h"
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
	if(_controlSet!=0) delete _controlSet;
	if(_copStore!=0) delete _copStore;
	if(_qStore!=0) delete _qStore;
	if(_uStore!=0) delete _uStore;
	if(_yStore!=0) delete _yStore;
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
	_bTor(_bTorProp.getValueDblArray())
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
	_bTor(_bTorProp.getValueDblArray())
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
	_bTor(_bTorProp.getValueDblArray())
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
	_bTor(_bTorProp.getValueDblArray())
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

	// POINTERS
	_controlSet = NULL;
	_copStore = NULL;
	_qStore = NULL;
	_uStore = NULL;
	_yStore = NULL;

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

	// SET OUTPUT PRECISION
	IO::SetPrecision(_outputPrecision);

	// REGISTER TYPES
	Object::RegisterType(ControlLinear());
	Object::RegisterType(ControlLinearNode());

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocument()->getFileName());
	IO::chDir(directoryOfSetupFile);

	// INPUT
	// Controls
	cout<<endl<<endl<<"Loading controls from file "<<_controlsFileName<<".\n";
	_controlSet = new ControlSet(_controlsFileName);
	cout<<"Found "<<_controlSet->getSize()<<" controls.\n\n";
	// Center of pressure
	cout<<endl<<endl<<"Loading center of pressure data from file ";
	cout<<_copFileName<<".\n";
	_copStore = new Storage(_copFileName);
	// Qs and Us
	cout<<endl<<endl<<"Loading generalized coordinates and speeds from files ";
	cout<<_qFileName<<" and "<<_uFileName<<".\n";
	_qStore = new Storage(_qFileName);
	_uStore = new Storage(_uFileName);
	// States
	_yStore = new Storage(_yFileName);

	// CHECK FOR A MODEL
	if(_model==NULL) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	// ASSIGN NUMBERS OF THINGS
	int ny = _model->getNumStates();
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int na = _model->getNumActuators();
	int nb = _model->getNumBodies();
	int numBodyKinCols = 6*nb + 3;

	// CONVERT Qs AND Us TO RADIANS AND QUATERNIONS
	_model->getDynamicsEngine().convertDegreesToRadians(_qStore);
	_model->getDynamicsEngine().convertAnglesToQuaternions(_qStore);
	_model->getDynamicsEngine().convertDegreesToRadians(_uStore);

	// CONSTRUCT CORRECTIVE SPRINGS
	constructCorrectiveSprings();

	// ADD ANALYSES
	// Kinematics
	Kinematics *kinAngles = new Kinematics(_model);
	_model->addAnalysis(kinAngles);
	kinAngles->getPositionStorage()->setWriteSIMMHeader(true);
	kinAngles->setOn(true);
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
	integrand.setControlSet(*_controlSet);
	Manager manager(&integrand);
	manager.setSessionName(getName());
	// Initial and final times
	// If the times lie outside the range for which control values are
	// available, the initial and final times are altered.
	int first = 0;
	ControlLinear *control = (ControlLinear*)_controlSet->get(first);
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


	// RESULT VARIABLES
	int i,m,endIndex;
	char fileName[Object::NAME_LENGTH];
	double PXBody,PYBody,PZBody;
	Array<double> PFXBody(0.0,na),PFYBody(0.0,na),PFZBody(0.0,na);
	int indexCOMX = numBodyKinCols - 3;
	int indexCOMY = numBodyKinCols - 2;
	int indexCOMZ = numBodyKinCols - 1;
	Array<double> lastRowPerturbedKin(0.0,numBodyKinCols);
	Array<double> lastRowPerturbedVel(0.0,numBodyKinCols);
	Array<double> rowUnperturbedKin(0.0,numBodyKinCols);
	Array<double> rowUnperturbedVel(0.0,numBodyKinCols);
	Array<double> rowUnperturbedForces(0.0,na);
	Array<double> daXdf(0.0,na),deltaAX(0.0,na);
	Array<double> daYdf(0.0,na),deltaAY(0.0,na);
	Array<double> daZdf(0.0,na),deltaAZ(0.0,na);
	for(i=0;i<na;i++)	{
		PFXBody[i] = PFYBody[i] = PFZBody[i] = 0.0;
		daXdf[i] = daYdf[i] = daZdf[i] = 0.0;
		deltaAX[i] = deltaAY[i] = deltaAZ[i] = 0,0;
	}

	// Storage objects for results
	Storage *perturbedPos;
	Storage daXdfStore,deltaAXStore,PFXBodyStore;
	Storage daYdfStore,deltaAYStore,PFYBodyStore;
	Storage daZdfStore,deltaAZStore,PFZBodyStore;
	string columnLabels = "time";
	ActuatorSet *as = _model->getActuatorSet();
	for(i=0; i<as->getSize(); i++)
	{
		AbstractActuator *act=as->get(i);
		columnLabels += "\t" + act->getName();
	}

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

	// Storage objects for unperturbed data
	Storage *unperturbedPos=0,*unperturbedVel=0,*unperturbedAcc=0;
	Storage *unperturbedFrc=0;


	//********************************************************************
	// LOOP
	//********************************************************************
	int index;
	double tiPert,tfPert;
	double lastPertTime = _tf - _pertWindow;
	Array<double> yi(0.0,ny);
	for(tiPert=_ti;tiPert<=lastPertTime;tiPert+=_pertIncrement) {

		// SET INITIAL AND FINAL TIME AND THE INITIAL STATES
		index = _yStore->findIndex(tiPert);
		_yStore->getTime(index,tiPert);
		_yStore->getData(index,ny,&yi[0]);
		tfPert = tiPert + _pertWindow;
		manager.setInitialTime(tiPert);
		manager.setFinalTime(tfPert);
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
		IO::makeDir(getResultsDir());
		_model->getAnalysisSet()->printResults("Gc05g1_unpert",getResultsDir());
		
		// INTEGRATE (2) - Record unperturbed muscle forces
		integ->setUseSpecifiedDT(true);
		perturbation->setOn(true);
		perturbation->getUnperturbedForceStorage()->reset(); 
		perturbation->setRecordUnperturbedForces(true);
		cout<<"\nUnperturbed integration 2 to record forces and kinematics\n";
		manager.integrate();
		perturbation->setRecordUnperturbedForces(false);

		// COPY UNPERTURBED DATA
		if(unperturbedPos!=0) { delete unperturbedPos;  unperturbedPos=0; }
		if(unperturbedVel!=0) { delete unperturbedVel;  unperturbedVel=0; }
		if(unperturbedAcc!=0) { delete unperturbedAcc;  unperturbedAcc=0; }
		if(unperturbedFrc!=0) { delete unperturbedFrc;  unperturbedFrc=0; }
		unperturbedPos = new Storage(*kin->getPositionStorage());
		unperturbedVel = new Storage(*kin->getVelocityStorage());
		unperturbedAcc = new Storage(*kin->getAccelerationStorage());
		unperturbedFrc = new Storage(*actuation->getForceStorage());

		// Get unperturbed kinmatics
		endIndex = unperturbedPos->getSize() - 1;
		unperturbedPos->getData(endIndex,numBodyKinCols,&rowUnperturbedKin[0]);
		PXBody = rowUnperturbedKin[indexCOMX];
		PYBody = rowUnperturbedKin[indexCOMY];
		PZBody = rowUnperturbedKin[indexCOMZ];

		// Get unperturbed forces
		unperturbedFrc->getData(0,na,&rowUnperturbedForces[0]);

		// Loop over muscles
		//int tib_ant_l = _model->getActuatorIndex("tib_ant_l");
		//for (m=tib_ant_l;m<=tib_ant_l;m++)	{
		//ai->reset();
		//act = ai->next();
		for (m=0;m<na;m++)	{
			AbstractActuator *act = as->get(m);
			// Set up pertubation callback
			cout<<"\nPerturbation of muscle "<<act->getName()<<" ("<<m<<") in loop"<<endl;
			DerivCallbackSet *callbackSet = _model->getDerivCallbackSet();
			for(i=0;i<callbackSet->getSize();i++)	{
				DerivCallback *callback = callbackSet->getDerivCallback(i);
				if(callback == NULL) continue;
				callback->reset();
			}
			perturbation->reset(); 
			perturbation->setActuator(act); 
			perturbation->setPerturbation(ActuatorPerturbationIndependent::DELTA,+_pertDF);

			// Integrate
 			manager.integrate();

			// Get perturbed kinematics
			perturbedPos = kin->getPositionStorage();
			endIndex = perturbedPos->getSize() - 1;
			perturbedPos->getData(endIndex,numBodyKinCols,&lastRowPerturbedKin[0]);
			PFXBody[m] = lastRowPerturbedKin[indexCOMX];
			PFYBody[m] = lastRowPerturbedKin[indexCOMY];
			PFZBody[m] = lastRowPerturbedKin[indexCOMZ];

			// Compute derivatives
			daXdf[m] = 2*(PFXBody[m]-PXBody)/(_pertDF*_pertWindow*_pertWindow);
			deltaAX[m] = rowUnperturbedForces[m] *daXdf[m];
			daYdf[m] = 2*(PFYBody[m]-PYBody)/(_pertDF*_pertWindow*_pertWindow);
			deltaAY[m] = rowUnperturbedForces[m] *daYdf[m];
			daZdf[m] = 2*(PFZBody[m]-PZBody)/(_pertDF*_pertWindow*_pertWindow);
			deltaAZ[m] = rowUnperturbedForces[m] *daZdf[m];

			cout << "muscle:\t"<<m<<"\tforce:\t"<<rowUnperturbedForces[m]<<endl;
			printf("PFXBody:\t%.16f\tPXBody:\t%.16f\tdifference:\t%.16f\n",
				PFXBody[m],PXBody,PFXBody[m]-PXBody);
			printf("daXdf:\t%.16f\tdeltaAX:\t%.16f\n",daXdf[m],deltaAX[m]);

			printf("PFYBody:\t%.16f\tPYBody:\t%.16f\tdifference:\t%.16f\n",
				PFYBody[m],PYBody,PFYBody[m]-PYBody);
			printf("daYdf:\t%.16f\tdeltaAY:\t%.16f\n",daYdf[m],deltaAY[m]);
			//act = ai->next();
		} //end muscle loop

		// Append to storage objects
		daXdfStore.append(tiPert,na,&daXdf[0]);
		deltaAXStore.append(tiPert,na,&deltaAX[0]);
		PFXBodyStore.append(tiPert,na,&PFXBody[0]);
		daYdfStore.append(tiPert,na,&daYdf[0]);
		deltaAYStore.append(tiPert,na,&deltaAY[0]);
		PFYBodyStore.append(tiPert,na,&PFYBody[0]);
		daZdfStore.append(tiPert,na,&daZdf[0]);
		deltaAZStore.append(tiPert,na,&deltaAZ[0]);
		PFZBodyStore.append(tiPert,na,&PFZBody[0]);

		// Print results
		IO::makeDir("ResultsPerturbed");
		sprintf(fileName,"./ResultsPerturbed/daXdf_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		daXdfStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/deltaAX_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		deltaAXStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/PFXBody_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		PFXBodyStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/daYdf_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		daYdfStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/deltaAY_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		deltaAYStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/PFYBody_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		PFYBodyStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/daZdf_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		daZdfStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/deltaAZ_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
		deltaAZStore.print(fileName);
		sprintf(fileName,"./ResultsPerturbed/PFZBody_dt_%.3f_df_%.3lf.sto",_pertWindow,_pertDF);
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
	// SCALING FUNCTIONS FOR SPRINGS
	double tScale,dtScale=0.001;
	double tiScale = _qStore->getFirstTime();
	double tfScale = _qStore->getLastTime();
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

	// CONSTRUCT SPRINGS
	VectorGCVSplineR1R3 *cop;
	int size = _copStore->getSize();
	double *t=0,*x=0,*y=0,*z=0;
	_copStore->getTimeColumn(t);

	// LINEAR
	// right
	string colName;
	colName = "ground_force_px_r";
	_copStore->getDataColumn(colName,x);
	colName = "ground_force_py_r";
	_copStore->getDataColumn(colName,y);
	colName = "ground_force_pz_r";
	_copStore->getDataColumn(colName,z);
	cop = new VectorGCVSplineR1R3(5,size,t,x,y,z);
	LinearSpring *rLin = new LinearSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_r"));
	rLin->computePointAndTargetFunctions(_qStore,_uStore,*cop);
	rLin->setKValue(&_kLin[0]);
	rLin->setBValue(&_bLin[0]);
	rLin->setScaleFunction(rScaleTranslationalSpline);
	Storage *rLinStore = rLin->getAppliedForceStorage();
	_model->addDerivCallback(rLin);
	// left linear
	colName = "ground_force_px_l";
	_copStore->getDataColumn(colName,x);
	colName = "ground_force_py_l";
	_copStore->getDataColumn(colName,y);
	colName = "ground_force_pz_l";
	_copStore->getDataColumn(colName,z);
	cop = new VectorGCVSplineR1R3(5,size,t,x,y,z);
	LinearSpring *lLin = new LinearSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_l"));
	lLin->computePointAndTargetFunctions(_qStore,_uStore,*cop);
	lLin->setKValue(&_kLin[0]);
	lLin->setBValue(&_bLin[0]);
	lLin->setScaleFunction(lScaleTranslationalSpline);
	Storage *lLinStore = lLin->getAppliedForceStorage();
	_model->addDerivCallback(lLin);

	// TORSIONAL
	// right
	TorsionalSpring *rTrq = new TorsionalSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_r"));
	rTrq->computeTargetFunctions(_qStore,_uStore);
	rTrq->setKValue(&_kTor[0]);
	rTrq->setBValue(&_bTor[0]);
	rTrq->setScaleFunction(rScaleTorsionalSpline);
	Storage *rTrqStore = rTrq->getAppliedTorqueStorage();
	_model->addDerivCallback(rTrq);
	// left
	TorsionalSpring *lTrq = new TorsionalSpring(_model,_model->getDynamicsEngine().getBodySet()->get("calcn_l"));
	lTrq->computeTargetFunctions(_qStore,_uStore);
	lTrq->setKValue(&_kTor[0]);
	lTrq->setBValue(&_bTor[0]);
	lTrq->setScaleFunction(lScaleTorsionalSpline);
	Storage *lTrqStore = lTrq->getAppliedTorqueStorage();
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
