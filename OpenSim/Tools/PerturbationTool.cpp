// PerturbationTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "PerturbationTool.h"
#include "ForwardTool.h"
#include <time.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Model/OpenSimForceSubsystem.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/ActuatorPerturbationIndependent.h>



using namespace std;
using namespace OpenSim;
using namespace SimTK;


static bool Check = true;



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
	_perturbGravity(_perturbGravityProp.getValueBool())
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
PerturbationTool::PerturbationTool(const string &aFileName,bool aUpdateFromXMLNode,bool aLoadModel):
	ForwardTool(aFileName,false,false),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_actuatorsToPerturb(_actuatorsToPerturbProp.getValueStrArray()),
	_perturbGravity(_perturbGravityProp.getValueBool())
{
	setType("PerturbationTool");
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
	if(aLoadModel) { 
		loadModel(aFileName); 
		setToolOwnsModel(true);
	}
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
	ForwardTool(aTool),
	_pertWindow(_pertWindowProp.getValueDbl()),
	_pertIncrement(_pertIncrementProp.getValueDbl()),
	_pertDF(_pertDFProp.getValueDbl()),
	_actuatorsToPerturb(_actuatorsToPerturbProp.getValueStrArray()),
	_perturbGravity(_perturbGravityProp.getValueBool())
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
	AbstractTool::operator=(aTool);

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
bool PerturbationTool::run()
{
	cout<<"Running tool "<<getName()<<".\n";

	// CHECK FOR A MODEL
	if(_model==NULL) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}
    bool externalLoads = createExternalLoads(_externalLoadsFileName, 
                         _externalLoadsModelKinematicsFileName,
                         *_model );

	// Add actuation analysis -- needed in order to evaluate unperturbed forces
	// Actuation
	Actuation *actuation = new Actuation(_model);
	_model->addAnalysis(actuation);
	actuation->setStepInterval(1);

    // Re create the system with forces above and Realize the topology
    SimTK::State& si = _model->initSystem();
    _model->getSystem().realize(si, Stage::Position );
   
	loadStatesStorage(_statesFileName, _yStore);  // Is using _yStore the right thing to do?

    // set the desired states for controllers  
    _model->updControllerSet().setDesiredStates( _yStore );

	// SET OUTPUT PRECISION
	IO::SetPrecision(_outputPrecision);

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	// Modify _ti to match time of closest initial state vector 
// determineInitialTimeFromStatesStorage(_ti);

	// Initial and final times
	// If the times lie outside the range for which control values are
	// available, the initial and final times are altered.
	double ti = _model->getControllerSet().getFirstTime();
	double tf = _model->getControllerSet().getLastTime();

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
	
	// GROUND REACTION FORCES
	if( externalLoads ) {
	    initializeExternalLoads(si, _ti, _tf, *_model,
        _externalLoadsFileName,_externalLoadsModelKinematicsFileName,
        _lowpassCutoffFrequencyForLoadKinematics);
     }

	// CORRECTIVE SPRINGS
	// Perturbation Tool have settings for corrective springs for exactly two forces
	// We'll keep this assumption for now and use _externalFroces[0], [1] 
	if (_externalForces.getSize()!=2){
		string msg = "ERROR- Perturbation Tool's number of external forces is not 2. Unsupported...";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}
	addCorrectiveSprings(si,_yStore,(PrescribedForce*)&(_externalForces[0]),(PrescribedForce*)&(_externalForces[1]));

	// Gather actuators to perturb
	const Set<Actuator>& fSet = _model->getActuators();
	ArrayPtrs<Actuator> actuators;
	Array<int> mapPerturbedActuatorsToAllActuators;
	actuators.setMemoryOwner(false);
	actuators.setSize(_actuatorsToPerturb.getSize());
	for(int i=0; i<_actuatorsToPerturb.getSize(); i++) {
		if(_actuatorsToPerturb[i] == "all") {
			actuators.setSize(fSet.getSize());
			for(int j=0,k=0;j<fSet.getSize();j++) {
                Actuator& act = fSet.get(j);
			    actuators.set(k,&act);
				mapPerturbedActuatorsToAllActuators.append(k);
                k++;
			}
			break;
		}
		int index = fSet.getIndex(_actuatorsToPerturb[i]);
		if(index<0) throw Exception("PerturbationTool: ERR- Could not find actuator '"+_actuatorsToPerturb[i]+
										    "' (listed in "+_actuatorsToPerturbProp.getName()+") in model's actuator set.",__FILE__,__LINE__);
        Actuator& act = fSet.get(index);
	    actuators.set(i,&act);
		mapPerturbedActuatorsToAllActuators.append(index);
	}

	if(_actuatorsToPerturb.getSize() == 0 && !_perturbGravity)
		//throw Exception("PerturbationTool: ERR- Nothing to perturb (no actuators selected and gravity perturbation disabled).",__FILE__,__LINE__);
		cout << "PerturbationTool: WARNING- Nothing will be perturbed (no actuators to perturb and gravity perturbation is off)" << endl;

	Kinematics *kin=0;
	BodyKinematics *bodyKin=0;
	AnalysisSet& analysisSet = _model->updAnalysisSet();
	for(int i=0;i<analysisSet.getSize();i++) {
		if(!analysisSet.get(i).getOn()) continue;
		if(dynamic_cast<Kinematics*>(&analysisSet.get(i))) kin=dynamic_cast<Kinematics*>(&analysisSet.get(i));
		else if(dynamic_cast<BodyKinematics*>(&analysisSet.get(i))) bodyKin=dynamic_cast<BodyKinematics*>(&analysisSet.get(i));
	}

    // Re create the system with forces above and Realize the topology
	SimTK::State s = _model->initSystem();
    _model->getSystem().realize(s, Stage::Position );

	// SETUP SIMULATION
	// Manager
    RungeKuttaMersonIntegrator* integrator = new RungeKuttaMersonIntegrator(_model->getSystem());
    Manager manager(*_model, *integrator);
	manager.setSessionName(getName());

	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);
    // Initialize integrator
    integrator->setInternalStepLimit(_maxSteps);
    integrator->setMaximumStepSize(_maxDT);
    integrator->setAccuracy(_errorTolerance);
    
	cout<<"\n\nPerforming perturbations over the range ti=";
	cout<<_ti<<" to tf="<<_tf<<endl<<endl;

	// Pertubation callback
	ActuatorPerturbationIndependent *perturbation = 
		new ActuatorPerturbationIndependent(_model);

    _model->setPerturbation(perturbation); 

	int gravity_axis = 1;
	Vec3 original_gravity;
	int nperturb = actuators.getSize() + (_perturbGravity ? 1 : 0);

	if(_perturbGravity) original_gravity = _model->getGravity();

	Array<string> columnLabels;
	columnLabels.append("time");
	for(int i=0; i<actuators.getSize(); i++) columnLabels.append(actuators.get(i)->getName());
	if(_perturbGravity) columnLabels.append("gravity");

	// Figure out which columns are being recorded in the Kinematics and BodyKinematics analyses...
	// but make sure to ignore the first (time) column
	int ncoords = kin ? kin->getPositionStorage()->getColumnLabels().getSize()-1 : 0;
	int nbodycoords = bodyKin ? bodyKin->getPositionStorage()->getColumnLabels().getSize()-1 : 0;
	int nvalues = ncoords + nbodycoords;
	if(nvalues==0) 
		throw Exception("PerturbationTool.run: ERROR- No (active) analyses found -- no perturbation to compute.",__FILE__,__LINE__);

	Array<string> values_name("",nvalues);
	for(int i=0;i<ncoords;i++)
		values_name[i] = kin->getPositionStorage()->getColumnLabels()[i+1];
	for(int i=0;i<nbodycoords;i++)
		values_name[ncoords+i] = bodyKin->getPositionStorage()->getColumnLabels()[i+1];

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
			values_name[i] = kin->getPositionStorage()->getColumnLabels()[i+1];
			cout << (i>0?", ":"") << values_name[i];
		}
		cout << endl;
	}
	if(nbodycoords) {
		cout << "BodyKinematics: ";
		for(int i=0;i<nbodycoords;i++) {
			values_name[ncoords+i] = bodyKin->getPositionStorage()->getColumnLabels()[i+1];
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
		values_perturbedStorage[i]->setColumnLabels(columnLabels);
		values_dAdFStorage[i]->setName(values_name[i]+"_dAdF");
		values_dAdFStorage[i]->setColumnLabels(columnLabels);
		values_deltaAStorage[i]->setName(values_name[i]+"_deltaA");
		values_deltaAStorage[i]->setColumnLabels(columnLabels);
	}

	Storage unperturbedAccelStorage;
	unperturbedAccelStorage.setName("unperturbedAccel");
	columnLabels.setSize(0);
	columnLabels.append("time");
	for(int i=0;i<nvalues;i++) columnLabels.append(values_name[i]);
	unperturbedAccelStorage.setColumnLabels(columnLabels);

	Array<double> deltaA(0,nperturb);

	// From now on we'll only need the last state vectors recorded in these analyses, so we
	// set their step interval to a large number to avoid them computing and writing their
	// data at the (many) individual integration steps.
	int stepInterval = 1000000;
	if(Check) stepInterval = 1;
	if(kin && !_outputDetailedResults) kin->setStepInterval(stepInterval);
	if(bodyKin) bodyKin->setStepInterval(stepInterval);

	IO::makeDir(getResultsDir());

	time_t startTime,finishTime,iterationTime;
	time(&startTime);
	iterationTime = startTime;
	struct tm *localTime = localtime(&startTime);
	double elapsedTime;

	cout<<"================================================================\n";
	cout<<"Start time = "<<asctime(localTime);
	cout<<"================================================================\n";

	//********************************************************************
	// LOOP
	//********************************************************************
	double lastPertTime = _tf - _pertWindow;
    bool resetGravity = false;
	for(double t=_ti;t<=lastPertTime;t+=_pertIncrement) {
		// SET INITIAL AND FINAL TIME AND THE INITIAL STATES
		int index = _yStore->findIndex(t);
		double tiPert;
		_yStore->getTime(index,tiPert);
		double tfPert = tiPert + _pertWindow;
		manager.setInitialTime(tiPert);
		manager.setFinalTime(tfPert);
        if( resetGravity ) {
  		    _model->setGravity(original_gravity);
  		    s = _model->initSystem();
  		    integrator = new RungeKuttaMersonIntegrator(_model->getSystem());
  		    integrator->setInternalStepLimit(_maxSteps);
  		    integrator->setMaximumStepSize(_maxDT);
  		    integrator->setAccuracy(_errorTolerance);
  		    manager.setIntegrator( integrator );
  		    resetGravity = false;
  	 	}
  		 
  		// set state to initial conditions
		_yStore->getData(index, s.getNY(), &s.updY()[0]);

		// RESET ANALYSES
		if(actuation->getForceStorage())
			actuation->getForceStorage()->reset();
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

	   // SOLVE FOR EQUILIBRIUM FOR AUXILIARY STATES (E.G., MUSCLE FIBER LENGTHS)
 	   if(_solveForEquilibriumForAuxiliaryStates) {
	  	  _model->computeEquilibriumForAuxiliaryStates(s);  
	   }
       // TURN OFF CORRECTIVE SPRINGS
       if(_body1Lin) _body1Lin->setOn(false);
  	   if(_body1Tor) _body1Tor->setOn(false);
  	   if(_body2Lin) _body2Lin->setOn(false);
  	   if(_body2Tor) _body2Tor->setOn(false);

		// INTEGRATE (1)
		// This integration is to record the actuator forces applied during
		// the unperturbed simulation.
		perturbation->setOn(false);
		cout<<"\n\nUnperturbed integration (1) from "<<tiPert<<" to "<<tfPert;
		cout<<" to record the unperturbed kinematics and actuator forces."<<endl;
		manager.integrate(s);

		// SET THE UNPERTURBED ACTUATOR FORCES
		perturbation->setUnperturbedForceSplineSet(actuation->getForceStorage());
        _model->setPerturbForcesEnabled(true);

        // TURN ON CORRECTIVE SPRINGS
        if(_body1Lin)  _body1Lin->setOn(true);
  		if(_body1Tor)  _body1Tor->setOn(true);
  		if(_body2Lin)  _body2Lin->setOn(true);
  		if(_body2Tor)  _body2Tor->setOn(true);

        // restore state to initial conditions
        _yStore->getData(index, s.getNY(), &s.updY()[0]);

		if(Check) {
			double dx = 1.0e-4;
			GCVSplineSet *forceSplines = perturbation->getUnperturbedForceSplineSet();
			Storage *forceStore = forceSplines->constructStorage(0,dx);
			forceStore->print("unperturbedForcesFromSplines.sto");
			delete forceStore;
		}

		// INTEGRATE (2)
		// This integration is to record the kinematics after the corrective
		// springs have been added and the actuator forces are applied via the splines.
		cout<<"\n\nUnperturbed integration (2) from "<<tiPert<<" to "<<tfPert;
		cout<<" to record the unperturbed kinematics after corrective springs has been";
		cout<<" added and the perturbation analysis has been turned on."<<endl;
		perturbation->setOn(true);
		_yStore->getData(index, s.getNY(), &s.updY()[0]);
		manager.integrate(s);

		// record unperturbed accelerations:
		Array<double> unperturbedAccel(0.0, nvalues);
		if(kin) {
			const Array<double> &posStart = kin->getPositionStorage()->getStateVector(0)->getData();
			const Array<double> &velStart = kin->getVelocityStorage()->getStateVector(0)->getData();
			const Array<double> &posEnd = kin->getPositionStorage()->getLastStateVector()->getData();
			double dt=kin->getPositionStorage()->getLastTime() - kin->getPositionStorage()->getFirstTime();
			if(dt>1e-8) for(int i=0;i<ncoords;i++) unperturbedAccel[i] = 2*(posEnd[i]-posStart[i]-dt*velStart[i])/(dt*dt);
			const Array<double> &unperturbedCoordinates = kin->getPositionStorage()->getLastStateVector()->getData(); // at end of timestep
			for(int i=0;i<ncoords;i++) values_unperturbed[i] = unperturbedCoordinates[i];
			if(Check) kin->getPositionStorage()->print("unperturbedKinematics.sto");
		}
		if(bodyKin) {
			const Array<double> &posStart = bodyKin->getPositionStorage()->getStateVector(0)->getData();
			const Array<double> &velStart = bodyKin->getVelocityStorage()->getStateVector(0)->getData();
			const Array<double> &posEnd = bodyKin->getPositionStorage()->getLastStateVector()->getData();
			double dt=bodyKin->getPositionStorage()->getLastTime() - bodyKin->getPositionStorage()->getFirstTime();
			if(dt>1e-8) for(int i=0;i<nbodycoords;i++) unperturbedAccel[ncoords+i] = 2*(posEnd[i]-posStart[i]-dt*velStart[i])/(dt*dt);
			const Array<double> &unperturbedBodyCoordinates = bodyKin->getPositionStorage()->getLastStateVector()->getData(); // at end of timestep
			for(int i=0;i<nbodycoords;i++) values_unperturbed[ncoords+i] = unperturbedBodyCoordinates[i];
			if(Check) bodyKin->getPositionStorage()->print("unperturbedBodyKinematics.sto");
		}
		unperturbedAccelStorage.append(tiPert, nvalues, &unperturbedAccel[0]);
		char fileName[Object::NAME_LENGTH];
		sprintf(fileName,"%s/%s_unperturbedAccel_dt_%.3f_df_%.3lf.sto",getResultsDir().c_str(),getName().c_str(),_pertWindow,_pertDF);
		unperturbedAccelStorage.print(fileName);

		// If we are not perturbing any actuators/gravity, we must only be computing unperturbed accelerations...
		// so can skip rest of this loop body.
		if(nperturb==0) continue;

		// Unperturbed forces
		Array<double> &unperturbedForcesAll = actuation->getForceStorage()->getStateVector(0)->getData(); // at BEGINNING of timestep
		// Unperturbed forces for only the subset of actuators we care about
		Array<double> unperturbedForces(0,actuators.getSize());
		for(int m=0;m<actuators.getSize();m++) unperturbedForces[m]=unperturbedForcesAll[mapPerturbedActuatorsToAllActuators[m]];
		// include unperturbed gravity value if doing gravity perturbation
		if(_perturbGravity) unperturbedForces.append(original_gravity[gravity_axis]);

		// Loop over actuators/gravity to be perturbed
		for (int m=0;m<nperturb;m++)	{
			perturbation->reset(s); 
			string actuatorName;
			if(m<actuators.getSize()) {
				Actuator *act = actuators.get(m);
				actuatorName = act->getName();
				// Set up pertubation callback
				cout<<"\nPerturbation of muscle "<<act->getName()<<" ("<<m<<") in loop"<<endl;
				perturbation->setActuator(act); 
			    perturbation->setPerturbation(s, ActuatorPerturbationIndependent::DELTA,_pertDF);


                // restore state to intitial conditions 
		        _yStore->getData(index, s.getNY(), &s.updY()[0]);

		  	    // Integrate
			    manager.integrate(s);
				cout << "actuator:\t"<<m<<"\tunperturbed force:\t"<<unperturbedForces[m]<<endl;
			} else {
				cout<<"\nGravity perturbation"<<endl;
				actuatorName = "gravity";
				perturbation->setActuator(0); 
				Vec3 grav;
				grav = original_gravity;
				grav[gravity_axis] += _pertDF;

cout << "perturbed gravity = " << grav << endl;

				_model->setGravity(grav);

                s = _model->initSystem(); 
                delete integrator;
                integrator = new RungeKuttaMersonIntegrator(_model->getSystem());
                integrator->setInternalStepLimit(_maxSteps);
                integrator->setMaximumStepSize(_maxDT);
                integrator->setAccuracy(_errorTolerance);
                manager.setIntegrator( integrator );


                // restore state to intitial conditions 
    		    _yStore->getData(index, s.getNY(), &s.updY()[0]);
                _model->getSystem().realize(s, Stage::Position );
    
    			// Integrate
    			manager.integrate(s);
				cout << "gravity original:\t"<<unperturbedForces[m]<<endl;

				// undo gravity perturbation
                resetGravity = true;
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

			if(_outputDetailedResults) {
				if(kin) {
					sprintf(fileName,"%s/%s_detailed_actuator_%s_time_%.3f_Kinematics_q.sto",getResultsDir().c_str(),getName().c_str(),actuatorName.c_str(), tiPert);
					kin->getPositionStorage()->print(fileName);
				}
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

		time(&finishTime);
		elapsedTime = difftime(finishTime,iterationTime);
		iterationTime = finishTime;
		cout<<"Iteration finished (took " << elapsedTime << " seconds)\n";

	} // end time loop
	//***************************************************************************
	IO::chDir(saveWorkingDirectory);

	cout<<"\n\n=================================================================\n";
	cout<<"PerturbationTool finished\n";
	cout<<"=================================================================\n";
	localTime = localtime(&startTime);
	cout<<"Start time   = "<<asctime(localTime);
	time(&finishTime);
	localTime = localtime(&finishTime);
	cout<<"Finish time  = "<<asctime(localTime);
	elapsedTime = difftime(finishTime,startTime);
	cout<<"Elapsed time = "<<elapsedTime<<" seconds.\n";
	cout<<"================================================================\n\n\n";
  
    delete integrator;

	return true;
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
