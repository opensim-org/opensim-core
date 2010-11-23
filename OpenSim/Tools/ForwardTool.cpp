// ForwardTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2006 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include "ForwardTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/InterruptedException.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Control/Controller.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/PrescribedForce.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "CorrectionController.h"

using namespace std;
using namespace SimTK;
using namespace OpenSim;


void extractConfiguration(int nq, int nu, const Storage &aYStore,Storage &rQStore,Storage &rUStore)
{
	int nqnu = nq+nu;

	// CONFIGURE
	// Names
	rQStore.setName("Coordinates");
	rUStore.setName("Speeds");
	// Description
	rQStore.setDescription(aYStore.getDescription());
	rUStore.setDescription(aYStore.getDescription());
	// Column labels
	OpenSim::Array<string> qLabels("",nq),uLabels("",nu);
	OpenSim::Array<string> labels = aYStore.getColumnLabels();
	for(int i=0;i<nq;i++) qLabels[i] = labels[i];
	for(int i=0;i<nu;i++) uLabels[i] = labels[nq+i];
	rQStore.setColumnLabels(qLabels);
	rUStore.setColumnLabels(uLabels);
	// Purge
	rQStore.purge();
	rUStore.purge();

	// LOOP THROUGH STATES
	int size = aYStore.getSize();
	StateVector *vector;
	double time;
	OpenSim::Array<double> data(0.0);
	OpenSim::Array<double> q(0.0,nq);
	for(int i=0;i<size;i++) {
		vector = aYStore.getStateVector(i);
		if(vector->getSize()<nqnu) continue;
		time = vector->getTime();
		data = vector->getData();
		rQStore.append(time,nq,&data[0]);
		rUStore.append(time,nu,&data[nq]);
	}
}


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

}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ForwardTool::ForwardTool() :
	AbstractTool(),
	_statesFileName(_statesFileNameProp.getValueStr()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_outputDetailedResults(_outputDetailedResultsProp.getValueBool()),
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
	_kLin(_kLinProp.getValueDblVec()),
	_bLin(_bLinProp.getValueDblVec()),
	_kTor(_kTorProp.getValueDblVec()),
	_bTor(_bTorProp.getValueDblVec())
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
	_statesFileName(_statesFileNameProp.getValueStr()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_outputDetailedResults(_outputDetailedResultsProp.getValueBool()),
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
	_kLin(_kLinProp.getValueDblVec()),
	_bLin(_bLinProp.getValueDblVec()),
	_kTor(_kTorProp.getValueDblVec()),
	_bTor(_bTorProp.getValueDblVec())
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
	_statesFileName(_statesFileNameProp.getValueStr()),
	_useSpecifiedDt(_useSpecifiedDtProp.getValueBool()),
	_outputDetailedResults(_outputDetailedResultsProp.getValueBool()),
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
	_kLin(_kLinProp.getValueDblVec()),
	_bLin(_bLinProp.getValueDblVec()),
	_kTor(_kTorProp.getValueDblVec()),
	_bTor(_bTorProp.getValueDblVec())
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
void ForwardTool::setNull()
{
	setupProperties();

	// BASIC
	_statesFileName = "";
	_useSpecifiedDt = false;
	_printResultFiles = true;

	_replaceForceSet = false;	// default should be false for Forward.

	// INTERNAL WORK VARIABLES
	_yStore = NULL;

	// Start parsing log as empty. 
	_parsingLog="";
    _body1Lin = 0;
    _body2Lin = 0;
    _body1Tor = 0;
    _body2Tor = 0;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ForwardTool::setupProperties()
{
	string comment;

	// BASIC

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
	_springTransitionEndForceProp.setName("linear_spring_transition_end_force");
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
	_statesFileName = aTool._statesFileName;
	_useSpecifiedDt = aTool._useSpecifiedDt;

	_outputDetailedResults = aTool._outputDetailedResults;


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

    bool externalLoads = createExternalLoads(_externalLoadsFileName, *_model);


    // Re create the system with forces above and Realize the topology
    SimTK::State& s = _model->initSystem();
    _model->getMultibodySystem().realize(s, Stage::Position );

	loadStatesStorage(_statesFileName, _yStore);

    // set the desired states for controllers  
    _model->updControllerSet().setDesiredStates( _yStore );

	// INITIAL AND FINAL TIMES AND STATES INDEX
	int startIndexForYStore = determineInitialTimeFromStatesStorage(_ti);


    if( externalLoads ) {
	    initializeExternalLoads( s, _ti, _tf);
     }


	// SETUP SIMULATION
	// Manager (now allocated on the heap so that getManager doesn't return stale pointer on stack
    RungeKuttaMersonIntegrator integrator(_model->getMultibodySystem());
    Manager manager(*_model, integrator);
    setManager( manager );
	manager.setSessionName(getName());
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);
    
	// Initialize integrator
	integrator.setInternalStepLimit(_maxSteps);
	integrator.setMaximumStepSize(_maxDT);
	integrator.setAccuracy(_errorTolerance);


	// integ->setFineTolerance(_fineTolerance); No equivalent in SimTK
	if(_useSpecifiedDt) InitializeSpecifiedTimeStepping(_yStore, manager);

	// SET INITIAL AND FINAL TIME
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf);

	// SET THE INITIAL STATES
	if(_yStore!=NULL) _yStore->getData(startIndexForYStore,s.getNY(),&s.updY()[0]);
	if(startIndexForYStore >= 0) {
		_yStore->getData(startIndexForYStore,s.getNY(),&s.updY()[0]);
	}

	// SOLVE FOR EQUILIBRIUM FOR AUXILIARY STATES (E.G., MUSCLE FIBER LENGTHS)
	if(_solveForEquilibriumForAuxiliaryStates) {
		_model->computeEquilibriumForAuxiliaryStates(s);  
		
	}


	bool completed = true;

	try {
		// INTEGRATE
        _model->printDetailedInfo(s, std::cout );

		cout<<"\n\nIntegrating from "<<_ti<<" to "<<_tf<<endl;
        manager.integrate(s);
	} catch (char *str) { // e.g. may get InterruptedException
        printf("ForwardTool::run() caught exception \n"  );
        cout << "Caught some other exception: " << str << "\n";
		completed = false;
	}

	// PRINT RESULTS
	string fileName;
	if(_printResultFiles) printResults();

	IO::chDir(saveWorkingDirectory);

	removeAnalysisSetFromModel();
	return completed;
}
//=============================================================================
// PRINT RESULTS. 
// This method needs to be private since it shouldn't be called except from 
// within tool::run() otherwise will crash because Manager is out of scope
//=============================================================================
void ForwardTool::printResults() 
{
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	AbstractTool::printResults(getName(),getResultsDir()); // this will create results directory if necessary
    if(_model) {
        _model->printControlStorage(getResultsDir() + "/" + getName() + "_controls.sto");
        getManager().getStateStorage().print(getResultsDir() + "/" + getName() + "_states.sto");

        Storage statesDegrees(getManager().getStateStorage());
        _model->getSimbodyEngine().convertRadiansToDegrees(statesDegrees);
        statesDegrees.setWriteSIMMHeader(true);
        statesDegrees.print(getResultsDir() + "/" + getName() + "_states_degrees.mot");
    }


	
	IO::chDir(saveWorkingDirectory);
}



//=============================================================================
// UTILITY
//=============================================================================
/**
 * Add corrective springs.
 */
// TODO: Remove unused input storage
void ForwardTool::addCorrectiveSprings(SimTK::State &s, const Storage* aUnused, const PrescribedForce *aBody1Force,const PrescribedForce *aBody2Force)
{
       if(_yStore!=NULL) {
             Storage qStore,uStore;
             extractConfiguration(_model->getNumCoordinates(), _model->getNumSpeeds(), *_yStore, qStore, uStore);
             Body *body1 = &_model->updBodySet().get(aBody1Force->getBodyName());
             Body *body2 = &_model->updBodySet().get(aBody2Force->getBodyName());

             // Body1 Linear
             if(_body1LinSpringActive) {
                  _body1Lin = addLinearCorrectiveSpring(s, qStore,uStore,*aBody1Force);
             }
             // Body1 Torsional
             if(_body1TorSpringActive) {
                  double tauOn = _tauBody1OnProp.getUseDefault() ? _tau : _tauBody1On;
                  double tauOff = _tauBody1OffProp.getUseDefault() ? _tau : _tauBody1Off;
                  _body1Tor = addTorsionalCorrectiveSpring(s, qStore,uStore,body1,tauOn,_body1TorSpringTimeOn,tauOff,_body1TorSpringTimeOff);
             }

             // Body2 Linear
             if(_body2LinSpringActive) {
                  _body2Lin = addLinearCorrectiveSpring(s, qStore,uStore,*aBody2Force);
             }
             // Body2 Torsional
             if(_body2TorSpringActive) {
                  double tauOn = _tauBody2OnProp.getUseDefault() ? _tau : _tauBody2On;
                  double tauOff = _tauBody2OffProp.getUseDefault() ? _tau : _tauBody2Off;
                  _body2Tor = addTorsionalCorrectiveSpring(s, qStore,uStore,body2,tauOn,_body2TorSpringTimeOn,tauOff,_body2TorSpringTimeOff);
             }
       }
}
//_____________________________________________________________________________
/**
 * Add a linear corrective spring.
 */
LinearSpring* ForwardTool::addLinearCorrectiveSpring(SimTK::State &s, const Storage &aQStore,const Storage &aUStore,const PrescribedForce &aAppliedForce)
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
		if (tScale > _tf)
			break;
		timeScale.append(tScale);
		Vec3 force = aAppliedForce.getForceAtTime(tScale);
		linearScale.append(Step(force.norm(),_springTransitionStartForce,_springTransitionEndForce));
	}
	GCVSpline *scaleSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&linearScale[0]);
	scaleSpline->setName("ScaleForLinearCorrectiveSpring");

	// Create linear spring
	LinearSpring *spring = new LinearSpring(aAppliedForce.getBody(), tiScale, tfScale);

	//Gymnastics to get the PrescribedForce point functions (3) into a VectorFunction for the spring
	Function *px=NULL, *py=NULL, *pz=NULL;
	const FunctionSet& pointFunctions=aAppliedForce.getPointFunctions();
	VectorGCVSplineR1R3 *pFunc = new VectorGCVSplineR1R3();
	pFunc->setSplineY0((GCVSpline *)&pointFunctions[0]); 
	pFunc->setSplineY1((GCVSpline *)&pointFunctions[1]);
	pFunc->setSplineY2((GCVSpline *)&pointFunctions[2]);
	
	spring->setPointFunction((VectorFunction*)pFunc);
	spring->computeTargetFunctions(s, aQStore,aUStore);
	spring->setKValue(_kLin);
	spring->setBValue(_bLin);
	spring->setThreshold(_forceThreshold);
	spring->setScaleFunction(scaleSpline);
	//if(_outputDetailedResults) spring->setRecordAppliedLoads(true);
	_model->addForce(spring);

	return(spring);
}
//_____________________________________________________________________________
/**
 * Add a torsional corrective spring.
 */
TorsionalSpring* ForwardTool::addTorsionalCorrectiveSpring(SimTK::State &s, const Storage &aQStore,const Storage &aUStore, 
														   OpenSim::Body *aBody,double aTauOn,double aTimeOn,double aTauOff,double aTimeOff)
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
		double value1 = SigmaUp(aTauOn,aTimeOn,tScale);
		double value2 = SigmaDn(aTauOff,aTimeOff,tScale);
		torsionalScale.append(value1+value2-1.0);
	}
	GCVSpline *scaleSpline = new GCVSpline(3,timeScale.getSize(),&timeScale[0],&torsionalScale[0]);
	scaleSpline->setName("ScaleForTorsionalCorrectiveSpring");

	TorsionalSpring *spring = new TorsionalSpring(*aBody, tiScale, tfScale);
	spring->computeTargetFunctions(s, aQStore, aUStore);
	spring->setKValue(_kTor);
	spring->setBValue(_bTor);
	spring->setThreshold(_torqueThreshold);
	spring->setScaleFunction(scaleSpline);
	//if(_outputDetailedResults) spring->setRecordAppliedLoads(true);
	_model->addForce(spring);

	return(spring);
}

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
int ForwardTool::determineInitialTimeFromStatesStorage(double &rTI)
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

void ForwardTool::loadStatesStorage (std::string& statesFileName, Storage*& rYStore) const {
	// Initial states
	rYStore = NULL;
	if(_statesFileName!="") {
		cout<<"\nLoading states from file "<<_statesFileName<<"."<<endl;
		Storage temp(statesFileName);
		rYStore = new Storage();
		_model->formStateStorage(temp, *rYStore);

		cout<<"Found "<<rYStore->getSize()<<" state vectors with time stamps ranging"<<endl;
		cout<<"from "<<rYStore->getFirstTime()<<" to "<<rYStore->getLastTime()<<"."<<endl;
	}
}

//_____________________________________________________________________________
/**
 * Setup time stepping so that the integrator follows a pre-specified series
 * of time steps.
 */
void ForwardTool::InitializeSpecifiedTimeStepping(Storage *aYStore, Manager& aManager)
{
	// USE INITIAL STATES FILE FOR TIME STEPS

	if(aYStore) {
		std::cout << "\nUsing dt specified from storage "<< aYStore->getName()<< std::endl;
		Array<double> tArray(0.0,aYStore->getSize());
		Array<double> dtArray(0.0,aYStore->getSize());
		aYStore->getTimeColumn(tArray);
		for(int i=0;i<aYStore->getSize()-1;i++) dtArray[i]=tArray[i+1]-tArray[i];
		aManager.setUseSpecifiedDT(true);
		aManager.setDTArray(aYStore->getSize()-1,&dtArray[0],tArray[0]);
		//std::cout << "ForwardTool.InitializeSpecifiedTimeStepping: " << tArray << endl;

	// NO AVAILABLE STATES FILE
	} else {
		std::cout << "WARNING: Ignoring 'use_specified_dt' property because no initial states file is specified" << std::endl;
	}
}


//=============================================================================
// CUBIC STEP FUNCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * A smooth step-down function using cubic polynomial.
 * x=0 for t<t0, x=1 for t>t1, and x=smooth step in between t0 and t1.
 *
 * @param t    Parameter at which to evaluate step function
 * @param t0   Parameter value at which step starts (result=0 to the left)
 * @param t1   Parameter value at which step ends (result=1 to the right)
 */
double ForwardTool::
Step(double t, double t0, double t1)
{
	double tn=(t-t0)/(t1-t0);
	if(tn<0) return 0;
	else if(tn>1) return 1;
	else return pow(tn,2)*(3-2*tn);
}
//=============================================================================
// EXPONENTIALS
//=============================================================================
//_____________________________________________________________________________
/**
 * A smooth step-up function using exponentials.
 *
 * @param tau  Rise and fall time constant.
 * @param to   Location of the midpoint of the step.
 * @param t    Independent variable
 */
double ForwardTool::
SigmaUp(double tau,double to,double t)
{
	return(  1.0 / (1.0 + exp(-(t-to)/tau)) );
}
//_____________________________________________________________________________
/**
 * A smooth step-down function using exponentials.
 *
 * @param tau  Rise and fall time constant.
 * @param to   Location of the midpoint of the step.
 * @param t    Independent variable
 */
double ForwardTool::
SigmaDn(double tau,double to,double t)
{
    return(1.0 -  1.0 / (1.0 + exp(-(t-to)/tau)) );
}
//_____________________________________________________________________________
/**
 *  Set the current integration manager
 *
 * @param m   pointer to itegration manager  
 */

void ForwardTool::setManager( Manager& m ) {
   _manager = &m;
}
//_____________________________________________________________________________
/**
 *  Get the current integration manager
 *
 */
const Manager& ForwardTool::getManager() const {
   return( *_manager);
}
//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the tool to match current version
 */
/*virtual*/ void ForwardTool::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	bool neededSprings=false;
	if ( documentVersion < XMLDocument::getLatestVersion()){
			// Now check if we need to create a correction controller to replace springs
		if (_node!=NULL && documentVersion<10904){
			std::string propNames[]={
				"body1_linear_corrective_spring_active",
				"body1_torsional_corrective_spring_active",
				"body2_linear_corrective_spring_active",
				"body2_torsional_corrective_spring_active"
			};
			int i=0;
			while (!neededSprings && i<4){
				neededSprings = (XMLNode::GetFirstChildElementByTagName(_node, propNames[i++])!=NULL);
			}
			AbstractTool::updateFromXMLNode();
			if (neededSprings){
				CorrectionController* cc = new CorrectionController();
				cc->setKp(16.0);
				cc->setKv(8.0);
				_controllerSet.append(cc);
				_parsingLog+= "This setup file contains corrective springs.\n";
				_parsingLog+= "Corrective springs are deprecated in OpenSim 2.0\n";
				_parsingLog+= "Instead, a Corrective Controller has been created.\n";

			}
		}
		else
			AbstractTool::updateFromXMLNode();
	}
	else
		AbstractTool::updateFromXMLNode();
}
