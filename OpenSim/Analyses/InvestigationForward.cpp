// InvestigationForward.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "InvestigationForward.h"
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
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
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr())
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
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr())
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
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr())
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
	_initialStatesFileName(_initialStatesFileNameProp.getValueStr())
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
}

