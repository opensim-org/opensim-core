// AbstractTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
#include "AbstractTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Common/VectorFunction.h>
#include "PrescribedForce.h"
#include "ForceSet.h"
#include "BodySet.h"
#include "Model.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
#include <OpenSim/Simulation/Control/ControlSetController.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
AbstractTool::~AbstractTool()
{
	//if (_toolOwnsModel)
	//	delete _model;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
AbstractTool::AbstractTool():
	_modelFile(_modelFileProp.getValueStr()),
	_replaceForceSet(_replaceForceSetProp.getValueBool()),
	_forceSetFiles(_forceSetFilesProp.getValueStrArray()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_minDT(_minDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj()),
    _controllerSetProp(PropertyObj("Controllers", ControllerSet())),
    _controllerSet((ControllerSet&)_controllerSetProp.getValueObj()),
	_toolOwnsModel(true)
{
	setType("AbstractTool");
	setNull();
}

/**
 * Construct from file, and an optional GuiModel
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
AbstractTool::AbstractTool(const string &aFileName, bool aUpdateFromXMLNode):
	Object(aFileName, false),
	_modelFile(_modelFileProp.getValueStr()),
	_replaceForceSet(_replaceForceSetProp.getValueBool()),
	_forceSetFiles(_forceSetFilesProp.getValueStrArray()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_minDT(_minDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj()),
    _controllerSetProp(PropertyObj("Controllers", ControllerSet())),
    _controllerSet((ControllerSet&)_controllerSetProp.getValueObj()),
	_toolOwnsModel(true)
{
    _analysisSet.setMemoryOwner(false);
	setType("AbstractTool");
	setNull();
	if(aUpdateFromXMLNode) updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all SimulationTools only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for a AbstractTool:
 *
 * 1) Construction based on XML file (@see AbstractTool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by AbstractTool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * AbstractTool member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated AbstractTool member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the AbstractTool from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aTool Object to be copied.
 * @see AbstractTool(const XMLDocument *aDocument)
 * @see AbstractTool(const char *aFileName)
 * @see generateXMLDocument()
 */
AbstractTool::AbstractTool(const AbstractTool &aTool):
	Object(aTool),
	_modelFile(_modelFileProp.getValueStr()),
	_replaceForceSet(_replaceForceSetProp.getValueBool()),
	_forceSetFiles(_forceSetFilesProp.getValueStrArray()),
	_resultsDir(_resultsDirProp.getValueStr()),
	_outputPrecision(_outputPrecisionProp.getValueInt()),
	_ti(_tiProp.getValueDbl()),
	_tf(_tfProp.getValueDbl()),
	_solveForEquilibriumForAuxiliaryStates(_solveForEquilibriumForAuxiliaryStatesProp.getValueBool()),
	_maxSteps(_maxStepsProp.getValueInt()),
	_maxDT(_maxDTProp.getValueDbl()),
	_minDT(_minDTProp.getValueDbl()),
	_errorTolerance(_errorToleranceProp.getValueDbl()),
	_analysisSetProp(PropertyObj("Analyses",AnalysisSet())),
	_analysisSet((AnalysisSet&)_analysisSetProp.getValueObj()),
    _controllerSetProp(PropertyObj("Controllers", ControllerSet())),
    _controllerSet((ControllerSet&)_controllerSetProp.getValueObj()),
	_toolOwnsModel(true)
{
    _analysisSet.setMemoryOwner(false);
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void AbstractTool::
setNull()
{
	setupProperties();

	_model = NULL;
	_modelFile = "";
	_replaceForceSet = true;
	_resultsDir = "./";
	_outputPrecision = 20;
	_ti = 0.0;
	_tf = 1.0;
	_solveForEquilibriumForAuxiliaryStates = false;
	_maxSteps = 20000;
	_maxDT = 1.0;
	_minDT = 1.0e-8;
	_errorTolerance = 1.0e-3;
	_toolOwnsModel=true;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void AbstractTool::setupProperties()
{
	string comment;
	comment = "Name of the .osim file used to construct a model.";
	_modelFileProp.setComment(comment);
	_modelFileProp.setName("model_file");
	_propertySet.append( &_modelFileProp );


	comment = "Replace the model's force set with sets specified in <force_set_files>? "
				 "If false, the force set is appended to.";
	_replaceForceSetProp.setComment(comment);
	_replaceForceSetProp.setName("replace_force_set");
	_propertySet.append( &_replaceForceSetProp );

	comment = "List of xml files used to construct an force set for the model.";
	_forceSetFilesProp.setComment(comment);
	_forceSetFilesProp.setValue(Array<string>(""));
	_forceSetFilesProp.setName("force_set_files");
	_propertySet.append( &_forceSetFilesProp );

	comment = "Directory used for writing results.";
	_resultsDirProp.setComment(comment);
	_resultsDirProp.setName("results_directory");
	_propertySet.append( &_resultsDirProp );

	comment = "Output precision.  It is 20 by default.";
	_outputPrecisionProp.setComment(comment);
	_outputPrecisionProp.setName("output_precision");
	_propertySet.append( &_outputPrecisionProp );

	comment = "Initial time for the simulation.";
	_tiProp.setComment(comment);
	_tiProp.setName("initial_time");
	_propertySet.append( &_tiProp );

	comment = "Final time for the simulation.";
	_tfProp.setComment(comment);
	_tfProp.setName("final_time");
	_propertySet.append( &_tfProp );

	comment = "Flag indicating whether or not to compute equilibrium values for states other than the coordinates or speeds.  "
			    "For example, equilibrium muscle fiber lengths or muscle forces.";
	_solveForEquilibriumForAuxiliaryStatesProp.setComment(comment);
	_solveForEquilibriumForAuxiliaryStatesProp.setName("solve_for_equilibrium_for_auxiliary_states");
	_propertySet.append( &_solveForEquilibriumForAuxiliaryStatesProp );

	comment = "Maximum number of integrator steps.";
	_maxStepsProp.setComment(comment);
	_maxStepsProp.setName("maximum_number_of_integrator_steps");
	_propertySet.append( &_maxStepsProp );

	comment = "Maximum integration step size.";
	_maxDTProp.setComment(comment);
	_maxDTProp.setName("maximum_integrator_step_size");
	_propertySet.append( &_maxDTProp );

	comment = "Minimum integration step size.";
	_minDTProp.setComment(comment);
	_minDTProp.setName("minimum_integrator_step_size");
	_propertySet.append( &_minDTProp );

	comment = "Integrator error tolerance. When the error is greater, the integrator step size is decreased.";
	_errorToleranceProp.setComment(comment);
	_errorToleranceProp.setName("integrator_error_tolerance");
	_propertySet.append( &_errorToleranceProp );

	comment = "Set of analyses to be run during the investigation.";
	_analysisSetProp.setComment(comment);
	_analysisSetProp.setName("Analyses");
	_propertySet.append( &_analysisSetProp );

    _controllerSetProp.setComment("Controller objects in the model.");
    _controllerSetProp.setName("ControllerSet");
    _propertySet.append(&_controllerSetProp);

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
AbstractTool& AbstractTool::
operator=(const AbstractTool &aTool)
{
	// BASE CLASS
	Object::operator=(aTool);

	// MEMEBER VARIABLES
	_model = aTool._model;

	_modelFile = aTool._modelFile;
	_replaceForceSet = aTool._replaceForceSet;
	_forceSetFiles = aTool._forceSetFiles;
	_resultsDir = aTool._resultsDir;

	_outputPrecision = aTool._outputPrecision;
	_ti = aTool._ti;
	_tf = aTool._tf;
	_solveForEquilibriumForAuxiliaryStates = aTool._solveForEquilibriumForAuxiliaryStates;
	_maxSteps = aTool._maxSteps;
	_maxDT = aTool._maxDT;
	_minDT = aTool._minDT;
	_errorTolerance = aTool._errorTolerance;
	_analysisSet = aTool._analysisSet;
	_toolOwnsModel = aTool._toolOwnsModel;

    // CONTROLLER
    _controllerSet = aTool._controllerSet;
    
	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// MODEL
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the model to be investigated.
 * NOTE: setup() should have been called on the model prior to calling this method
 */
void AbstractTool::
setModel(Model& aModel)
{
	_model = &aModel;
	_toolOwnsModel=false;
    if(_model) {
       addAnalysisSetToModel();
       addControllerSetToModel();
    }
}
//_____________________________________________________________________________
/**
 * Get the model to be investigated.
 */
Model& AbstractTool::
getModel() const
{
	if (_model==NULL)
        throw Exception("AbstractTool::getModel(): Model has not been set");
	return(*_model);
}

//-----------------------------------------------------------------------------
// ANALYSIS SET
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the analysis set.
 */
AnalysisSet& AbstractTool::
getAnalysisSet() const
{
	return(_analysisSet);
}

//=============================================================================
// LOAD MODEL
//=============================================================================
//_____________________________________________________________________________
/**
 * Load and construct a model based on the property settings of
 * this investigation.
 */
void AbstractTool::
loadModel(const string &aToolSetupFileName, ForceSet *rOriginalForceSet )
{
	if (_modelFile != "") {
		string saveWorkingDirectory = IO::getCwd();
		string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
		IO::chDir(directoryOfSetupFile);

		cout<<"AbstractTool "<<getName()<<" loading model '"<<_modelFile<<"'"<<endl;

		Model *model = 0;

		try {
			model = new Model(_modelFile);
		} catch(...) { // Properly restore current directory if an exception is thrown
			IO::chDir(saveWorkingDirectory);
			throw;
		}

		IO::chDir(saveWorkingDirectory);

		// Append to or replace model forces with those (i.e. actuators) specified by the analysis
		updateModelForces(*model, aToolSetupFileName, rOriginalForceSet);

		setModel(*model);	

		setToolOwnsModel(true);
	}
}

void AbstractTool::
updateModelForces(Model& model, const string &aToolSetupFileName, ForceSet *rOriginalForceSet )
{
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aToolSetupFileName);
	IO::chDir(directoryOfSetupFile);

	try {
		if(rOriginalForceSet) *rOriginalForceSet = model.getForceSet();

		// If replacing force set read in from model file, clear it here
		if(_replaceForceSet) model.updForceSet().setSize(0);

		// Load force set(s)
		for(int i=0;i<_forceSetFiles.getSize();i++) {
			cout<<"Adding force object set from "<<_forceSetFiles[i]<<endl;
			ForceSet *forceSet=new ForceSet(model, _forceSetFiles[i]);
			model.updForceSet().append(*forceSet);
		}

	} catch (...) {
		IO::chDir(saveWorkingDirectory);
		throw;
	}

	IO::chDir(saveWorkingDirectory);
}
//_____________________________________________________________________________
/**
 * Adds Analysis objects from analysis set to model.
 *
 * NOTE: Makes copies of analyses.  Also, both this tool and the model have ownership of their analysis
 * objects, so making a copy is necessary so a single analysis won't be deleted twice.
 *
 * To avoid leaking when the tool is run from the GUI, pointers to the model's copy of the analyses
 * are kept around so that they can be removed at the end of tool execution.
 *  _analysisCopies is used to do this book keeping.
 */
void AbstractTool::
addAnalysisSetToModel()
{
	if (!_model) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	int size = _analysisSet.getSize();
	_analysisCopies.setMemoryOwner(false);
	for(int i=0;i<size;i++) {
		if(!&_analysisSet.get(i)) continue;
		Analysis *analysis = (Analysis*)_analysisSet.get(i).copy();
		_model->addAnalysis(analysis);
		_analysisCopies.append(analysis);
	}
}
//_____________________________________________________________________________
/**
 * Adds Controller objects from Controller set to model.
 *
 * NOTE: Makes copies of Controller.  Also, both this tool and the model have ownership of their Controller
 * objects, so making a copy is necessary so a single analysis won't be deleted twice.
 *
 * To avoid leaking when the tool is run from the GUI, pointers to the model's copy of the Controller
 * are kept around so that they can be removed at the end of tool execution.
 *  _controllerCopies is used to do this book keeping.
 */
void AbstractTool::
addControllerSetToModel()
{
	if (!_model) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	int size = _controllerSet.getSize();
	_controllerCopies.setMemoryOwner(false);
	for(int i=0;i<size;i++) {
		if(!&_controllerSet.get(i)) continue;
		Controller *controller = (Controller*)_controllerSet.get(i).copy();
		_model->addController(controller);
		_controllerCopies.append(controller);
	}
}
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Remove Analysis objects that were added earlier from analysis set to model.
 *
 * NOTE: Pointers to the .
 */
void AbstractTool::
removeAnalysisSetFromModel()
{
	if (!_model) {
		return;
	}

	int size = _analysisCopies.getSize();
	for(int i=size-1;i>=0;i--) {
		Analysis& analysis = (Analysis&)_analysisCopies.get(i);
		_model->removeAnalysis(&analysis);
	}
}
void AbstractTool::
removeControllerSetFromModel()
{
	if (!_model) {
		return;
	}

	int size = _controllerCopies.getSize();
	for(int i=size-1;i>=0;i--) {
		Controller& controller = (Controller&)_controllerCopies.get(i);
		_model->removeController(&controller);
	}
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
void AbstractTool::
printResults(const string &aBaseName,const string &aDir,double aDT,
				 const string &aExtension)
{
	cout<<"Printing results of investigation "<<getName()<<" to "<<aDir<<"."<<endl;
	IO::makeDir(aDir);
	_model->updAnalysisSet().printResults(aBaseName,aDir,aDT,aExtension);
}


bool AbstractTool::createExternalLoads( const string& aExternalLoadsFileName,
                                        const string& aExternalLoadsModelKinematicsFileName,
                                        Model& aModel) {

    if(aExternalLoadsFileName=="") {
        cout<<"No external loads will be applied (external loads file not specified)."<<endl;
        return false;
    }


	// CREATE FORCE AND TORQUE APPLIERS
	_externalForces = ForceSet(aModel, aExternalLoadsFileName);
	
	for (int i=0; i<_externalForces.getSize(); i++){
		aModel.addForce(&_externalForces.get(i));
	}
	_externalForces.setMemoryOwner(false);

	// LOAD MODEL KINEMATICS FOR EXTERNAL LOADS
	// To get the forces to be applied in the correct location, this file
	// should be from the IK solution, not from pass 2 of rra which alters
	// the kinematics.
	/*if(aExternalLoadsModelKinematicsFileName=="") {
		cout<<"\n\nERROR- a external loads kinematics file was not specified.\n\n";
		return false;
	}*/
    return(true);

}

void AbstractTool::
initializeExternalLoads( SimTK::State& s, 
                         const double& analysisStartTime, 
                         const double& analysisFinalTime,
						 Model& aModel, 
                         const string &aExternalLoadsFileName,
					     const string &aExternalLoadsModelKinematicsFileName,
					  	 double aLowpassCutoffFrequencyForLoadKinematics)
{
	bool externalLoadKinematicsSpecified = (aExternalLoadsModelKinematicsFileName!="");
	Storage *qStore=NULL;
	Storage *uStore=NULL;
	if (externalLoadKinematicsSpecified){
		cout<<"\n\nLoading external loads kinematics from file "<<aExternalLoadsModelKinematicsFileName<<" ...\n";
		Storage loadsKinStore(aExternalLoadsModelKinematicsFileName);

		// Form complete storage objects for the q's and u's
		// This means filling in unspecified generalized coordinates and
		// setting constrained coordinates to their valid values.
		Storage *uStoreTmp=NULL;
		aModel.getSimbodyEngine().formCompleteStorages(s, loadsKinStore,qStore,uStoreTmp);
		aModel.getSimbodyEngine().convertDegreesToRadians(*qStore);
		// Filter
		qStore->pad(qStore->getSize()/2); 
		if(aLowpassCutoffFrequencyForLoadKinematics>=0) {
			int order = 50;
			cout<<"Low-pass filtering external load kinematics with a cutoff frequency of "
				<<aLowpassCutoffFrequencyForLoadKinematics<<"..."<<endl;
			qStore->lowpassFIR(order, aLowpassCutoffFrequencyForLoadKinematics);
		} else {
			cout<<"Note- not filtering the external loads model kinematics."<<endl;
		}
		// Spline
		GCVSplineSet qSet(3,qStore);
		uStore = qSet.constructStorage(1);
	}
	// LOAD COP, FORCE, AND TORQUE
	Storage kineticsStore(_externalForces.getDataFileName());

	int copSize = kineticsStore.getSize();
	if(copSize<=0) return;
	// Make sure we have data for the requested range
	if (kineticsStore.getFirstTime() > analysisStartTime || 
		kineticsStore.getLastTime() < analysisFinalTime){
		char durationString[100];
		sprintf(durationString, "from t=%lf to t=%lf", analysisStartTime, analysisFinalTime);
		string msg = "ERR- Requested simulation time  extends outside provided data." + string(durationString);
		throw Exception(msg,__FILE__,__LINE__);
	}
	
	/* Due to earlier resampling and in general analysisStartTime, analysisFinalTime may not coincide with actual
	 * rows in qStore, in this case we need to evaluate the ForceApplier and TorqueApplier outside 
	 * analysis[start, final]time up to the time of the row before analysisStartTime, and the row after analysisFinalTime.
	 * Adjust analysisStartTime, analysisFinalTime to qStartTime, qFinalTime to account for that */
	if (externalLoadKinematicsSpecified){
		int startIndex = qStore->findIndex(analysisStartTime);
		double qStartTime, qFinalTime;
		qStore->getTime(startIndex, qStartTime);
		int finalIndex = qStore->findIndex(analysisFinalTime);
		qStore->getTime(finalIndex, qFinalTime);
		Array<double> analysisBoundTimes;
		if (qStartTime < analysisStartTime && startIndex >=1){
			analysisBoundTimes.append(analysisStartTime);
			//qStore->getTime(startIndex-1, qStartTime); 
			qStartTime = analysisStartTime;
		}
		if (qFinalTime < analysisFinalTime && finalIndex < qStore->getSize()-1){
			analysisBoundTimes.append(analysisFinalTime);
			//qStore->getTime(finalIndex+1, qFinalTime); 
			qFinalTime = analysisFinalTime;
		}
		qStore->interpolateAt(analysisBoundTimes);
	}
	// Construt point, force and torque functions from file
	computeFunctions(s, analysisStartTime, analysisFinalTime, kineticsStore, qStore, uStore);

}
//-----------------------------------------------------------------------------
// COMPUTE POSITION FUNCTIONS 
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the position and velocity functions that set the position and
 * velocity of the point at which the linear spring applies its force.
 * This method takes the time histories of a point's
 * position and velocity in the inertial frame and converts them to the local
 * (body) frame.
 *
 * @param aQStore Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 * @param aUStore Storage containing the time history of generalized
 * speeds for the model.  Note that all generalized speeds must
 * be specified and in radians.
 * 
 * @kineticsStore Storage containing the time history for forces.
 */
void AbstractTool::computeFunctions(SimTK::State& s, 
                                double startTime,
                                double endTime, 
								const Storage& kineticsStore, 
								Storage* aQStore, 
								Storage* aUStore)
{
	// If force is global but applied to non-ground body we need to transfrom to
	// correct frame. 
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int size = (aQStore!=NULL)?aQStore->getSize():0;
	int forceSize = kineticsStore.getSize();
	int startIndex=0;
	int lastIndex=size;
	if (aQStore != NULL){
		if (startTime!= -SimTK::Infinity){	// Start time was actually specified.
			startIndex = aQStore->findIndex(startTime); 
		}
		if (endTime!= SimTK::Infinity){	// Start time was actually specified.
			lastIndex = aQStore->findIndex(endTime);
		}
	}
	// Cycle thru forces to check if xform is needed
	for(int f=0; f <_externalForces.getSize(); f++){
		Force& nextForce = _externalForces.get(f);
		PrescribedForce* pf = dynamic_cast<PrescribedForce*>(&nextForce);
		if (pf){
			// Now we have a PrescribedForce
			// We can set ForceFunctions and TorqueFunctions directly
			if (pf->getPointFunctions().getSize()==3){
				if (pf->getPointIsInGlobalFrame()){
					double *t=0,*x=0,*y=0,*z=0;
					kineticsStore.getTimeColumn(t);
					const FunctionSet& ffSet=pf->getPointFunctions();
					kineticsStore.getDataColumn(ffSet[0].getName(), x);
					kineticsStore.getDataColumn(ffSet[1].getName(), y);
					kineticsStore.getDataColumn(ffSet[2].getName(), z);
					GCVSpline *localrtFx, *localrtFy, *localrtFz;
					localrtFx = new GCVSpline( 3, forceSize, t, x );
					localrtFy = new GCVSpline( 3, forceSize, t, y );
					localrtFz = new GCVSpline( 3, forceSize, t, z );
					pf->setPointFunctions( localrtFx, localrtFy, localrtFz  );
				}
				else{ // need to xform point to proper frame utilizing
					Array<double> localt(0.0,1);
					Array<double> localy(0.0,nq+nu);
					Vec3 originGlobal(0.0),origin(0.0);
					Vec3 pGlobal(0.0, 0.0, 0.0);
					Vec3 pLocal(0.0);
					Vec3 vGlobal(0.0),vLocal(0.0);
					Storage pStore,vStore;
					const FunctionSet& ffSet=pf->getPointFunctions();
					double *t=0,*x=0,*y=0,*z=0;
					VectorGCVSplineR1R3 *aGlobal;
					kineticsStore.getTimeColumn(t);
					kineticsStore.getDataColumn(ffSet[0].getName(),x);
					kineticsStore.getDataColumn(ffSet[1].getName(),y);
					kineticsStore.getDataColumn(ffSet[2].getName(),z);
					aGlobal = new VectorGCVSplineR1R3(3,forceSize,t,x,y,z);
					if (aQStore!= NULL){	// Kinematics specified
						for(int i=startIndex;i<lastIndex;i++) {
							// Set the model state
							aQStore->getTime(i,*(&localt[0]));
							aQStore->getData(i,nq,&localy[0]);
							aUStore->getData(i,nu,&localy[nq]);
							for (int j = 0; j < nq; j++) {
								Coordinate& coord = _model->getCoordinateSet().get(j);
								coord.setValue(s, localy[j], j==nq-1);
								coord.setSpeedValue(s, localy[nq+j]);
							}

							// Position in local frame (i.e. with respect to body's origin, not center of mass)
							_model->getSimbodyEngine().getPosition(s,pf->getBody(),origin,originGlobal);
							aGlobal->calcValue(&localt[0],&pGlobal[0], localt.getSize());
							pLocal=pGlobal-originGlobal; 
							_model->getSimbodyEngine().transform(s,_model->getSimbodyEngine().getGroundBody(),&pLocal[0],pf->getBody(),&pLocal[0]);
							pStore.append(localt[0],3,&pLocal[0]);
						}
					}
					else
						pStore = kineticsStore;
					// CREATE POSITION FUNCTION
					double *time=NULL;
					double *p0=0,*p1=0,*p2=0;
					int padSize = size / 4;
					if(padSize>100) padSize = 100;
					pStore.pad(padSize);
					size = pStore.getTimeColumn(time);
					pStore.getDataColumn(0,p0);
					pStore.getDataColumn(1,p1);
					pStore.getDataColumn(2,p2);
					GCVSpline *xFunc = new GCVSpline(3,size,time,p0);
					GCVSpline *yFunc = new GCVSpline(3,size,time,p1);
					GCVSpline *zFunc = new GCVSpline(3,size,time,p2);
					pf->setPointFunctions(xFunc, yFunc, zFunc);
					delete[] time;
					delete[] p0;
					delete[] p1;
					delete[] p2;

				}
			}
			if (pf->getForceFunctions().getSize()==3){
				double *t=0,*x=0,*y=0,*z=0;
				kineticsStore.getTimeColumn(t);
				const FunctionSet& ffSet=pf->getForceFunctions();
				kineticsStore.getDataColumn(ffSet[0].getName(), x);
				kineticsStore.getDataColumn(ffSet[1].getName(), y);
				kineticsStore.getDataColumn(ffSet[2].getName(), z);
				GCVSpline *localrtFx, *localrtFy, *localrtFz;
				localrtFx = new GCVSpline( 3, forceSize, t, x );
				localrtFy = new GCVSpline( 3, forceSize, t, y );
				localrtFz = new GCVSpline( 3, forceSize, t, z );
				pf->setForceFunctions( localrtFx, localrtFy, localrtFz  );
			}
			if (pf->getTorqueFunctions().getSize()==3){
				double *t=0,*x=0,*y=0,*z=0;
				kineticsStore.getTimeColumn(t);
				const FunctionSet& ffSet=pf->getTorqueFunctions();
				kineticsStore.getDataColumn(ffSet[0].getName(), x);
				kineticsStore.getDataColumn(ffSet[1].getName(), y);
				kineticsStore.getDataColumn(ffSet[2].getName(), z);
				GCVSpline *localrtFx, *localrtFy, *localrtFz;
				localrtFx = new GCVSpline( 3, forceSize, t, x );
				localrtFy = new GCVSpline( 3, forceSize, t, y );
				localrtFz = new GCVSpline( 3, forceSize, t, z );
				pf->setTorqueFunctions( localrtFx, localrtFy, localrtFz  );
			}
		}
	}
}

//_____________________________________________________________________________
/**
 * Compute the position and velocity functions that set the position and
 * velocity of the point at which the linear spring applies its force.
 * This method takes the time histories of a point's
 * position and velocity in the inertial frame and converts them to the local
 * (body) frame.
 *
 * @param aQStore Storage containing the time history of generalized
 * coordinates for the model. Note that all generalized coordinates must
 * be specified and in radians and Euler parameters.
 * @param aUStore Storage containing the time history of generalized
 * speeds for the model.  Note that all generalized speeds must
 * be specified and in radians.
 * @param aPStore Storage containing the time history of the position at
 * which the force is to be applied in the global frame.
 * DEPRECATED
 */
void AbstractTool::computePointFunctions(SimTK::State& s, 
                     double startTime,
                     double endTime, 
                     const OpenSim::Body& body,
                     const Storage& aQStore,
                     const Storage& aUStore,
                     VectorGCVSplineR1R3& aPGlobal,
                     GCVSpline*& xFunc, 
                     GCVSpline*& yFunc, 
                     GCVSpline*& zFunc)
{
	int i;
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int size = aQStore.getSize();
	Array<double> t(0.0,1);
	Array<double> y(0.0,nq+nu);
	Vec3 originGlobal(0.0),origin(0.0);
	Vec3 pGlobal(0.0, 0.0, 0.0);
	Vec3 pLocal(0.0);
	Vec3 vGlobal(0.0),vLocal(0.0);
	Storage pStore,vStore;
	int startIndex=0;
	int lastIndex=size;
	if (startTime!= -SimTK::Infinity){	// Start time was actually specified.
		startIndex = aQStore.findIndex(startTime); 
	}
	if (endTime!= SimTK::Infinity){	// Start time was actually specified.
		lastIndex = aQStore.findIndex(endTime);
	}
	for(i=startIndex;i<lastIndex;i++) {
		// Set the model state
		aQStore.getTime(i,*(&t[0]));
		aQStore.getData(i,nq,&y[0]);
		aUStore.getData(i,nu,&y[nq]);
        for (int j = 0; j < nq; j++) {
    		Coordinate& coord = _model->getCoordinateSet().get(j);
            coord.setValue(s, y[j], j==nq-1);
            coord.setSpeedValue(s, y[nq+j]);
        }

		// Position in local frame (i.e. with respect to body's origin, not center of mass)
		_model->getSimbodyEngine().getPosition(s,body,origin,originGlobal);
		aPGlobal.calcValue(&t[0],&pGlobal[0], t.getSize());
		pLocal=pGlobal-originGlobal; //Mtx::Subtract(1,3,&pGlobal[0],&originGlobal[0],&pLocal[0]);
		_model->getSimbodyEngine().transform(s,_model->getSimbodyEngine().getGroundBody(),&pLocal[0],body,&pLocal[0]);
		pStore.append(t[0],3,&pLocal[0]);
	}

	// CREATE POSITION FUNCTION
	double *time=NULL;
	double *p0=0,*p1=0,*p2=0;
	int padSize = size / 4;
	if(padSize>100) padSize = 100;
	pStore.pad(padSize);
	size = pStore.getTimeColumn(time);
	pStore.getDataColumn(0,p0);
	pStore.getDataColumn(1,p1);
	pStore.getDataColumn(2,p2);
	xFunc = new GCVSpline(3,size,time,p0);
	yFunc = new GCVSpline(3,size,time,p1);
	zFunc = new GCVSpline(3,size,time,p2);
	delete[] time;
	delete[] p0;
	delete[] p1;
	delete[] p2;

}
//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the tool to match current version
 */
/*virtual*/ void AbstractTool::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	std::string controlsFileName ="";
	if ( documentVersion < XMLDocument::getLatestVersion()){
		// Replace names of properties
		if (_node!=NULL && documentVersion<10900){
			renameChildNode("replace_actuator_set", "replace_force_set");
		}
		if (_node!=NULL && documentVersion<10904){
			renameChildNode("actuator_set_files", "force_set_files");
			// Get name of controls_file of any and use it to build a ControlSetController later
			DOMElement* controlsFileNode = XMLNode::GetFirstChildElementByTagName(_node,"controls_file");
			if (controlsFileNode){
				DOMText* txtNode=NULL;
				if(controlsFileNode && (txtNode=XMLNode::GetTextNode(controlsFileNode))) {
					// Could still be empty, whiteSpace or Unassigned
					string transcoded = XMLNode::TranscodeAndTrim(txtNode->getNodeValue());
					if (transcoded.length()>0){
						std::string value = XMLNode::GetValue<std::string>(controlsFileNode);
						controlsFileName = value;
					}
				}
				_node->removeChild(controlsFileNode);
			}
		}
		if (_node!=NULL && documentVersion<20001){
			// if external loads .mot file has been speified, create 
			// an XML file corresponding to it and set it as new external loads file
			string oldFile = parseStringProperty(std::string("external_loads_file"));
			if (oldFile!="" && oldFile!="Unassigned"){
				if (oldFile.substr(oldFile.length()-4, 4)!=".xml"){
					// get names of bodies for external loads and create an xml file for the forceSet 
					string body1, body2;
					body1 = parseStringProperty(std::string("external_loads_body1"));
					body2 = parseStringProperty(std::string("external_loads_body2"));
					string newFileName=createExternalLoadsFile(oldFile, body1, body2);
					DOMElement* pNode = XMLNode::GetFirstChildElementByTagName(_node,"external_loads_file");
					DOMText* txtNode=NULL;
					if(txtNode=XMLNode::GetTextNode(pNode)) {
						XMLCh* temp=XMLString::transcode(newFileName.c_str());
						txtNode->setNodeValue(temp);
						delete temp;
					}
				}
			}
		}
	}
	Object::updateFromXMLNode();
	// Create controllers and add them as needed.
	if (controlsFileName!="" && controlsFileName!="Unassigned"){
		// controls_file was specified, create a ControlSetController for it
		ControlSetController* csc = new ControlSetController();
		csc->setControlSetFileName(controlsFileName);
		_controllerSet.append(csc);
	}
}
void AbstractTool::loadQStorage (const std::string& statesFileName, Storage& rQStore) const {
	// Initial states
	if(statesFileName!="") {
		cout<<"\nLoading q's from file "<<statesFileName<<"."<<endl;
		Storage temp(statesFileName);
		_model->formQStorage(temp, rQStore);

		cout<<"Found "<<rQStore.getSize()<<" q's with time stamps ranging"<<endl;
		cout<<"from "<<rQStore.getFirstTime()<<" to "<<rQStore.getLastTime()<<"."<<endl;
	}
}
//_____________________________________________________________________________
/**
 *  Interfaces to build controller from a file and add it to the tool
 *
 */
std::string AbstractTool::getControlsFileName() const
{
	int numControllers = _controllerSet.getSize();
	if (numControllers>=1){	// WE have potentially more than one, make sure there's controlset controller
		for(int i=0; i<numControllers; i++){
			OpenSim::Controller& controller= _controllerSet.get(i);
			if (dynamic_cast<OpenSim::ControlSetController *>(&controller)==0)
				continue;
			OpenSim::ControlSetController& dController = (OpenSim::ControlSetController&) _controllerSet.get(i);
			return (dController.getControlSetFileName());
		}
	}
	return ("Unassigned");
}
//_____________________________________________________________________________
/**
 * A Convenience method to add a ControlSetController from a file and add it
 * to the model
 */
void AbstractTool::setControlsFileName(const std::string& controlsFilename)
{
	if (controlsFilename=="" || controlsFilename=="Unassigned") return;

	int numControllers = _controllerSet.getSize();

	for(int i=0; i<numControllers;i++){
		OpenSim::Controller& controller= _controllerSet.get(i);
		if (dynamic_cast<OpenSim::ControlSetController *>(&controller)==0)
			continue;
		OpenSim::ControlSetController& dController = (OpenSim::ControlSetController&)_controllerSet[i];
		dController.setControlSetFileName(controlsFilename);
		return;
	}
	// Create a new controlsetController and add it to the tool
	ControlSetController* csc = new ControlSetController();
	csc->setControlSetFileName(controlsFilename);
	_controllerSet.append(csc);
}
//_____________________________________________________________________________
/**
 * A Convenience method to verify that ColumnLabels are unique
 */
bool AbstractTool::verifyUniqueComulnLabels(const Storage& aStore) const
{
	const Array<string> lbls = aStore.getColumnLabels();
	bool isUnique = true;
	for(int i=0; i< lbls.getSize() && isUnique; i++){
		isUnique= (lbls.findIndex(lbls[i])==i);
	}
	return isUnique;
}

//_____________________________________________________________________________
/**
 * A Convenience method to get the next unique
 */
std::string AbstractTool::getNextAvailableForceName(const std::string prefix) const
{
	int candidate=0;
	char pad[3];
	std::string candidateName;
	bool found = false;
	while (!found) {
		candidate++;
		sprintf(pad, "%d", candidate);
		candidateName = prefix +"_"+string(pad);
		if (_model) {
			if (_model->getForceSet().contains(candidateName))
				continue;
		}
		found = !(_externalForces.contains(candidateName));
	};
	return candidateName;
}

std::string AbstractTool::parseStringProperty(const std::string& propertyName)
{
	std::string propValue="";
	DOMElement* pNode = XMLNode::GetFirstChildElementByTagName(_node,propertyName);
	//Get name of the file
	if (pNode != 0){
		DOMText* txtNode=NULL;
		if(txtNode=XMLNode::GetTextNode(pNode)) {
			// Could still be empty or whiteSpace
			string transcoded = XMLNode::TranscodeAndTrim(txtNode->getNodeValue());
			if (transcoded.length()>0){
				propValue = XMLNode::GetValue<std::string>(txtNode);
				
			}
		}
	}
	return propValue;
}

std::string AbstractTool::createExternalLoadsFile(const std::string& oldFile, 
										  const std::string& body1, 
										  const std::string& body2)
{
	ForceSet& fs=_externalForces;
	bool oldFileValid = !(oldFile=="" || oldFile=="Unassigned");

	std::string savedCwd;
	if(_document) {
		savedCwd = IO::getCwd();
		IO::chDir(IO::getParentDirectory(_document->getFileName()));
	}
	if (oldFileValid){
		if(!ifstream(oldFile.c_str(), ios_base::in).good()) {
			if(_document) IO::chDir(savedCwd);
			string msg =
				"Object: ERR- Could not open file " + oldFile+ ". It may not exist or you don't have permission to read it.";
			throw Exception(msg,__FILE__,__LINE__);
		}
	}
	Storage dataFile(oldFile);
	const Array<string>& labels=dataFile.getColumnLabels();
	bool body1Valid = !(body1=="" || body1=="Unassigned");
	bool body2Valid = !(body2=="" || body2=="Unassigned");
	std::string forceLabels[9] = {"ground_force_vx", "ground_force_vy", "ground_force_vz", 
		"ground_force_px", "ground_force_py", "ground_force_pz", 
		"ground_torque_x", "ground_torque_y", "ground_torque_z"};
	// We'll create upto 2 PrescribedForces
	if (body1Valid && body2Valid){
		// Find first occurance of ground_force_vx
		int indices[9][2];
		for(int i=0; i<9; i++){
			indices[i][0]= labels.findIndex(forceLabels[i]);
			if (indices[i][0]==-1){	// Something went wrong, abort here 
				if(_document) IO::chDir(savedCwd);
				string msg =
					"Object: ERR- Could not find label "+forceLabels[i]+ "in file " + oldFile+ ". Aborting.";
				throw Exception(msg,__FILE__,__LINE__);
			}
			for(int j=indices[i][0]+1; j<labels.getSize(); j++){
				if (labels[j]==forceLabels[i]){
					indices[i][1]=j;
					break;
				}
			}
		}
		for(int f=0; f<2; f++){
			PrescribedForce* pf = new PrescribedForce();
			pf->setBodyName((f==0)?body1:body2);
			char pad[3];
			sprintf(pad,"%d", f+1);
			std::string suffix = "ExternalForce_"+string(pad);
			pf->setName(suffix);
			// Create 9 new dummy GCVSplines and assign names
			GCVSpline** splines= new GCVSpline *[9];
			for(int func=0; func<9; func++){
				splines[func] = new GCVSpline();
				char columnNumber[5];
				sprintf(columnNumber, "#%d", indices[func][f]);
				splines[func]->setName(columnNumber);
			}
			pf->setForceFunctions(splines[0], splines[1], splines[2]);
			pf->setPointFunctions(splines[3], splines[4], splines[5]);
			pf->setTorqueFunctions(splines[6], splines[7], splines[8]);
			pf->setPointIsInGlobalFrame(false);
			pf->setForceIsInGlobalFrame(true);
			fs.append(pf);
		}
		fs.setDataFileName(oldFile);
		std::string newName=oldFile.substr(0, oldFile.length()-4)+".xml";
		fs.print(newName);
		if(_document) IO::chDir(savedCwd);
		cout<<"\n\n- Created ForceSet file " << newName << "to apply forces from " << oldFile << ".\n\n";
		return newName;
	}
	else {
			if(_document) IO::chDir(savedCwd);
			string msg =
				"Object: ERR- Only one body is specified in " + oldFile+ ".";
			throw Exception(msg,__FILE__,__LINE__);
	}
	if(_document) IO::chDir(savedCwd);
}
