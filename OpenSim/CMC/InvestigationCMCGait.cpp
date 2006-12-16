// InvestigationCMCGait.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson, Eran Guendelman
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include <time.h>
#include "InvestigationCMCGait.h"
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/GCVSplineSet.h>
#include <OpenSim/Tools/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Model/ModelIntegrandForActuators.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Analyses/ForceApplier.h>
#include <OpenSim/Analyses/TorqueApplier.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/InvestigationForward.h>
#include "rdCMC.h"
#include "rdCMC_TaskSet.h"
#include "rdActuatorForceTarget.h"
#include "rdActuatorForceTargetFast.h"

using namespace std;
using namespace OpenSim;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
InvestigationCMCGait::~InvestigationCMCGait()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
InvestigationCMCGait::InvestigationCMCGait() :
	Investigation(),
	_desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_taskSetFileName(_taskSetFileNameProp.getValueStr()),
	_constraintsFileName(_constraintsFileNameProp.getValueStr()),
	_rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_targetDT(_targetDTProp.getValueDbl()),
	_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_useReflexes(_useReflexesProp.getValueBool()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_includePipelineActuators(_includePipelineActuatorsProp.getValueBool()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool())
{
	setType("InvestigationCMCGait");
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct from an XML property file.
 *
 * @param aFileName File name of the XML document.
 */
InvestigationCMCGait::InvestigationCMCGait(const string &aFileName) :
	Investigation(aFileName),
	_desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_taskSetFileName(_taskSetFileNameProp.getValueStr()),
	_constraintsFileName(_constraintsFileNameProp.getValueStr()),
	_rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_targetDT(_targetDTProp.getValueDbl()),
	_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_useReflexes(_useReflexesProp.getValueBool()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_includePipelineActuators(_includePipelineActuatorsProp.getValueBool()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool())
{
	setType("InvestigationCMCGait");
	setNull();
	updateFromXMLNode();
	if(_model) addAnalysisSetToModel();
}
//_____________________________________________________________________________
/**
 * Construct from a DOMElement.  DOM is an acronym for 'Document Object Model'.
 * Once an XML file is read into memory, its content is contained in a DOM
 * object.  A CMC investigation can be constructed from one of these DOM
 * elements.
 *
 * @param aElement DOM element for the InvestigationCMCGait object.
 */
InvestigationCMCGait::InvestigationCMCGait(DOMElement *aElement) :
	Investigation(aElement),
	_desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_taskSetFileName(_taskSetFileNameProp.getValueStr()),
	_constraintsFileName(_constraintsFileNameProp.getValueStr()),
	_rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_targetDT(_targetDTProp.getValueDbl()),
	_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_useReflexes(_useReflexesProp.getValueBool()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_includePipelineActuators(_includePipelineActuatorsProp.getValueBool()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool())
{
	setType("InvestigationCMCGait");
	setNull();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Investigations do not copy the Object's DOMnode
 * and XMLDocument.  The reason for this is that for the
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
 * XML document that is held in memory.  In this way the proper connection
 * between an object's node and the corresponding node within the XML
 * document is established.
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
InvestigationCMCGait::
InvestigationCMCGait(const InvestigationCMCGait &aInvestigation) :
	Investigation(aInvestigation),
	_desiredKinematicsFileName(_desiredKinematicsFileNameProp.getValueStr()),
	_externalLoadsFileName(_externalLoadsFileNameProp.getValueStr()),
	_externalLoadsModelKinematicsFileName(_externalLoadsModelKinematicsFileNameProp.getValueStr()),
	_externalLoadsBody1(_externalLoadsBody1Prop.getValueStr()),
	_externalLoadsBody2(_externalLoadsBody2Prop.getValueStr()),
	_taskSetFileName(_taskSetFileNameProp.getValueStr()),
	_constraintsFileName(_constraintsFileNameProp.getValueStr()),
	_rraControlsFileName(_rraControlsFileNameProp.getValueStr()),
	_lowpassCutoffFrequency(_lowpassCutoffFrequencyProp.getValueDbl()),
	_lowpassCutoffFrequencyForLoadKinematics(_lowpassCutoffFrequencyForLoadKinematicsProp.getValueDbl()),
	_targetDT(_targetDTProp.getValueDbl()),
	_useCurvatureFilter(_useCurvatureFilterProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_useReflexes(_useReflexesProp.getValueBool()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_includePipelineActuators(_includePipelineActuatorsProp.getValueBool()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_adjustedCOMFileName(_adjustedCOMFileNameProp.getValueStr()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool())
{
	setType("InvestigationCMCGait");
	setNull();
	*this = aInvestigation;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 *
 * @return Copy of this object.
 */
Object* InvestigationCMCGait::
copy() const
{
	InvestigationCMCGait *object = new InvestigationCMCGait(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Virtual copy constructor from DOMElement.  This is the method that is
 * used to construct objects from an XML file.  This constructor creates
 * a copy of the object in all respects except that property values that
 * are specified in the DOMElement are used to over-write the values in
 * this object.
 *
 * @return Copy of this object with properties overriden by any properties
 * specified in the DOMElement.
 */
Object* InvestigationCMCGait::
copy(DOMElement *aElement) const
{
	InvestigationCMCGait *object = new InvestigationCMCGait(aElement);
	*object = *this;
	object->updateFromXMLNode();
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void InvestigationCMCGait::
setNull()
{
	setupProperties();

	_desiredKinematicsFileName = "";
	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = -1;
	_externalLoadsBody2 = -1;
	_taskSetFileName = "";
	_constraintsFileName = "";
	_rraControlsFileName = "";
	_lowpassCutoffFrequency = -1.0;
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;
	_targetDT = 0.010;
	_useCurvatureFilter = false;
	_useFastTarget = true;
	_useReflexes = false;
	_optimizerDX = 1.0e-4;
	_convergenceCriterion = 1.0e-6;
	_maxIterations = 100;
	_printLevel = 0;
	_includePipelineActuators = true;
	_adjustedCOMBody = "";
	_adjustedCOMFileName = "";
	_computeAverageResiduals = false;
	_adjustCOMToReduceResiduals = false;
}
//_____________________________________________________________________________
/**
 * Give this object's properties their XML names and add them to the property
 * list held in class Object (@see OpenSim::Object).
 */
void InvestigationCMCGait::setupProperties()
{
	string comment;

	comment = "Name of the file containing the desired kinematic trajectories.";
	_desiredKinematicsFileNameProp.setComment(comment);
	_desiredKinematicsFileNameProp.setName("desired_kinematics_file_name");
	_propertySet.append( &_desiredKinematicsFileNameProp );

	comment = "Name of the file containing the external loads applied to the model.";
	_externalLoadsFileNameProp.setComment(comment);
	_externalLoadsFileNameProp.setName("external_loads_file_name");
	_propertySet.append( &_externalLoadsFileNameProp );

	comment = "Name of the file containing the model kinematics corresponding to the external loads.";
	_externalLoadsModelKinematicsFileNameProp.setComment(comment);
	_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file_name");
	_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

	comment = "Name of the body to which the first set of external loads ";
	comment += "should be applied (e.g., the body name for the right foot).";
	_externalLoadsBody1Prop.setComment(comment);
	_externalLoadsBody1Prop.setName("external_loads_body1");
	_propertySet.append( &_externalLoadsBody1Prop );

	comment = "Name of the body to which the second set of external loads ";
	comment += "should be applied (e.g., the body name for the left foot).";
	_externalLoadsBody2Prop.setComment(comment);
	_externalLoadsBody2Prop.setName("external_loads_body2");
	_propertySet.append( &_externalLoadsBody2Prop );

	comment = "Name of the file containing the tracking tasks.";
	_taskSetFileNameProp.setComment(comment);
	_taskSetFileNameProp.setName("task_set_file_name");
	_propertySet.append( &_taskSetFileNameProp );

	comment = "Name of the file containing the constraints on the controls.";
	_constraintsFileNameProp.setComment(comment);
	_constraintsFileNameProp.setName("constraints_file_name");
	_propertySet.append( &_constraintsFileNameProp );

	comment = "Name of the file containing the actuator controls output by RRA.";
	comment += " These are used to place constraints on the residuals.";
	_rraControlsFileNameProp.setComment(comment);
	_rraControlsFileNameProp.setName("rra_controls_file_name");
	_propertySet.append( &_rraControlsFileNameProp );

	comment = "Low-pass cut-off frequency for filtering the desired kinematics.";
	comment += " A negative value results in no filtering. The default value is -1.0, so no filtering.";
	_lowpassCutoffFrequencyProp.setComment(comment);
	_lowpassCutoffFrequencyProp.setName("lowpass_cutoff_frequency");
	_propertySet.append( &_lowpassCutoffFrequencyProp );

	comment = "Low-pass cut-off frequency for filtering the model kinematics corresponding ";
	comment += "to the external loads. A negative value results in no filtering. ";
	comment += "The default value is -1.0, so no filtering.";
	_lowpassCutoffFrequencyForLoadKinematicsProp.setComment(comment);
	_lowpassCutoffFrequencyForLoadKinematicsProp.setName("lowpass_cutoff_frequency_for_load_kinematics");
	_propertySet.append( &_lowpassCutoffFrequencyForLoadKinematicsProp );

	comment = "Time window over which the desired actuator forces are achieved.";
	_targetDTProp.setComment(comment);
	_targetDTProp.setName("cmc_time_window");
	_propertySet.append( &_targetDTProp );

	comment = "Flag indicating whether or not to use the curvature filter.";
	_useCurvatureFilterProp.setComment(comment);
	_useCurvatureFilterProp.setName("use_curvature_filter");
	_propertySet.append( &_useCurvatureFilterProp );

	comment = "Flag indicating whether or not to use reflexes.  This is a hook ";
	comment += "for folks wanting to modify controls based on additonal information.";
	_useReflexesProp.setComment(comment);
	_useReflexesProp.setName("use_reflexes");
	_propertySet.append( &_useReflexesProp );

	comment = "Flag indicating whether to use the fast CMC optimization target. ";
	comment += "The fast target requires the desired accelerations to be met ";
	comment += "within the tolerance set by the convergence criterion. ";
	comment += "The optimizer fails if the acclerations constraints cannot be ";
	comment += "met, so the fast target can be less robust.  The regular target ";
	comment += "does not require the acceleration constraints to be met; it ";
	comment += "meets them as well as it can, but it is slower and less accurate.";
	_useFastTargetProp.setComment(comment);
	_useFastTargetProp.setName("use_fast_optimization_target");
	_propertySet.append( &_useFastTargetProp );

	comment = "Perturbation size used by the optimizer to compute numerical derivatives.";
	_optimizerDXProp.setComment(comment);
	_optimizerDXProp.setName("optimizer_derivative_dx");
	_propertySet.append( &_optimizerDXProp );

	comment = "Convergence criterion for the optimizer.";
	_convergenceCriterionProp.setComment(comment);
	_convergenceCriterionProp.setName("optimizer_convergence_criterion");
	_propertySet.append( &_convergenceCriterionProp );

	comment = "Maximum number of iterations for the optimizer.";
	_maxIterationsProp.setComment(comment);
	_maxIterationsProp.setName("optimizer_max_iterations");
	_propertySet.append( &_maxIterationsProp );

	comment = "Print level for the optimizer, 0 - 3. 0=no printing, 3=detailed printing, 2=in between";
	_printLevelProp.setComment(comment);
	_printLevelProp.setName("optimizer_print_level");
	_propertySet.append( &_printLevelProp );

	comment = "Flag indicating whether or not to include SIMM Pipeline actuators.";
	_includePipelineActuatorsProp.setComment(comment);
	_includePipelineActuatorsProp.setName("include_pipeline_actuators");
	_propertySet.append( &_includePipelineActuatorsProp );

	comment = "Name of the body whose center of mass is adjusted.";
	_adjustedCOMBodyProp.setComment(comment);
	_adjustedCOMBodyProp.setName("adjusted_com_body");
	_propertySet.append( &_adjustedCOMBodyProp );

	comment = "Name of the file specifying a change to the center of mass of a body.";
	comment += " This adjustment is made to remove dc offset in the residuals.";
	_adjustedCOMFileNameProp.setComment(comment);
	_adjustedCOMFileNameProp.setName("adjusted_com_file_name");
	_propertySet.append( &_adjustedCOMFileNameProp );

	comment = "Flag indicating whether or not to compute average residuals.";
	_computeAverageResidualsProp.setComment(comment);
	_computeAverageResidualsProp.setName("compute_average_residuals");
	_propertySet.append( &_computeAverageResidualsProp );

	comment = "Flag indicating whether or not to make an adjustment ";
	comment += "in the center of mass of a body to reduced DC offsets in MX and MZ.";
	_adjustCOMToReduceResidualsProp.setComment(comment);
	_adjustCOMToReduceResidualsProp.setName("adjust_com_to_reduce_residuals");
	_propertySet.append( &_adjustCOMToReduceResidualsProp );
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
InvestigationCMCGait& InvestigationCMCGait::
operator=(const InvestigationCMCGait &aInvestigation)
{
	// BASE CLASS
	Investigation::operator=(aInvestigation);

	// MEMEBER VARIABLES
	_desiredKinematicsFileName = aInvestigation._desiredKinematicsFileName;
	_externalLoadsFileName = aInvestigation._externalLoadsFileName;
	_externalLoadsModelKinematicsFileName = aInvestigation._externalLoadsModelKinematicsFileName;
	_externalLoadsBody1 = aInvestigation._externalLoadsBody1;
	_externalLoadsBody2 = aInvestigation._externalLoadsBody2;
	_taskSetFileName = aInvestigation._taskSetFileName;
	_constraintsFileName = aInvestigation._constraintsFileName;
	_rraControlsFileName = aInvestigation._rraControlsFileName;
	_lowpassCutoffFrequency = aInvestigation._lowpassCutoffFrequency;
	_lowpassCutoffFrequencyForLoadKinematics = aInvestigation._lowpassCutoffFrequencyForLoadKinematics;
	_targetDT = aInvestigation._targetDT;
	_useCurvatureFilter = aInvestigation._useCurvatureFilter;
	_optimizerDX = aInvestigation._optimizerDX;
	_convergenceCriterion = aInvestigation._convergenceCriterion;
	_useFastTarget = aInvestigation._useFastTarget;
	_useReflexes = aInvestigation._useReflexes;
	_maxIterations = aInvestigation._maxIterations;
	_printLevel = aInvestigation._printLevel;
	_includePipelineActuators = aInvestigation._includePipelineActuators;
	_adjustedCOMBody = aInvestigation._adjustedCOMBody;
	_adjustedCOMFileName = aInvestigation._adjustedCOMFileName;
	_computeAverageResiduals = aInvestigation._computeAverageResiduals;
	_adjustCOMToReduceResiduals = aInvestigation._adjustCOMToReduceResiduals;

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
void InvestigationCMCGait::run()
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

	// USE PIPELINE ACTUATORS?
	//_model->setIncludePipelineActuators(_includePipelineActuators);  All actuators are now in the OpenSim model.
	_model->printDetailedInfo(cout);

	// ALTER COM ?
	if(_adjustedCOMFileName!="") {
		FILE *fpCOM = fopen(_adjustedCOMFileName.c_str(),"r");
		if(fpCOM!=NULL) {
			Array<double> com(0.0,3);
			fscanf(fpCOM,"%lf %lf %lf",&com[0],&com[1],&com[2]);
			AbstractBody *body = _model->getDynamicsEngine().getBodySet()->get(_adjustedCOMBody);
			cout<<"NOTE- altering center of mass for "<<_adjustedCOMBody<<endl;
			cout<<com<<endl<<endl;
			body->setMassCenter(&com[0]);
			fclose(fpCOM);
		}
	}

	// OUTPUT DIRECTORY
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory
	string aFileName = string(getDocument()->getFileName());
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	// ASSIGN NUMBERS OF THINGS
	int i;
	int nx = _model->getNumControls();
	int ny = _model->getNumStates();
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int na = _model->getNumActuators();
	int nb = _model->getNumBodies();


	// ---- INPUT ----
	// DESIRED KINEMATICS
	if(_desiredKinematicsFileName=="") {
		cout<<"ERROR- a desired kinematics file was not specified.\n\n";
		IO::chDir(saveWorkingDirectory);
		return;
	}
	cout<<"\n\nLoading desired kinematics from file "<<_desiredKinematicsFileName<<" ...\n";
	Storage desiredKinStore(_desiredKinematicsFileName);

	// Filter
	// Eran: important to filter *before* calling formCompleteStorages because we need the
	// constrained coordinates (e.g. tibia-patella joint angle) to be consistent with the
	// filtered trajectories
	desiredKinStore.pad(60);
	desiredKinStore.print("qStore_test.sto");
	if(_lowpassCutoffFrequency>=0) {
		int order = 50;
		cout<<"\n\nLow-pass filtering desired kinematics with a cutoff frequency of ";
		cout<<_lowpassCutoffFrequency<<"...\n\n";
		desiredKinStore.lowpassFIR(order,_lowpassCutoffFrequency);
	} else {
		cout<<"\n\nNote- not filtering the desired kinematics.\n\n";
	}

	// Form complete storage objects for the q's and u's
	// This means filling in unspecified generalized coordinates and
	// setting constrained coordinates to their valid values.
	Storage *qStore=NULL;
	Storage *uStoreTmp=NULL;
	_model->getDynamicsEngine().formCompleteStorages(desiredKinStore,qStore,uStoreTmp);
	_model->getDynamicsEngine().convertDegreesToRadians(qStore);

	// Spline
	cout<<"\nConstruction function set for tracking...\n\n";
	GCVSplineSet qSet(5,qStore);
	Storage *uStore = qSet.constructStorage(1);
	GCVSplineSet uSet(5,uStore);
	Storage *dudtStore = qSet.constructStorage(2);
	dudtStore->print("accelerations.sto");

	// CONVERT TO QUATERNIONS
	_model->getDynamicsEngine().convertAnglesToQuaternions(qStore);

	// GROUND REACTION FORCES
	InvestigationForward::initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics);

	// ANALYSES
	addNecessaryAnalyses();

	// TASK SET
	if(_taskSetFileName=="") {
		cout<<"ERROR- a task set was not specified\n\n";
		IO::chDir(saveWorkingDirectory);
		return;
	}
	rdCMC_TaskSet taskSet(_taskSetFileName);
	cout<<"\n\n taskSet size = "<<taskSet.getSize()<<endl<<endl;
	taskSet.setModel(_model);
	taskSet.setFunctions(qSet);

	// CONSTRAINTS ON THE CONTROLS
	ControlSet *controlConstraints = NULL;
	if(_constraintsFileName!="") {
		controlConstraints = new ControlSet(_constraintsFileName);
	}

	// RRA CONTROLS
	ControlSet *rraControlSet = constructRRAControlSet(controlConstraints);
		
	// ---- INITIAL AND FINAL TIME ----
	// Initial Time
	double ti = desiredKinStore.getFirstTime();
	if(_ti<ti) {
		cout<<"\nThe initial time set for the cmc run precedes the first time\n";
		cout<<"in the desired kinematics file "<<_desiredKinematicsFileName<<".\n";
		cout<<"Resetting the initial time from "<<_ti<<" to "<<ti<<".\n\n";
		_ti = ti;
	}
	// Final time
	double tf = desiredKinStore.getLastTime();
	if(_tf>tf) {
		cout<<"\n\nWARN- The final time set for the cmc run is past the last time stamp\n";
		cout<<"in the desired kinematics file "<<_desiredKinematicsFileName<<".\n";
		cout<<"Resetting the final time from "<<_tf<<" to "<<tf<<".\n\n";
		_tf = tf;
	}


	// ---- INITIAL STATES ----
	Array<double> yi(0.0,ny);
	_model->getInitialStates(&yi[0]);
	cout<<"Using the generalized coordinates specified in "<<_desiredKinematicsFileName;
	cout<<" to set the initial configuration.\n";
	Array<double> q(0.0,nq);
	Array<double> u(0.0,nu);
	qSet.evaluate(q,0,_ti);
	uSet.evaluate(u,0,_ti);
	for(i=0;i<nq;i++) yi[i] = q[i];
	for(i=0;i<nu;i++) yi[i+nq] = u[i];
	_model->setInitialStates(&yi[0]);


	// ---- CMC CONTROLLER ----
	// Controller
	rdCMC controller(_model,&taskSet);
	controller.setUseCurvatureFilter(_useCurvatureFilter);
	controller.setTargetDT(_targetDT);
	controller.setCheckTargetTime(true);
	controller.setControlConstraints(controlConstraints);

	// Actuator force predictor
	// This requires the trajectories of the generalized coordinates
	// to be specified.
	string rraControlName;
	ModelIntegrandForActuators cmcIntegrand(_model);
	cmcIntegrand.setCoordinateTrajectories(&qSet);
	ControlSet *rootSet = cmcIntegrand.getControlSet();
	if(_includePipelineActuators) setControlsToUseStepsExceptResiduals(rraControlSet,rootSet);
	VectorFunctionForActuators *predictor =
		new VectorFunctionForActuators(&cmcIntegrand);
	controller.setActuatorForcePredictor(predictor);

	// Optimization target
	rdOptimizationTarget *target = NULL;
	if(_useFastTarget) {
		target = new rdActuatorForceTargetFast(na,&controller);
	} else {
		target = new rdActuatorForceTarget(na,&controller);
	}
	controller.setOptimizationTarget(target);
	controller.setCheckTargetTime(true);
	target->setDX(_optimizerDX);

	// Reflexes
	controller.setUseReflexes(_useReflexes);

	// Optimizer settings
	rdFSQP *sqp = controller.getOptimizer();
	cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
	sqp->setPrintLevel(_printLevel);
	cout<<"Setting optimizer convergence criterion to "<<_convergenceCriterion<<".\n";
	sqp->setConvergenceCriterion(_convergenceCriterion);
	cout<<"Setting optimizer maximum iterations to "<<_maxIterations<<".\n";
	sqp->setMaxIterations(_maxIterations);
	

	// ---- SIMULATION ----
	// Manager
	ModelIntegrand integrand(_model);
	integrand.setController(&controller);
	Manager manager(&integrand);
	manager.setSessionName(getName());
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf-_targetDT-rdMath::ZERO);

	// Integrator settings
	IntegRKF *integ = manager.getIntegrator();
	integ->setMaximumNumberOfSteps(_maxSteps);
	integ->setMaxDT(_maxDT);
	integ->setTolerance(_errorTolerance);
	integ->setFineTolerance(_fineTolerance);

	// Initial auxilliary states
	time_t startTime,finishTime;
	struct tm *localTime;
	double elapsedTime;
	if(_includePipelineActuators) {
	cout<<"\n\n\n";
	cout<<"================================================================\n";
	cout<<"================================================================\n";
	cout<<"Computing initial values for muscles states (activation, length)\n";
	time(&startTime);
	localTime = localtime(&startTime);
	cout<<"Start time = "<<asctime(localTime);
	cout<<"================================================================";
	controller.computeInitialStates(_ti,&yi[0]);
	manager.setInitialTime(_ti);
	_model->setInitialStates(&yi[0]);
	time(&finishTime);
	cout<<endl;
	cout<<"-----------------------------------------------------------------\n";
	cout<<"Finished computing initial states:\n";
	cout<<"-----------------------------------------------------------------\n";
	cout<<yi<<endl;
	cout<<"=================================================================\n";
	localTime = localtime(&startTime);
	cout<<"Start time   = "<<asctime(localTime);
	localTime = localtime(&finishTime);
	cout<<"Finish time  = "<<asctime(localTime);
	elapsedTime = difftime(finishTime,startTime);
	cout<<"Elapsed time = "<<elapsedTime<<" seconds.\n";
	cout<<"=================================================================\n";
	}

	// Set controls to use steps.
	ControlSet *controlSet = integrand.getControlSet();
	if(_includePipelineActuators) setControlsToUseStepsExceptResiduals(rraControlSet,controlSet);


	// ---- INTEGRATE ----
	cout<<"\n\n\n";
	cout<<"================================================================\n";
	cout<<"================================================================\n";
	cout<<"Using CMC to track the specified kinematics\n";
	cout<<"Integrating from "<<_ti<<" to "<<_tf<<endl;
	time(&startTime);
	localTime = localtime(&startTime);
	cout<<"Start time = "<<asctime(localTime);
	cout<<"================================================================\n";
	manager.integrate();
	time(&finishTime);
	cout<<"----------------------------------------------------------------\n";
	cout<<"Finished tracking the specified kinematics\n";
	cout<<"=================================================================\n";
	localTime = localtime(&startTime);
	cout<<"Start time   = "<<asctime(localTime);
	localTime = localtime(&finishTime);
	cout<<"Finish time  = "<<asctime(localTime);
	elapsedTime = difftime(finishTime,startTime);
	cout<<"Elapsed time = "<<elapsedTime<<" seconds.\n";
	cout<<"================================================================\n\n\n";

	// ---- RESIDUAL COMPUTATIONS ----
	// Average
	Array<double> FAve(0.0,3),MAve(0.0,3);
	if(_adjustCOMToReduceResiduals || _computeAverageResiduals) {
		computeAverageResiduals(FAve,MAve);
		cout<<"\n\nAverage residuals:\n";
		cout<<"FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
		cout<<"MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl<<endl<<endl;
	}

	// Adjust center of mass
	if(_adjustCOMToReduceResiduals) {
		adjustCOMToReduceResiduals(FAve,MAve);
	}

	// ---- RESULTS -----
	double dt = 0.001;
	printResults(getName(),getResultsDir(),dt); // this will create results directory if necessary
	controlSet->print(getResultsDir() + "/" + getName() + "_controls.xml");
	Storage *xStore = integrand.getControlStorage();
	Storage *yStore = integrand.getStateStorage();
	Storage *ypStore = integrand.getPseudoStateStorage();
	xStore->print(getResultsDir() + "/" + getName() + "_controls.sto");
	yStore->print(getResultsDir() + "/" + getName() + "_states.sto");
	ypStore->print(getResultsDir() + "/" + getName() + "_pseudo.sto");
	controller.getPositionErrorStorage()->print(getResultsDir() + "/" + getName() + "_pErr.sto");

	IO::chDir(saveWorkingDirectory);
}


//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute the average residuals.
 *
 * @param rFAve The computed average force residuals.  The size of rFAve is
 * set to 3.
 * @param rMAve The computed average moment residuals.  The size of rMAve is
 * set to 3.
 */
void InvestigationCMCGait::
computeAverageResiduals(Array<double> &rFAve,Array<double> &rMAve)
{
	// GET FORCE STORAGE
	Actuation *actuation = (Actuation*)_model->getAnalysisSet()->get("Actuation");
	if(actuation==NULL) return;
	Storage *forceStore = actuation->getForceStorage();

	// COMPUTE AVERAGE
	int size = forceStore->getSmallestNumberOfStates();
	Array<double> ave(0.0);
	ave.setSize(size);
	forceStore->computeAverage(size,&ave[0]);

	// GET INDICES
	int iFX = forceStore->getColumnIndex("FX");
	int iFY = forceStore->getColumnIndex("FY");
	int iFZ = forceStore->getColumnIndex("FZ");
	int iMX = forceStore->getColumnIndex("MX");
	int iMY = forceStore->getColumnIndex("MY");
	int iMZ = forceStore->getColumnIndex("MZ");
	cout<<"Residual Indices:\n";
	cout<<"iFX="<<iFX<<" iFY="<<iFY<<" iFZ="<<iFZ<<endl;
	cout<<"iMX="<<iMX<<" iMY="<<iMY<<" iMZ="<<iMZ<<endl;

	// GET AVE FORCES
	if(iFX>=0) rFAve[0] = ave[iFX];
	if(iFY>=0) rFAve[1] = ave[iFY];
	if(iFZ>=0) rFAve[2] = ave[iFZ];
	
	// GET AVE MOMENTS
	if(iMX>=0) rMAve[0] = ave[iMX];
	if(iMY>=0) rMAve[1] = ave[iMY];
	if(iMZ>=0) rMAve[2] = ave[iMZ];
}
//_____________________________________________________________________________
/**
 * Adjust the center of mass to reduce any DC offsets in MX and MZ.
 *
 * @param aFAve The average residual forces.  The dimension of aFAve should
 * be 3.
 * @param aMAve The average residual moments.  The dimension of aMAve should
 * be 3.
 */
void InvestigationCMCGait::
adjustCOMToReduceResiduals(const Array<double> &aFAve,const Array<double> &aMAve)
{
	// CHECK SIZE
	if(aFAve.getSize()<3) {
		cout<<"InvestigationCMCGait.adjustedCOMToReduceResiduals: \n";
		cout<<"ERR- The size of aFAve should be at least 3.\n";
		return;
	}
	if(aMAve.getSize()<3) {
		cout<<"InvestigationCMCGait.adjustedCOMToReduceResiduals: \n";
		cout<<"ERR- The size of aMAve should be at least 3.\n";
		return;
	}

	// GRAVITY
	double g[3];
	_model->getGravity(g);

	// COMPUTE TORSO WEIGHT
	AbstractBody *body = _model->getDynamicsEngine().getBodySet()->get(_adjustedCOMBody);
	double bodyMass = body->getMass();
	double bodyWeight = abs(g[1])*bodyMass;
	if(bodyWeight<rdMath::ZERO) {
		cout<<"\nInvestigationCMCGait.adjustCOMToReduceResiduals: ERR- ";
		cout<<_adjustedCOMBody<<" has no weight.\n";
		return;
	}
	
	//---- COM CHANGE ----
	double limit = 0.100;
	double dx =  aMAve[2] / bodyWeight;
	double dz = -aMAve[0] / bodyWeight;
	if(dz > limit) dz = limit;
	if(dz < -limit) dz = -limit;
	if(dx > limit) dx = limit;
	if(dx < -limit) dx = -limit;

	cout<<"InvestigationCMCGait.adjustCOMToReduceResiduals:\n";
	cout<<_adjustedCOMBody<<" weight = "<<bodyWeight<<"\n";
	cout<<"dx="<<dx<<", dz="<<dz<<"\n\n";

	if(_adjustedCOMFileName=="") {
		cout<<"InvestigationCMCGait.adjustCOMToReduceResidulas: WARN-";
		cout<<" an adjusted COM was computed\n";
		cout<<"but no file name was specfied for writing the results to file.\n";
		cout<<"Using the file name rra_newCOM.txt.\n\n";
		_adjustedCOMFileName = "rra_newCOM.txt";
	}

	// GET EXISTING COM
	Array<double> com(0.0,3);
	body->getMassCenter(&com[0]);

	// COMPUTE ALTERED COM
	com[0] -= dx;
	com[2] -= dz;

	// SAVE TO FILE
	ofstream rraCOMFile;
	rraCOMFile.open(_adjustedCOMFileName.c_str());
	rraCOMFile<<com[0]<<" "<<com[1]<<" "<<com[2]<<endl;
	rraCOMFile.close();

	//---- MASS CHANGE ----
	// Get recommended mass change.
	double dmass = aFAve[1] / g[1];
	cout<<"\n\ndmass = "<<dmass<<endl;
	// Loop through bodies
	int i;
	int nb = _model->getNumBodies();
	double massTotal=0.0;
	Array<double> mass(0.0,nb),massChange(0.0,nb),massNew(0.0,nb);
	BodySet *bodySet = _model->getDynamicsEngine().getBodySet();
	nb = bodySet->getSize();
	for(i=0;i<nb;i++) {
		body = (*bodySet)[i];
		if(body==NULL) continue;
		mass[i] = body->getMass();
		massTotal += mass[i];
	}
	cout<<"\n\nRecommended mass adjustments:\n";
	for(i=0;i<nb;i++) {
		massChange[i] = dmass * mass[i]/massTotal;
		massNew[i] = mass[i] + massChange[i];
		cout<<body->getName()<<":  orig mass = "<<mass[i]<<", new mass = "<<massNew[i]<<endl;
	}

}

//_____________________________________________________________________________
/**
 * Add Actuation/Kinematics analyses if necessary
 */
void InvestigationCMCGait::
addNecessaryAnalyses()
{
	int stepInterval = 1;
	int index;
	if((index=_model->getAnalysisSet()->getIndex("Actuation"))==-1) {
		std::cout << "No Actuation analysis found in analysis set -- adding one" << std::endl;
		Actuation *actuation = new Actuation(_model);
		actuation->setStepInterval(stepInterval);
		_model->addAnalysis(actuation);
	}
	if((index=_model->getAnalysisSet()->getIndex("Kinematics"))==-1) {
		std::cout << "No Kinematics analysis found in analysis set -- adding one" << std::endl;
		Kinematics *kin = new Kinematics(_model);
		kin->setStepInterval(stepInterval);
		kin->getPositionStorage()->setWriteSIMMHeader(true);
		_model->addAnalysis(kin);
	} else {
		Kinematics *kin = (Kinematics*)_model->getAnalysisSet()->get(index);
		kin->getPositionStorage()->setWriteSIMMHeader(true);
	}
}

//_____________________________________________________________________________
/**
 * Set controls to use steps except for the residuals, which use linear
 * interpolation.  This method also sets the min and max values for
 * the controls.
 *
 * @param aRRAControlSet Controls that were output by an RRA pass.
 * @param rControlSet Controls for the current run of CMC.
 * @todo The residuals are assumed to be the first 6 actuators.  This
 * needs to be made more general.  And, the control bounds need to
 * be set elsewhere, preferably in a file.
 */
void InvestigationCMCGait::
setControlsToUseStepsExceptResiduals(
	const ControlSet *aRRAControlSet,ControlSet *rControlSet)
{
	// SET BOUNDS AND ALL TO USE STEPS
	int i;
	int size = rControlSet->getSize();
	for(i=0;i<size;i++) {
		ControlLinear *control = (ControlLinear*)rControlSet->get(i);
		if(i>=6) {
			control->setUseSteps(true);
		} else {
			control->setUseSteps(false);
		}
		control->getNodeArray().setSize(0);
		control->setControlValueMin(0.0,0.02);
		control->setControlValueMax(0.0,1.0);
		control->setDefaultParameterMin(0.02);
		control->setDefaultParameterMax(1.0);
	}

	// FOR RESIDUAL CONTROLS, SET TO USE LINEAR INTERPOLATION
	string rraControlName;
	if(aRRAControlSet!=NULL) {
		size = aRRAControlSet->getSize();
		for(i=0;i<size;i++){
			rraControlName = aRRAControlSet->get(i)->getName();
			ControlLinear *control;
			try {
				control = (ControlLinear*)rControlSet->get(rraControlName);
			} catch(Exception x) {
				continue;
			}
			if(control==NULL) continue;
			control->setUseSteps(false);
			cout<<"Set "<<rraControlName<<" to use linear interpolation.\n";
		}
	}	
}
//_____________________________________________________________________________
/**
 * Create a set of control constraints based on an RRA solution.
 * If RRA (Residual Reduction Algorithm) was run to compute or reduce the 
 * residuals as a preprocessing step, those residuals need to be applied 
 * during the CMC run.  They are applied by reading in the residuals computed
 * during RRA and then using these controls to place narrow constraints on 
 * the controls for the residual actuators active during the CMC run.
 *
 * @param aControlConstraints Constraints based on a previous RRA solution
 * to be used during this run of CMC.
 */
ControlSet* InvestigationCMCGait::
constructRRAControlSet(ControlSet *aControlConstraints)
{
	if(_rraControlsFileName=="") return(NULL);

	int i;
	ControlLinear *controlConstraint=NULL;
	string rraControlName,cmcControlName;
	ControlLinear *rraControl=NULL;

	// LOAD RRA CONTROLS
	ControlSet *rraControlSet=NULL;
	rraControlSet = new ControlSet(_rraControlsFileName);

	// Loop through controls looking for corresponding actuators
	int nrra = rraControlSet->getSize();
	for(i=0;i<nrra;i++) {
		rraControl = (ControlLinear*)rraControlSet->get(i);
		if(rraControl==NULL) continue;
		rraControlName = rraControl->getName();

		// Does control exist in the model?
		// TODO- We are not using this functionality at the moment,
		// so I'm going to comment this out.
		//int index = _model->getControlIndex(rraControlName);
		int index = 0;

		// Add a constraint based on the rra control 
		if(index>=0) {

			// Create control constraint set if necessary
			if(aControlConstraints==NULL) {
				aControlConstraints = new ControlSet();
				aControlConstraints->setName("ControlConstraints");
			}

			// Get control constraint
			controlConstraint = (ControlLinear*)aControlConstraints->get(rraControlName);
			// Control constraint already exists, so clear the existing nodes
			if(controlConstraint!=NULL) {
					controlConstraint->getNodeArray().setSize(0);
			// Make a new control constraint
			} else {
				controlConstraint = new ControlLinear();
				controlConstraint->setName(rraControlName);
				aControlConstraints->append(controlConstraint);
			}

			// Set max and min values
			ArrayPtrs<ControlLinearNode> &nodes = rraControl->getNodeArray();
			int j,nnodes = nodes.getSize();
			double t,x,max,min,dx;
			for(j=0;j<nnodes;j++) {
				t = nodes[j]->getTime();
				x = nodes[j]->getValue();
				dx = 1.00 * (nodes[j]->getMax() - nodes[j]->getMin());
				max = x + dx;
				min = x - dx;
				controlConstraint->setControlValue(t,x);
				controlConstraint->setControlValueMax(t,max);
				controlConstraint->setControlValueMin(t,min);
				//cout<<controlConstraint->getName()<<": t="<<t<<" min="<<min<<" x="<<x<<" max="<<max<<endl;
			}
		}
	}

	// Print out modified control constraints
	//if(aControlConstraints!=NULL) aControlConstraints->print("cmc_aControlConstraints_check.xml");

	return(rraControlSet);
}
