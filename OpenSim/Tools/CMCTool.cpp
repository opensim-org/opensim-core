// CMCTool.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson, Eran Guendelman, Chand T. John
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
#include "CMCTool.h"
#include "AnalyzeTool.h"
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/VectorGCVSplineR1R3.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/DerivCallbackSet.h>
#include <OpenSim/Simulation/Model/IntegCallbackSet.h>
#include <OpenSim/Simulation/Model/ModelIntegrand.h>
#include <OpenSim/Simulation/Model/ModelIntegrandForActuators.h>
#include <OpenSim/Simulation/Model/VectorFunctionForActuators.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Actuators/ForceApplier.h>
#include <OpenSim/Actuators/TorqueApplier.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Analyses/InverseDynamics.h>
#include "ForwardTool.h"
#include <OpenSim/Common/DebugUtilities.h>
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
CMCTool::~CMCTool()
{
	delete _integrand;
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
CMCTool::CMCTool() :
	AbstractTool(),
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
	_useReflexes(_useReflexesProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool()),
	_initialTimeForCOMAdjustment(_initialTimeForCOMAdjustmentProp.getValueDbl()),
	_finalTimeForCOMAdjustment(_finalTimeForCOMAdjustmentProp.getValueDbl()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_outputModelFile(_outputModelFileProp.getValueStr()),
	_adjustKinematicsToReduceResiduals(_adjustKinematicsToReduceResidualsProp.getValueBool()),
	_verbose(_verboseProp.getValueBool())
{
	setType("CMCTool");
	setNull();
}
//_____________________________________________________________________________
/**
 * Construct from an XML property file.
 *
 * @param aFileName File name of the XML document.
 */
CMCTool::CMCTool(const string &aFileName, bool aLoadModel) :
	AbstractTool(aFileName, false),
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
	_useReflexes(_useReflexesProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool()),
	_initialTimeForCOMAdjustment(_initialTimeForCOMAdjustmentProp.getValueDbl()),
	_finalTimeForCOMAdjustment(_finalTimeForCOMAdjustmentProp.getValueDbl()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_outputModelFile(_outputModelFileProp.getValueStr()),
	_adjustKinematicsToReduceResiduals(_adjustKinematicsToReduceResidualsProp.getValueBool()),
	_verbose(_verboseProp.getValueBool())
{
	setType("CMCTool");
	setNull();
	updateFromXMLNode();
	if(aLoadModel) loadModel(aFileName, &_originalActuatorSet);
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Tools do not copy the Object's DOMnode
 * and XMLDocument.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for a Tool:
 *
 * 1) Construction based on XML file (@see Tool(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Tool(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document that is held in memory.  In this way the proper connection
 * between an object's node and the corresponding node within the XML
 * document is established.
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
CMCTool::
CMCTool(const CMCTool &aTool) :
	AbstractTool(aTool),
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
	_useReflexes(_useReflexesProp.getValueBool()),
	_useFastTarget(_useFastTargetProp.getValueBool()),
	_optimizerAlgorithm(_optimizerAlgorithmProp.getValueStr()),
	_optimizerDX(_optimizerDXProp.getValueDbl()),
	_convergenceCriterion(_convergenceCriterionProp.getValueDbl()),
	_maxIterations(_maxIterationsProp.getValueInt()),
	_printLevel(_printLevelProp.getValueInt()),
	_computeAverageResiduals(_computeAverageResidualsProp.getValueBool()),
	_adjustCOMToReduceResiduals(_adjustCOMToReduceResidualsProp.getValueBool()),
	_initialTimeForCOMAdjustment(_initialTimeForCOMAdjustmentProp.getValueDbl()),
	_finalTimeForCOMAdjustment(_finalTimeForCOMAdjustmentProp.getValueDbl()),
	_adjustedCOMBody(_adjustedCOMBodyProp.getValueStr()),
	_outputModelFile(_outputModelFileProp.getValueStr()),
	_adjustKinematicsToReduceResiduals(_adjustKinematicsToReduceResidualsProp.getValueBool()),
	_verbose(_verboseProp.getValueBool())
{
	setType("CMCTool");
	setNull();
	*this = aTool;
}

//_____________________________________________________________________________
/**
 * Virtual copy constructor.
 *
 * @return Copy of this object.
 */
Object* CMCTool::
copy() const
{
	CMCTool *object = new CMCTool(*this);
	return(object);
}

//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void CMCTool::
setNull()
{
	setupProperties();

	_desiredKinematicsFileName = "";
	_externalLoadsFileName = "";
	_externalLoadsModelKinematicsFileName = "";
	_externalLoadsBody1 = "";
	_externalLoadsBody2 = "";
	_taskSetFileName = "";
	_constraintsFileName = "";
	_rraControlsFileName = "";
	_lowpassCutoffFrequency = -1.0;
	_lowpassCutoffFrequencyForLoadKinematics = -1.0;
	_targetDT = 0.010;
	_useCurvatureFilter = false;
	_useFastTarget = true;
	_optimizerAlgorithm = "ipopt";
	_useReflexes = false;
	_optimizerDX = 1.0e-4;
	_convergenceCriterion = 1.0e-6;
	_maxIterations = 100;
	_printLevel = 0;
	_computeAverageResiduals = false;
	_adjustedCOMBody = "";
	_adjustCOMToReduceResiduals = false;
	_initialTimeForCOMAdjustment = -1;
	_finalTimeForCOMAdjustment = -1;
	_outputModelFile = "";
	_adjustKinematicsToReduceResiduals = true;
	_verbose = false;

	_integrand = NULL;
}
//_____________________________________________________________________________
/**
 * Give this object's properties their XML names and add them to the property
 * list held in class Object (@see OpenSim::Object).
 */
void CMCTool::setupProperties()
{
	string comment;

	comment = "Motion (.mot) or storage (.sto) file containing the desired kinematic trajectories.";
	_desiredKinematicsFileNameProp.setComment(comment);
	_desiredKinematicsFileNameProp.setName("desired_kinematics_file");
	_propertySet.append( &_desiredKinematicsFileNameProp );

	comment = "Motion file (.mot) or storage file (.sto) containing the external loads applied to the model.";
	_externalLoadsFileNameProp.setComment(comment);
	_externalLoadsFileNameProp.setName("external_loads_file");
	_propertySet.append( &_externalLoadsFileNameProp );

	comment = "Motion file (.mot) or storage file (.sto) containing the model kinematics corresponding to the external loads.";
	_externalLoadsModelKinematicsFileNameProp.setComment(comment);
	_externalLoadsModelKinematicsFileNameProp.setName("external_loads_model_kinematics_file");
	_propertySet.append( &_externalLoadsModelKinematicsFileNameProp );

	comment = "Name of the body to which the first set of external loads ";
	comment += "should be applied (e.g., the name of the right foot).";
	_externalLoadsBody1Prop.setComment(comment);
	_externalLoadsBody1Prop.setName("external_loads_body1");
	_propertySet.append( &_externalLoadsBody1Prop );

	comment = "Name of the body to which the second set of external loads ";
	comment += "should be applied (e.g., the name of the left foot).";
	_externalLoadsBody2Prop.setComment(comment);
	_externalLoadsBody2Prop.setName("external_loads_body2");
	_propertySet.append( &_externalLoadsBody2Prop );

	comment = "File containing the tracking tasks. Which coordinates are tracked and with what weights are specified here.";
	_taskSetFileNameProp.setComment(comment);
	_taskSetFileNameProp.setName("task_set_file");
	_propertySet.append( &_taskSetFileNameProp );

	comment = "File containing the constraints on the controls.";
	_constraintsFileNameProp.setComment(comment);
	_constraintsFileNameProp.setName("constraints_file");
	_propertySet.append( &_constraintsFileNameProp );

	comment = "File containing the controls output by RRA.";
	comment += " These can be used to place constraints on the residuals during CMC.";
	_rraControlsFileNameProp.setComment(comment);
	_rraControlsFileNameProp.setName("rra_controls_file");
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

	comment = "Time window over which the desired actuator forces are achieved. "
		       "Muscles forces cannot change instantaneously, so a finite time window must be allowed. "
				 "The recommended time window for RRA is about 0.001 sec, and for CMC is about 0.010 sec.";
	_targetDTProp.setComment(comment);
	_targetDTProp.setName("cmc_time_window");
	_propertySet.append( &_targetDTProp );

	comment = "Flag (true or false) indicating whether or not to use the curvature filter. "
				 "Setting this flag to true can reduce oscillations in the computed muscle excitations.";
	_useCurvatureFilterProp.setComment(comment);
	_useCurvatureFilterProp.setName("use_curvature_filter");
	_propertySet.append( &_useCurvatureFilterProp );

	comment = "Flag (true or false) indicating whether or not to use reflexes.  This is a hook ";
	comment += "for users wanting to modify controls based on additonal information.";
	_useReflexesProp.setComment(comment);
	_useReflexesProp.setName("use_reflexes");
	_propertySet.append( &_useReflexesProp );

	comment = "Flag (true or false) indicating whether to use the fast CMC optimization target. ";
	comment += "The fast target requires the desired accelerations to be met. ";
	comment += "The optimizer fails if the acclerations constraints cannot be ";
	comment += "met, so the fast target can be less robust.  The regular target ";
	comment += "does not require the acceleration constraints to be met; it ";
	comment += "meets them as well as it can, but it is slower and less accurate.";
	_useFastTargetProp.setComment(comment);
	_useFastTargetProp.setName("use_fast_optimization_target");
	_propertySet.append( &_useFastTargetProp );

	comment = "Preferred optimizer algorithm (currently support \"ipopt\" or \"cfsqp\", "
				 "the latter requiring the osimFSQP library.";
	_optimizerAlgorithmProp.setComment(comment);
	_optimizerAlgorithmProp.setName("optimizer_algorithm");
	_propertySet.append( &_optimizerAlgorithmProp );

	comment = "Perturbation size used by the optimizer to compute numerical derivatives. "
				 "A value between 1.0e-4 and 1.0e-8 is usually approprieate.";
	_optimizerDXProp.setComment(comment);
	_optimizerDXProp.setName("optimizer_derivative_dx");
	_propertySet.append( &_optimizerDXProp );

	comment = "Convergence criterion for the optimizer. The smaller this value, the deeper the convergence. "
				 "Decreasing this number can improve a solution, but will also likely increase computation time.";
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

	comment = "Flag (true or false) indicating whether or not to compute average residuals. "
				 "No actions are taken based on this flag other than printing the average residuals, "
				 "which can be useful for seeing if the solution is good.  Average residuals should be "
				 "be close to 0.0.  If not, there is likely problem in the experimental data, in the model, "
				 "or both.";
	_computeAverageResidualsProp.setComment(comment);
	_computeAverageResidualsProp.setName("compute_average_residuals");
	_propertySet.append( &_computeAverageResidualsProp );

	comment = "Flag (true or false) indicating whether or not to make an adjustment "
				 "in the center of mass of a body to reduced DC offsets in MX and MZ. "
				 "If true, a new model is writen out that has altered anthropometry.";
	_adjustCOMToReduceResidualsProp.setComment(comment);
	_adjustCOMToReduceResidualsProp.setName("adjust_com_to_reduce_residuals");
	_propertySet.append( &_adjustCOMToReduceResidualsProp );

	comment = "Initial time used when computing average residuals in order to adjust "
				 "the body's center of mass.  If both initial and final time are set to "
				 "-1 (their default value) then the main initial and final time settings will be used.";
	_initialTimeForCOMAdjustmentProp.setComment(comment);
	_initialTimeForCOMAdjustmentProp.setName("initial_time_for_com_adjustment");
	_propertySet.append( &_initialTimeForCOMAdjustmentProp );

	comment = "Final time used when computing average residuals in order to adjust "
				 "the body's center of mass.";
	_finalTimeForCOMAdjustmentProp.setComment(comment);
	_finalTimeForCOMAdjustmentProp.setName("final_time_for_com_adjustment");
	_propertySet.append( &_finalTimeForCOMAdjustmentProp );

	comment = "Name of the body whose center of mass is adjusted. "
				 "The heaviest segment in the model should normally be chosen. "
				 "For a gait model, the torso segment is usually the best choice.";
	_adjustedCOMBodyProp.setComment(comment);
	_adjustedCOMBodyProp.setName("adjusted_com_body");
	_propertySet.append( &_adjustedCOMBodyProp );

	comment = "Name of the output model file (.osim) containing adjustments to anthropometry "
				 "made to reduce average residuals. This file is written if the property "
				 "adjust_com_to_reduce_residuals is set to true. If a name is not specified, "
				 "the model is written out to a file called adjusted_model.osim.";
	_outputModelFileProp.setComment(comment);
	_outputModelFileProp.setName("output_model_file");
	_propertySet.append( &_outputModelFileProp );

	comment = "Flag (true or false) indicating whether or not to adjust the kinematics "
			    "in order to reduce residuals.  Set this flag to false, and set "
				 ""+_adjustCOMToReduceResidualsProp.getName()+" on in order to only adjust the "
				 "model's center of mass without adjusting kinematics";
	_adjustKinematicsToReduceResidualsProp.setComment(comment);
	_adjustKinematicsToReduceResidualsProp.setName("adjust_kinematics_to_reduce_residuals");
	_propertySet.append( &_adjustKinematicsToReduceResidualsProp );

	comment = "True-false flag indicating whether or not to turn on verbose printing for cmc.";
	_verboseProp.setComment(comment);
	_verboseProp.setName("use_verbose_printing");
	_propertySet.append( &_verboseProp );
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
CMCTool& CMCTool::
operator=(const CMCTool &aTool)
{
	// BASE CLASS
	AbstractTool::operator=(aTool);

	// MEMEBER VARIABLES
	_desiredKinematicsFileName = aTool._desiredKinematicsFileName;
	_externalLoadsFileName = aTool._externalLoadsFileName;
	_externalLoadsModelKinematicsFileName = aTool._externalLoadsModelKinematicsFileName;
	_externalLoadsBody1 = aTool._externalLoadsBody1;
	_externalLoadsBody2 = aTool._externalLoadsBody2;
	_taskSetFileName = aTool._taskSetFileName;
	_constraintsFileName = aTool._constraintsFileName;
	_rraControlsFileName = aTool._rraControlsFileName;
	_lowpassCutoffFrequency = aTool._lowpassCutoffFrequency;
	_lowpassCutoffFrequencyForLoadKinematics = aTool._lowpassCutoffFrequencyForLoadKinematics;
	_targetDT = aTool._targetDT;
	_useCurvatureFilter = aTool._useCurvatureFilter;
	_optimizerDX = aTool._optimizerDX;
	_convergenceCriterion = aTool._convergenceCriterion;
	_useFastTarget = aTool._useFastTarget;
	_optimizerAlgorithm = aTool._optimizerAlgorithm;
	_useReflexes = aTool._useReflexes;
	_maxIterations = aTool._maxIterations;
	_printLevel = aTool._printLevel;
	_adjustedCOMBody = aTool._adjustedCOMBody;
	_outputModelFile = aTool._outputModelFile;
	_adjustKinematicsToReduceResiduals = aTool._adjustKinematicsToReduceResiduals;
	_computeAverageResiduals = aTool._computeAverageResiduals;
	_adjustCOMToReduceResiduals = aTool._adjustCOMToReduceResiduals;
	_initialTimeForCOMAdjustment = aTool._initialTimeForCOMAdjustment;
	_finalTimeForCOMAdjustment = aTool._finalTimeForCOMAdjustment;
	_verbose = aTool._verbose;

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
bool CMCTool::run()
{
	cout<<"Running tool "<<getName()<<".\n";

	// CHECK FOR A MODEL
	if(_model==NULL) {
		string msg = "ERROR- A model has not been set.";
		cout<<endl<<msg<<endl;
		throw(Exception(msg,__FILE__,__LINE__));
	}

	// OUTPUT DIRECTORY
	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory
	string saveWorkingDirectory = IO::getCwd();
	string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
	IO::chDir(directoryOfSetupFile);

	try {

	// SET OUTPUT PRECISION
	IO::SetPrecision(_outputPrecision);

	// USE PIPELINE ACTUATORS?
	_model->printDetailedInfo(cout);

	// CHECK PROPERTIES FOR ERRORS/INCONSISTENCIES
	if(_adjustCOMToReduceResiduals) {
		if(_adjustedCOMBody == "")
			throw Exception("CMCTool: ERROR- "+_adjustCOMToReduceResidualsProp.getName()+" set to true but "+
								 _adjustedCOMBodyProp.getName()+" not set",__FILE__,__LINE__);
		else if(!_model->getDynamicsEngine().getBodySet()->get(_adjustedCOMBody))
			throw Exception("CMCTool: ERROR- Body '"+_adjustedCOMBody+"' specified in "+
								 _adjustedCOMBodyProp.getName()+" not found",__FILE__,__LINE__);
	}

	// ASSIGN NUMBERS OF THINGS
	int i;
	int ny = _model->getNumStates();
	int nq = _model->getNumCoordinates();
	int nu = _model->getNumSpeeds();
	int na = _model->getNumActuators();


	// ---- INPUT ----
	// DESIRED KINEMATICS
	if(_desiredKinematicsFileName=="") {
		cout<<"ERROR- a desired kinematics file was not specified.\n\n";
		IO::chDir(saveWorkingDirectory);
		return false;
	}
	cout<<"\n\nLoading desired kinematics from file "<<_desiredKinematicsFileName<<" ...\n";
	Storage desiredKinStore(_desiredKinematicsFileName);

	// ---- INITIAL AND FINAL TIME ----
	// NOTE: important to do this before padding (for filtering)
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

	// Filter
	// Eran: important to filter *before* calling formCompleteStorages because we need the
	// constrained coordinates (e.g. tibia-patella joint angle) to be consistent with the
	// filtered trajectories
	desiredKinStore.pad(60);
	desiredKinStore.print("desiredKinematics_padded.sto");
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
	Storage *uStore=NULL;
	_model->getDynamicsEngine().formCompleteStorages(desiredKinStore,qStore,uStore);
	_model->getDynamicsEngine().convertDegreesToRadians(*qStore);
	_model->getDynamicsEngine().convertDegreesToRadians(*uStore);

	// GROUND REACTION FORCES
	ForwardTool::initializeExternalLoads(_model,_externalLoadsFileName,_externalLoadsModelKinematicsFileName,
		_externalLoadsBody1,_externalLoadsBody2,_lowpassCutoffFrequencyForLoadKinematics);

	// Adjust COM to reduce residuals (formerly RRA pass 1) if requested
	if(_adjustCOMToReduceResiduals) {
		adjustCOMToReduceResiduals(*qStore,*uStore);

		// If not adjusting kinematics, we don't proceed with CMC, and just stop here.
		if(!_adjustKinematicsToReduceResiduals) {
			cout << "No kinematics adjustment requested." << endl;
			delete qStore;
			delete uStore;
			writeAdjustedModel();
			IO::chDir(saveWorkingDirectory);
			return true;
		}
	}

	// Spline
	cout<<"\nConstructing function set for tracking...\n\n";
	GCVSplineSet qSet(5,qStore);
	delete qStore; qStore = NULL;

	delete uStore;
	uStore = qSet.constructStorage(1);
	GCVSplineSet uSet(5,uStore);
	delete uStore; uStore=NULL;

	// Print dudt for debugging
	Storage *dudtStore = qSet.constructStorage(2);
	dudtStore->print("desiredKinematics_splinefit_accelerations.sto");
	delete dudtStore; dudtStore=NULL;

	// ANALYSES
	addNecessaryAnalyses();

	// TASK SET
	if(_taskSetFileName=="") {
		cout<<"ERROR- a task set was not specified\n\n";
		IO::chDir(saveWorkingDirectory);
		return false;
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

	// Actuator force predictor
	// This requires the trajectories of the generalized coordinates
	// to be specified.
	string rraControlName;
	ModelIntegrandForActuators cmcIntegrand(_model);
	cmcIntegrand.setCoordinateTrajectories(&qSet);
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
	target->setDX(_optimizerDX);

	// Pick optimizer algorithm
	SimTK::OptimizerAlgorithm algorithm = SimTK::InteriorPoint;
	if(IO::Uppercase(_optimizerAlgorithm) == "CFSQP") {
		if(!SimTK::Optimizer::isAlgorithmAvailable(SimTK::CFSQP)) {
			std::cout << "CFSQP optimizer algorithm unavailable.  Will try to use IPOPT instead." << std::endl;
			algorithm = SimTK::InteriorPoint;
		} else {
			std::cout << "Using CFSQP optimizer algorithm." << std::endl;
			algorithm = SimTK::CFSQP;
		}
	} else if(IO::Uppercase(_optimizerAlgorithm) == "IPOPT") {
		std::cout << "Using IPOPT optimizer algorithm." << std::endl;
		algorithm = SimTK::InteriorPoint;
	} else {
		throw Exception("CMCTool: ERROR- Unrecognized optimizer algorithm: '"+_optimizerAlgorithm+"'",__FILE__,__LINE__);
	}

	SimTK::Optimizer *optimizer = new SimTK::Optimizer(*target, algorithm);
	controller.setOptimizationTarget(target, optimizer);

	cout<<"\nSetting optimizer print level to "<<_printLevel<<".\n";
	optimizer->setDiagnosticsLevel(_printLevel);
	cout<<"Setting optimizer convergence criterion to "<<_convergenceCriterion<<".\n";
	optimizer->setConvergenceTolerance(_convergenceCriterion);
	cout<<"Setting optimizer maximum iterations to "<<_maxIterations<<".\n";
	optimizer->setMaxIterations(_maxIterations);
	optimizer->useNumericalGradient(false); // Use our own central difference approximations
	optimizer->useNumericalJacobian(false);
	if(algorithm == SimTK::InteriorPoint) {
		// Some IPOPT-specific settings
		optimizer->setLimitedMemoryHistory(500); // works well for our small systems
		optimizer->setAdvancedBoolOption("warm_start",true);
		optimizer->setAdvancedRealOption("obj_scaling_factor",1);
		optimizer->setAdvancedRealOption("nlp_scaling_max_gradient",100);
	}

	if(_verbose) cout<<"\nSetting cmc controller to use verbose printing."<<endl;
	else cout<<"\nSetting cmc controller to not use verbose printing."<<endl;
	controller.setUseVerbosePrinting(_verbose);

	controller.setCheckTargetTime(true);

	// Reflexes
	controller.setUseReflexes(_useReflexes);

	// ---- SIMULATION ----
	// Manager
	delete _integrand;
	_integrand = new ModelIntegrand(_model);
	_integrand->setController(&controller);
	Manager manager(_integrand);
	manager.setSessionName(getName());
	manager.setInitialTime(_ti);
	manager.setFinalTime(_tf-_targetDT-rdMath::ZERO);

	// Initialize integrand controls using controls read in from file (which specify min/max control values)
	// and set the CMC integrand's control set to be a reference to the model integrand's control set.
	ControlSet *controlSet = _integrand->getControlSet();
	initializeControlSetUsingConstraints(rraControlSet,controlConstraints,controlSet);
	cmcIntegrand.setControlSetReference(*controlSet);

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
	if(_model->getActuatorSet()->getNumStates() > 0) {
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

	// ---- RESULTS -----
	double dt = 0.001;
	printResults(getName(),getResultsDir(),dt); // this will create results directory if necessary
	controlSet->print(getResultsDir() + "/" + getName() + "_controls.xml");
	_integrand->getControlStorage()->print(getResultsDir() + "/" + getName() + "_controls.sto");
	_integrand->getStateStorage()->print(getResultsDir() + "/" + getName() + "_states.sto");
	_integrand->getPseudoStateStorage()->print(getResultsDir() + "/" + getName() + "_pseudo.sto");

	Storage statesDegrees(*_integrand->getStateStorage());
	_model->getDynamicsEngine().convertRadiansToDegrees(statesDegrees);
	statesDegrees.setWriteSIMMHeader(true);
	statesDegrees.print(getResultsDir() + "/" + getName() + "_states_degrees.mot");

	controller.getPositionErrorStorage()->print(getResultsDir() + "/" + getName() + "_pErr.sto");

	Actuation *actuation = (Actuation*)_model->getAnalysisSet()->get("Actuation");
	if(_computeAverageResiduals && actuation) {
		Array<double> FAve(0.0,3),MAve(0.0,3);
		Storage *forceStore = actuation->getForceStorage();
		computeAverageResiduals(*forceStore,FAve,MAve);
		cout<<"\n\nAverage residuals:\n";
		cout<<"FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
		cout<<"MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl<<endl<<endl;

		// Write the average residuals (DC offsets) out to a file
		ofstream residualFile((getResultsDir() + "/" + getName() + "_avgResiduals.txt").c_str());
		residualFile << "Average Residuals:\n\n";
		residualFile << "FX average = " << FAve[0] << "\n";
		residualFile << "FY average = " << FAve[1] << "\n";
		residualFile << "FZ average = " << FAve[2] << "\n";
		residualFile << "MX average = " << MAve[0] << "\n";
		residualFile << "MY average = " << MAve[1] << "\n";
		residualFile << "MZ average = " << MAve[2] << "\n";
		residualFile.close();
	}

	// Write new model file
	if(_adjustCOMToReduceResiduals) writeAdjustedModel();

	} catch(Exception &x) {
		// TODO: eventually might want to allow writing of partial results
		x.print(cout);
		IO::chDir(saveWorkingDirectory);
		return false;
	}

	IO::chDir(saveWorkingDirectory);

	return true;
}


//=============================================================================
// UTILITY
//=============================================================================
void CMCTool::
writeAdjustedModel() 
{
	if(_outputModelFile=="") {
		cerr<<"Warning: A name for the output model was not set.\n";
		cerr<<"Specify a value for the property "<<_outputModelFileProp.getName();
		cerr<<" in the setup file.\n";
		cerr<<"Writing to adjusted_model.osim ...\n\n";
		_outputModelFile = "adjusted_model.osim";
	}

	// Set the model's actuator set back to the original set.  e.g. in RRA1
	// we load the model but replace its (muscle) actuators with torque actuators.
	// So we need to put back the muscles before writing out the adjusted model.
	// NOTE: use operator= so actuator groups are properly copied over
	*_model->getActuatorSet() = _originalActuatorSet;

	_model->print(_outputModelFile);
}
//_____________________________________________________________________________
/**
 * Compute the average residuals.
 *
 * @param rFAve The computed average force residuals.  The size of rFAve is
 * set to 3.
 * @param rMAve The computed average moment residuals.  The size of rMAve is
 * set to 3.
 */
void CMCTool::
computeAverageResiduals(const Storage &aForceStore,Array<double> &rFAve,Array<double> &rMAve)
{
	// COMPUTE AVERAGE
	int size = aForceStore.getSmallestNumberOfStates();
	Array<double> ave(0.0);
	ave.setSize(size);
	aForceStore.computeAverage(size,&ave[0]);

	// GET INDICES
	int iFX = aForceStore.getStateIndex("FX");
	int iFY = aForceStore.getStateIndex("FY");
	int iFZ = aForceStore.getStateIndex("FZ");
	int iMX = aForceStore.getStateIndex("MX");
	int iMY = aForceStore.getStateIndex("MY");
	int iMZ = aForceStore.getStateIndex("MZ");

	// GET AVE FORCES
	if(iFX>=0) rFAve[0] = ave[iFX];
	if(iFY>=0) rFAve[1] = ave[iFY];
	if(iFZ>=0) rFAve[2] = ave[iFZ];
	
	// GET AVE MOMENTS
	if(iMX>=0) rMAve[0] = ave[iMX];
	if(iMY>=0) rMAve[1] = ave[iMY];
	if(iMZ>=0) rMAve[2] = ave[iMZ];
}

void CMCTool::
adjustCOMToReduceResiduals(const Storage &qStore, const Storage &uStore)
{
	// Create a states storage from q's and u's
	Storage *statesStore = AnalyzeTool::createStatesStorageFromCoordinatesAndSpeeds(_model, &qStore, &uStore);

	double ti = _ti;
	double tf = _tf;
	if(_initialTimeForCOMAdjustment!=-1 || _finalTimeForCOMAdjustment!=-1) {
		ti = _initialTimeForCOMAdjustment;
		tf = _finalTimeForCOMAdjustment;
	}

	Array<double> FAve(0.0,3),MAve(0.0,3);

	double actualTi, actualTf;
	statesStore->getTime(statesStore->findIndex(ti),actualTi);
	statesStore->getTime(statesStore->findIndex(tf),actualTf);
	cout<<"\nNote: requested COM adjustment time range "<<ti<<" - "<<tf<<" clamped to nearest available data times "<<actualTi<<" - "<<actualTf<<endl;

	computeAverageResiduals(*_model, ti, tf, *statesStore, FAve, MAve);
	cout<<"Average residuals before adjusting "<<_adjustedCOMBody<<" COM:"<<endl;
	cout<<"FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
	cout<<"MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl<<endl;

	adjustCOMToReduceResiduals(FAve,MAve);

	computeAverageResiduals(*_model, ti, tf, *statesStore, FAve, MAve);
	cout<<"Average residuals after adjusting "<<_adjustedCOMBody<<" COM:"<<endl;
	cout<<"FX="<<FAve[0]<<" FY="<<FAve[1]<<" FZ="<<FAve[2]<<endl;
	cout<<"MX="<<MAve[0]<<" MY="<<MAve[1]<<" MZ="<<MAve[2]<<endl<<endl;

	delete statesStore;
}

// Uses an inverse dynamics analysis to compute average residuals
void CMCTool::
computeAverageResiduals(Model &aModel, double aTi, double aTf, const Storage &aStatesStore, Array<double>& rFAve, Array<double>& rMAve)
{
	// Turn off whatever's currently there (but remember whether it was on/off)
	AnalysisSet *analysisSet = aModel.getAnalysisSet();
	Array<bool> analysisSetOn = analysisSet->getOn();
	analysisSet->setOn(false);
	IntegCallbackSet *callbackSet = aModel.getIntegCallbackSet();
	Array<bool> callbackSetOn = callbackSet->getOn();
	callbackSet->setOn(false);

	// add inverse dynamics analysis
	InverseDynamics *inverseDynamics = new InverseDynamics();
	aModel.addAnalysis(inverseDynamics);

	int iInitial = aStatesStore.findIndex(aTi);
	int iFinal = aStatesStore.findIndex(aTf);
	aStatesStore.getTime(iInitial,aTi);
	aStatesStore.getTime(iFinal,aTf);

	cout << "\nComputing average residuals between " << aTi << " and " << aTf << endl;
	AnalyzeTool::run(aModel, iInitial, iFinal, aStatesStore, 0, 0, false);

	computeAverageResiduals(*inverseDynamics->getStorage(),rFAve,rMAve);

	aModel.removeAnalysis(inverseDynamics);

	// Turn off whatever's currently there
	analysisSet->setOn(analysisSetOn);
	callbackSet->setOn(callbackSetOn);
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
void CMCTool::
adjustCOMToReduceResiduals(const Array<double> &aFAve,const Array<double> &aMAve)
{
	// CHECK SIZE
	assert(aFAve.getSize()==3 && aMAve.getSize()==3);

	// GRAVITY
	double g[3];
	_model->getGravity(g);

	// COMPUTE SEGMENT WEIGHT
	AbstractBody *body = _model->getDynamicsEngine().getBodySet()->get(_adjustedCOMBody);
	double bodyMass = body->getMass();
	double bodyWeight = fabs(g[1])*bodyMass;
	if(bodyWeight<rdMath::ZERO) {
		cout<<"\nCMCTool.adjustCOMToReduceResiduals: ERR- ";
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

	cout<<"CMCTool.adjustCOMToReduceResiduals:\n";
	cout<<_adjustedCOMBody<<" weight = "<<bodyWeight<<"\n";
	cout<<"dx="<<dx<<", dz="<<dz<<endl;

	// GET EXISTING COM
	Array<double> com(0.0,3);
	body->getMassCenter(&com[0]);

	// COMPUTE ALTERED COM
	com[0] -= dx;
	com[2] -= dz;

	// ALTHER THE MODEL
	body->setMassCenter(&com[0]);

	//---- MASS CHANGE ----
	// Get recommended mass change.
	double dmass = aFAve[1] / g[1];
	cout<<"\ndmass = "<<dmass<<endl;
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
	cout<<"\nRecommended mass adjustments:"<<endl;
	for(i=0;i<nb;i++) {
		body = (*bodySet)[i];
		if(body==NULL) continue;
		massChange[i] = dmass * mass[i]/massTotal;
		massNew[i] = mass[i] + massChange[i];
		cout<<body->getName()<<":  orig mass = "<<mass[i]<<", new mass = "<<massNew[i]<<endl;
	}
}

//_____________________________________________________________________________
/**
 * Add Actuation/Kinematics analyses if necessary
 */
void CMCTool::
addNecessaryAnalyses()
{
	int stepInterval = 1;
	AnalysisSet *as = _model->getAnalysisSet();
	// Add Actuation if necessary
	Actuation *act = NULL;
	for(int i=0; i<as->getSize(); i++) 
		if(as->get(i)->getType() == "Actuation") { act = (Actuation*)as->get(i); break; }
	if(!act) {
		std::cout << "No Actuation analysis found in analysis set -- adding one" << std::endl;
		act = new Actuation(_model);
		act->setStepInterval(stepInterval);
		_model->addAnalysis(act);
	}
	// Add Kinematics if necessary
	// NOTE: also checks getPrintResultFiles() so that the Kinematics analysis added from the GUI does not count
	Kinematics *kin = NULL;
	for(int i=0; i<as->getSize(); i++) 
		if(as->get(i)->getType() == "Kinematics" && as->get(i)->getPrintResultFiles()) { kin = (Kinematics*)as->get(i); break; }
	if(!kin) {
		std::cout << "No Kinematics analysis found in analysis set -- adding one" << std::endl;
		kin = new Kinematics(_model);
		kin->setStepInterval(stepInterval);
		kin->getPositionStorage()->setWriteSIMMHeader(true);
		_model->addAnalysis(kin);
	} else {
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
void CMCTool::
initializeControlSetUsingConstraints(
	const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints,ControlSet *rControlSet)
{
	// Initialize control set with control constraints file
	int size = rControlSet->getSize();
	if(aControlConstraints) {
		for(int i=0;i<size;i++) {
			const Control *control = aControlConstraints->get(rControlSet->get(i)->getName());
			if(control)
				rControlSet->set(i,(Control*)control->copy());
		}
	}

	// FOR RESIDUAL CONTROLS, SET TO USE LINEAR INTERPOLATION
	if(aRRAControlSet!=NULL) {
		OPENSIM_FUNCTION_NOT_IMPLEMENTED();
		// Need to make sure code below still works after changes to controls/control constraints
#if 0
		size = aRRAControlSet->getSize();
		for(i=0;i<size;i++){
			string rraControlName = aRRAControlSet->get(i)->getName();
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
#endif
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
ControlSet* CMCTool::
constructRRAControlSet(ControlSet *aControlConstraints)
{
	if(_rraControlsFileName=="") return(NULL);
	
	OPENSIM_FUNCTION_NOT_IMPLEMENTED();
	// Need to make sure code below still works after changes to controls/control constraints
#if 0
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
#endif
}

//_____________________________________________________________________________
/**
 * Get a pointer to the Storage object holding forces. This's a utility routine used 
 * by the SimTrack GUI primarily to get access to residuals while running RRA.
 *
 * User-beware, the storage will get out of scope and potentially get deleted when the 
 * analysis is done, so no assumptions about the lifetime of the returned storage object 
 * outside the owning analysis should be made.
 */
Storage* CMCTool::getForceStorage(){
		Actuation *actuation = (Actuation*)_model->getAnalysisSet()->get("Actuation");
		if(actuation==NULL) return 0;
		return actuation->getForceStorage();
}
//_____________________________________________________________________________
/**
 */
Storage *CMCTool::
getStateStorage() 
{
	return _integrand ? _integrand->getStateStorage() : 0;
}

void CMCTool::setOriginalActuatorSet(const ActuatorSet &aActuatorSet) {
	_originalActuatorSet = aActuatorSet;
}
