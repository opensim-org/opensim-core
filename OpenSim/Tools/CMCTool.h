// CMCTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contributors: Frank C. Anderson
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

//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifndef CMCTool_h__
#define CMCTool_h__

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Model/ActuatorSet.h>
#include <OpenSim/Simulation/Control/ControlSet.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

class ModelIntegrand;

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API CMCTool: public AbstractTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	/** Name of the file containing the desired kinematic
	trajectories. */
	PropertyStr _desiredKinematicsFileNameProp;
	std::string &_desiredKinematicsFileName;
	/** Name of the file containing the external loads applied to the model. */
	PropertyStr _externalLoadsFileNameProp;
	std::string &_externalLoadsFileName;
	/** Name of the file containing the model kinematics corresponding to the
	external loads. */
	PropertyStr _externalLoadsModelKinematicsFileNameProp;
	std::string &_externalLoadsModelKinematicsFileName;
	/** Name of the body to which the first set of external loads should be
	applied (e.g., the body name for the right foot). */
	PropertyStr _externalLoadsBody1Prop;
	std::string &_externalLoadsBody1;
	/** Name of the body to which the second set of external loads should be
	applied (e.g., the body name for the left foot). */
	PropertyStr _externalLoadsBody2Prop;
	std::string &_externalLoadsBody2;
	/** Name of the file containing the tracking tasks. */
	PropertyStr _taskSetFileNameProp;
	std::string &_taskSetFileName;
	/** Name of the file containing the constraints on the
	controls. */
	PropertyStr _constraintsFileNameProp;
	std::string &_constraintsFileName;
	/** Name of the file containing the actuator controls output by rra. */
	PropertyStr _rraControlsFileNameProp;
	std::string &_rraControlsFileName;
	/** Low-pass cut-off frequency for filtering the desired kinematics. A negative
	value results in no filtering.  The default value is -1.0, so no filtering. */
	PropertyDbl _lowpassCutoffFrequencyProp;
	double &_lowpassCutoffFrequency;
	/** Low-pass cut-off frequency for filtering the model kinematics corresponding
	to the external loads. A negative value results in no filtering.
	The default value is -1.0, so no filtering. */
	PropertyDbl _lowpassCutoffFrequencyForLoadKinematicsProp;
	double &_lowpassCutoffFrequencyForLoadKinematics;
	/** Time window over which the desired actuator forces are
	achieved. */
	PropertyDbl _targetDTProp;
	double &_targetDT;
	/** Flag indicating whether or not to use the curvature filter. */
	PropertyBool _useCurvatureFilterProp;
	bool &_useCurvatureFilter;
	/** Set whether or not to use reflexes. */
	PropertyBool _useReflexesProp;
	bool &_useReflexes;
	/** Flag indicating whether to use the fast CMC optimization
	target.  The fast target requires the desired accelerations to
	be met within the tolerance set by the convergence criterion.
	The optimizer fails if the acclerations constraints cannot be
	met, so the fast target is less robust.  The regular target
	does not require the acceleration constraints to be met; it
	meets them as well as it can. */
	PropertyBool _useFastTargetProp;
	bool &_useFastTarget;
	/** Preferred optimizer algorithm. */
	PropertyStr _optimizerAlgorithmProp;
	std::string &_optimizerAlgorithm;
	/** Perturbation size used by the optimizer to compute numerical derivatives. */
	PropertyDbl _optimizerDXProp;
	double &_optimizerDX;
	/** Convergence criterion for the optimizer. */
	PropertyDbl _convergenceCriterionProp;
	double &_convergenceCriterion;
	/** Maximum number of iterations for the optimizer. */
	PropertyInt _maxIterationsProp;
	int &_maxIterations;
	/** Print level for the optimizer, 0 - 3.
	0 = no printing, ..., 3 = detailed printing. */
	PropertyInt _printLevelProp;
	int &_printLevel;
	/** Flag indicating whether or not to compute average residuals. */
	PropertyBool _computeAverageResidualsProp;
	bool &_computeAverageResiduals;
	/** Flag indicating whether or not to make an adjustment in the center of
	mass of a body to reduced DC offsets in MX and MZ. */
	PropertyBool _adjustCOMToReduceResidualsProp;
	bool &_adjustCOMToReduceResiduals;
	/** Initial time for computing average residuals used to adjust COM. */
	PropertyDbl _initialTimeForCOMAdjustmentProp;
	double &_initialTimeForCOMAdjustment;
	/** Final time for computing average residuals used to adjust COM. */
	PropertyDbl _finalTimeForCOMAdjustmentProp;
	double &_finalTimeForCOMAdjustment;
	/** Name of the body whose center of mass is adjusted. */
	PropertyStr _adjustedCOMBodyProp;
	std::string &_adjustedCOMBody;
	/** Name of the output model file containing adjustments to anthropometry
	made to reduce average residuals. This file is written if the property
	adjust_com_to_reduce_residuals is set to true. */
	PropertyStr _outputModelFileProp;
	std::string &_outputModelFile;
	/** Flag indicating whether or not to adjust the kinematics in order to reduce residuals. */
	PropertyBool _adjustKinematicsToReduceResidualsProp;
	bool &_adjustKinematicsToReduceResiduals;
	/** Flag for turning on and off verbose printing. */
	PropertyBool _verboseProp;
	bool &_verbose;

	ActuatorSet _originalActuatorSet;
	/** Model integrand.  Make it a pointer so we can print results from a separate function. */
	ModelIntegrand *_integrand;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~CMCTool();
	CMCTool();
	CMCTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
	CMCTool(const CMCTool &aObject);
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void constructCorrectiveSprings();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	CMCTool&
		operator=(const CMCTool &aCMCTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	const std::string &getDesiredKinematicsFileName() { return _desiredKinematicsFileName; }
	void setDesiredKinematicsFileName(const std::string &aFileName) { _desiredKinematicsFileName = aFileName; }

	const std::string &getConstraintsFileName() { return _constraintsFileName; }
	void setConstraintsFileName(const std::string &aFileName) { _constraintsFileName = aFileName; }

	const std::string &getTaskSetFileName() { return _taskSetFileName; }
	void setTaskSetFileName(const std::string &aFileName) { _taskSetFileName = aFileName; }

	const std::string &getRRAControlsFileName() { return _rraControlsFileName; }
	void setRRAControlsFileName(const std::string &aFileName) { _rraControlsFileName = aFileName; }

	const std::string &getOutputModelFileName() { return _outputModelFile; }
	void setOutputModelFileName(const std::string &aFileName) { _outputModelFile = aFileName; }

	bool getAdjustCOMToReduceResiduals() { return _adjustCOMToReduceResiduals; }
	void setAdjustCOMToReduceResiduals(bool aAdjust) { _adjustCOMToReduceResiduals = aAdjust; }

	const std::string &getAdjustedCOMBody() { return _adjustedCOMBody; }
	void setAdjustedCOMBody(const std::string &aBody) { _adjustedCOMBody = aBody; }

	bool getAdjustKinematicsToReduceResiduals() { return _adjustKinematicsToReduceResiduals; }
	void setAdjustKinematicsToReduceResiduals(bool aAdjust) { _adjustKinematicsToReduceResiduals = aAdjust; }

	double getLowpassCutoffFrequency() const { return _lowpassCutoffFrequency; }
	void setLowpassCutoffFrequency(double aLowpassCutoffFrequency) { _lowpassCutoffFrequency = aLowpassCutoffFrequency; }

	// External loads get/set
	const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
	void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }
	const std::string &getExternalLoadsModelKinematicsFileName() const { return _externalLoadsModelKinematicsFileName; }
	void setExternalLoadsModelKinematicsFileName(const std::string &aFileName) { _externalLoadsModelKinematicsFileName = aFileName; }
	const std::string &getExternalLoadsBody1() const { return _externalLoadsBody1; }
	void setExternalLoadsBody1(const std::string &aName) { _externalLoadsBody1 = aName; }
	const std::string &getExternalLoadsBody2() const { return _externalLoadsBody2; }
	void setExternalLoadsBody2(const std::string &aName) { _externalLoadsBody2 = aName; }
	double getLowpassCutoffFrequencyForLoadKinematics() const { return _lowpassCutoffFrequencyForLoadKinematics; }
	void setLowpassCutoffFrequencyForLoadKinematics(double aLowpassCutoffFrequency) { _lowpassCutoffFrequencyForLoadKinematics = aLowpassCutoffFrequency; }

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	Storage *getForceStorage();
	Storage *getStateStorage();

	void setOriginalActuatorSet(const ActuatorSet &aActuatorSet);

#ifndef SWIG
	ControlSet* constructRRAControlSet(ControlSet *aControlConstraints);
	void initializeControlSetUsingConstraints(const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints,ControlSet *ControlSet);
	void adjustCOMToReduceResiduals(const Storage &qStore, const Storage &uStore);
	void adjustCOMToReduceResiduals(const Array<double> &aFAve,const Array<double> &aMAve);
	void addNecessaryAnalyses();
	void writeAdjustedModel();

	static void computeAverageResiduals(const Storage &aForceStore,Array<double> &rFAve,Array<double> &rMAve);
	static void computeAverageResiduals(Model &aModel, double aTi, double aTf, const Storage &aStatesStore, Array<double>& rFAve, Array<double>& rMAve);
#endif
//=============================================================================
};	// END of class CMCTool
//=============================================================================
//=============================================================================

}; // end namespace

#endif  // CMCTool_h__
