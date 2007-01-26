// InvestigationCMCGait.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
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
#ifndef InvestigationCMCGait_h__
#define InvestigationCMCGait_h__

#include "rdCMCDLL.h"
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/FunctionSet.h>
#include <OpenSim/Simulation/Model/Investigation.h>
#include <OpenSim/Simulation/Control/ControlSet.h>

#ifdef SWIG
	#ifdef RDCMC_API
		#undef RDCMC_API
		#define RDCMC_API
	#endif
#endif

namespace OpenSim {

class XMLDocument;
//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class RDCMC_API InvestigationCMCGait: public Investigation
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
	/** Name of the body whose center of mass is adjusted. */
	PropertyStr _adjustedCOMBodyProp;
	std::string &_adjustedCOMBody;
	/** Name of the output model file containing adjustments to anthropometry
	made to reduce average residuals. This file is written if the property
	adjust_com_to_reduce_residuals is set to true. */
	PropertyStr _outputModelFileProp;
	std::string &_outputModelFile;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~InvestigationCMCGait();
	InvestigationCMCGait();
	InvestigationCMCGait(const std::string &aFileName);
	InvestigationCMCGait(DOMElement *aElement);
	InvestigationCMCGait(const InvestigationCMCGait &aObject);
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
private:
	void setNull();
	void setupProperties();
	void constructCorrectiveSprings();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	InvestigationCMCGait&
		operator=(const InvestigationCMCGait &aInvestigationCMCGait);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	
	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual void run();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
#ifndef SWIG
	ControlSet*
		constructRRAControlSet(ControlSet *aControlConstraints);
	void
		initializeControlSetUsingConstraints(
		const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints,ControlSet *ControlSet);
	void computeInitialStatesFromCoordinates(
		const FunctionSet &aQSet,Array<double> &rYI);
	void computeAverageResiduals(
		Array<double> &rFAve,Array<double> &rMAve);
	void adjustCOMToReduceResiduals(
		const Array<double> &aFAve,const Array<double> &aMAve);
	void addNecessaryAnalyses();
#endif
//=============================================================================
};	// END of class InvestigationCMCGait
//=============================================================================
//=============================================================================

}; // end namespace

#endif  // InvestigationCMCGait_h__
