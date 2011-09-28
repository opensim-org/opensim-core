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
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Control/ControlSet.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {


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
	PropertyStr _desiredPointsFileNameProp;
	std::string &_desiredPointsFileName;

    /** Name of the file containing the desired kinematic trajectories. */ 		 
    PropertyStr _desiredKinematicsFileNameProp; 		 
    std::string &_desiredKinematicsFileName;

    /** Name of the file containing the tracking tasks. */  	 	 
    PropertyStr _taskSetFileNameProp; 		 
    std::string &_taskSetFileName; 		 

      /** Name of the file containing the constraints on the controls. */
	PropertyStr _constraintsFileNameProp;
	std::string &_constraintsFileName;

	/** Name of the file containing the actuator controls output by rra. */
	PropertyStr _rraControlsFileNameProp;
	std::string &_rraControlsFileName;
	/** Low-pass cut-off frequency for filtering the desired kinematics. A negative
	value results in no filtering.  The default value is -1.0, so no filtering. */
	PropertyDbl _lowpassCutoffFrequencyProp;
	double &_lowpassCutoffFrequency;

    /** Time window over which the desired actuator forces are achieved */
    PropertyDbl _targetDTProp;  	 	   
    double &_targetDT;

    /** Flag indicating whether or not to use the curvature filter. */
    PropertyBool _useCurvatureFilterProp;  	 	 
    bool &_useCurvatureFilter;

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
	PropertyDbl _numericalDerivativeStepSizeProp;
	double &_numericalDerivativeStepSize;
	/** Convergence criterion for the optimizer. */
	PropertyDbl _optimizationConvergenceToleranceProp;
	double &_optimizationConvergenceTolerance;
	/** Maximum number of iterations for the optimizer. */
	PropertyInt _maxIterationsProp;
	int &_maxIterations;
	/** Print level for the optimizer, 0 - 3.
	0 = no printing, ..., 3 = detailed printing. */
	PropertyInt _printLevelProp;
	int &_printLevel;
	/** Flag for turning on and off verbose printing. */
	PropertyBool _verboseProp;
	bool &_verbose;

	ForceSet _originalForceSet;

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

	const std::string &getDesiredPointsFileName() { return _desiredPointsFileName; }
	void setDesiredPointsFileName(const std::string &aFileName) { _desiredPointsFileName = aFileName; }

	const std::string &getDesiredKinematicsFileName() { return _desiredKinematicsFileName; }
	void setDesiredKinematicsFileName(const std::string &aFileName) { _desiredKinematicsFileName = aFileName; }

    const std::string &getConstraintsFileName() { return _constraintsFileName; }  	 	 
    void setConstraintsFileName(const std::string &aFileName) { _constraintsFileName = aFileName; } 		 
  		 
    const std::string &getTaskSetFileName() { return _taskSetFileName; } 		 
    void setTaskSetFileName(const std::string &aFileName) { _taskSetFileName = aFileName; }

	const std::string &getRRAControlsFileName() { return _rraControlsFileName; }
	void setRRAControlsFileName(const std::string &aFileName) { _rraControlsFileName = aFileName; }

	double getLowpassCutoffFrequency() const { return _lowpassCutoffFrequency; }
	void setLowpassCutoffFrequency(double aLowpassCutoffFrequency) { _lowpassCutoffFrequency = aLowpassCutoffFrequency; }

    double getTimeWindow() const { return _targetDT; }  	 	 
    void setTimeWindow(double aTargetDT) { _targetDT = aTargetDT; } 		 

	// External loads get/set
	const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
	void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }

    // Target selection
    bool getUseFastTarget() const { return _useFastTarget;};  	 	 
    void setUseFastTarget(bool useFastTarget) const {  _useFastTarget=useFastTarget; };


	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	Storage &getForceStorage();

	void setOriginalForceSet(const ForceSet &aForceSet);

#ifndef SWIG
	ControlSet* constructRRAControlSet(ControlSet *aControlConstraints);
	void initializeControlSetUsingConstraints(const ControlSet *aRRAControlSet,const ControlSet *aControlConstraints, ControlSet& rControlSet );
	void addNecessaryAnalyses();

#endif
//=============================================================================
};	// END of class CMCTool
//=============================================================================
//=============================================================================

}; // end namespace

#endif  // CMCTool_h__
