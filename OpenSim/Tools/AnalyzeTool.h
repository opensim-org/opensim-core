#ifndef __AnalyzeTool_h__
#define __AnalyzeTool_h__
// AnalyzeTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include "osimToolsDLL.h"

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim { 

class ForceApplier;
class TorqueApplier;

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API AnalyzeTool: public AbstractTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	/** Name of the controls file. */
	PropertyStr _controlsFileNameProp;
	std::string &_controlsFileName;
	/** Name of the states file. */
	PropertyStr _statesFileNameProp;
	std::string &_statesFileName;
	/** Name of the pseudo-states file. */
	PropertyStr _pseudoStatesFileNameProp;
	std::string &_pseudoStatesFileName;
	/** Name of the coordinates file. */
	PropertyStr _coordinatesFileNameProp;
	std::string &_coordinatesFileName;
	/** Name of the speeds file. */
	PropertyStr _speedsFileNameProp;
	std::string &_speedsFileName;
	/** Low-pass cut-off frequency for filtering the coordinates (does not apply to states). */
	PropertyDbl _lowpassCutoffFrequencyProp;
	double &_lowpassCutoffFrequency;

	/** Name of the file containing the external loads applied to the model. */
	OpenSim::PropertyStr _externalLoadsFileNameProp;
	std::string &_externalLoadsFileName;
	/** Name of the file containing the model kinematics corresponding to the
	external loads. */
	OpenSim::PropertyStr _externalLoadsModelKinematicsFileNameProp;
	std::string &_externalLoadsModelKinematicsFileName;
	/** Name of the body to which the first set of external loads should be
	applied (e.g., the body name for the right foot). */
	OpenSim::PropertyStr _externalLoadsBody1Prop;
	std::string &_externalLoadsBody1;
	/** Name of the body to which the second set of external loads should be
	applied (e.g., the body name for the left foot). */
	OpenSim::PropertyStr _externalLoadsBody2Prop;
	std::string &_externalLoadsBody2;
	/** Low-pass cut-off frequency for filtering the model kinematics corresponding
	to the external loads. A negative value results in no filtering.
	The default value is -1.0, so no filtering. */
	OpenSim::PropertyDbl _lowpassCutoffFrequencyForLoadKinematicsProp;
	double &_lowpassCutoffFrequencyForLoadKinematics;

	/** Control set. */
	ControlSet *_controlSet;

	/** Storage for the model states. */
	Storage *_statesStore;

	/** Storage for the model pseudo states. */
	Storage *_pseudoStore;

	/** Whether to write result storages to files. */
	bool _printResultFiles;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~AnalyzeTool();
	AnalyzeTool();
	AnalyzeTool(const std::string &aFileName, bool aLoadModelAndInput=true) SWIG_DECLARE_EXCEPTION;
	AnalyzeTool(const AnalyzeTool &aObject);
	AnalyzeTool(Model* aModel);
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
	AnalyzeTool&
		operator=(const AnalyzeTool &aAnalyzeTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setControlSet(ControlSet *aSet);
	ControlSet* getControlSet();
	void setStatesStorage(Storage *aStore);
	static Storage *createStatesStorageFromCoordinatesAndSpeeds(const Model *aModel, const Storage *aQStore, const Storage *aUStore);
	Storage* getStatesStorage();
	void setPseudoStatesStorage(Storage *aStore);
	Storage* getPseudoStatesStorage();

	const std::string &getControlsFileName() const { return _controlsFileName; }
	void setControlsFileName(const std::string &aFileName) { _controlsFileName = aFileName; }
	const std::string &getStatesFileName() const { return _statesFileName; }
	void setStatesFileName(const std::string &aFileName) { _statesFileName = aFileName; }
	const std::string &getPseudoStatesFileName() const { return _pseudoStatesFileName; }
	void setPseudoStatesFileName(const std::string &aFileName) { _pseudoStatesFileName = aFileName; }
	const std::string &getCoordinatesFileName() const { return _coordinatesFileName; }
	void setCoordinatesFileName(const std::string &aFileName) { _coordinatesFileName = aFileName; }
	const std::string &getSpeedsFileName() const { return _speedsFileName; }
	void setSpeedsFileName(const std::string &aFileName) { _speedsFileName = aFileName; }
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
	// UTILITIES
	//--------------------------------------------------------------------------
	void setStatesFromMotion(const Storage &aMotion, bool aInDegrees) SWIG_DECLARE_EXCEPTION;
	void loadControlsFromFile() SWIG_DECLARE_EXCEPTION;
	void loadStatesFromFile() SWIG_DECLARE_EXCEPTION;
	void loadPseudoStatesFromFile() SWIG_DECLARE_EXCEPTION;
	void verifyControlsStatesPseudoStates();
	double getControlsStatesPseudoStates(int aIndex,Array<double> &rX,Array<double> &rY,Array<double> &rP);
	void setPrintResultFiles(bool aToWrite);

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;

	//--------------------------------------------------------------------------
	// HELPER
	//--------------------------------------------------------------------------
	static void run(Model &aModel, int iInitial, int iFinal, const Storage &aStatesStore, Storage *aPseudoStore, ControlSet *aControlSet, bool aSolveForEquilibrium);

//=============================================================================
};	// END of class AnalyzeTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __AnalyzeTool_h__


