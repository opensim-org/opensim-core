#ifndef __AnalyzeTool_h__
#define __AnalyzeTool_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  AnalyzeTool.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

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

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API AnalyzeTool : public AbstractTool {
OpenSim_DECLARE_CONCRETE_OBJECT(AnalyzeTool, AbstractTool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	/** Name of the states file. */
	PropertyStr _statesFileNameProp;
	std::string &_statesFileName;
	/** Name of the coordinates file. */
	PropertyStr _coordinatesFileNameProp;
	std::string &_coordinatesFileName;
	/** Name of the speeds file. */
	PropertyStr _speedsFileNameProp;
	std::string &_speedsFileName;
	/** Low-pass cut-off frequency for filtering the coordinates (does not apply to states). */
	PropertyDbl _lowpassCutoffFrequencyProp;
	double &_lowpassCutoffFrequency;

	/** Storage for the model states. */
	Storage *_statesStore;

	/** Whether to write result storages to files. */
	bool _printResultFiles;

    /** Whether the model and states should be loaded from input files */
    bool _loadModelAndInput;
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
	explicit AnalyzeTool(Model& aModel);

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
	void setStatesStorage(Storage& aStore);
	static Storage *createStatesStorageFromCoordinatesAndSpeeds(const Model& aModel, const Storage& aQStore, const Storage& aUStore);
	Storage& getStatesStorage();

	const std::string &getStatesFileName() const { return _statesFileName; }
	void setStatesFileName(const std::string &aFileName) { _statesFileName = aFileName; }
	const std::string &getCoordinatesFileName() const { return _coordinatesFileName; }
	void setCoordinatesFileName(const std::string &aFileName) { _coordinatesFileName = aFileName; }
	const std::string &getSpeedsFileName() const { return _speedsFileName; }
	void setSpeedsFileName(const std::string &aFileName) { _speedsFileName = aFileName; }
	double getLowpassCutoffFrequency() const { return _lowpassCutoffFrequency; }
	void setLowpassCutoffFrequency(double aLowpassCutoffFrequency) { _lowpassCutoffFrequency = aLowpassCutoffFrequency; }
    const bool getLoadModelAndInput() const { return _loadModelAndInput; }
    void setLoadModelAndInput(bool b) { _loadModelAndInput = b; }

	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void setStatesFromMotion(const SimTK::State& s, const Storage &aMotion, bool aInDegrees) SWIG_DECLARE_EXCEPTION;
	void loadStatesFromFile(SimTK::State& s ) SWIG_DECLARE_EXCEPTION;
	void verifyControlsStates();
	void setPrintResultFiles(bool aToWrite);
    void disableIntegrationOnlyProbes();
	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;
	virtual bool run(bool plotting) SWIG_DECLARE_EXCEPTION;

	//--------------------------------------------------------------------------
	// HELPER
	//--------------------------------------------------------------------------
#ifndef SWIG
	static void run(SimTK::State& s, Model &aModel, int iInitial, int iFinal, const Storage &aStatesStore, bool aSolveForEquilibrium);
#endif
//=============================================================================
};	// END of class AnalyzeTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __AnalyzeTool_h__


