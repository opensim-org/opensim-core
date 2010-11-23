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

	/** Name of the file containing the external loads applied to the model. */
	OpenSim::PropertyStr _externalLoadsFileNameProp;
	std::string &_externalLoadsFileName;

	/** Storage for the model states. */
	Storage *_statesStore;

	/** Whether to write result storages to files. */
	bool _printResultFiles;

    /** Wheter the model and states should be loaded from input files */
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
	AnalyzeTool(Model& aModel);
	virtual Object* copy() const;
	//virtual void updateFromXMLNode();
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
	//void setControlSet(ControlSet& aSet);
	//ControlSet& getControlSet();
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

	// External loads get/set
	const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
	void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }

	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void setStatesFromMotion(const SimTK::State& s, const Storage &aMotion, bool aInDegrees) SWIG_DECLARE_EXCEPTION;
	void loadStatesFromFile(SimTK::State& s ) SWIG_DECLARE_EXCEPTION;
	void verifyControlsStates();
	void setPrintResultFiles(bool aToWrite);

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


