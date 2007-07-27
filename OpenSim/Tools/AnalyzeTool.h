#ifndef __AnalyzeTool_h__
#define __AnalyzeTool_h__
// AnalyzeTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
	void setStatesStorageFromCoordinatesAndSpeeds(const Storage *aQStore, const Storage *aUStore);
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
	void loadControlsStatesPseudoStatesExternalLoadsFromFiles();
	void verifyControlsStatesPseudoStates();
	double getControlsStatesPseudoStates(int aIndex,Array<double> &rX,Array<double> &rY,Array<double> &rP);
	void setPrintResultFiles(bool aToWrite);
	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run();


//=============================================================================
};	// END of class AnalyzeTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __AnalyzeTool_h__


