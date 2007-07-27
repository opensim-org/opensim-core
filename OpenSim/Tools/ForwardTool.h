#ifndef __ForwardTool_h__
#define __ForwardTool_h__
// ForwardTool.h
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
class OSIMTOOLS_API ForwardTool: public AbstractTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	/** Name of the controls file. */
	PropertyStr _controlsFileNameProp;
	std::string &_controlsFileName;
	/** Name of the initial states file. */
	PropertyStr _initialStatesFileNameProp;
	std::string &_initialStatesFileName;
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
	/** If true, the time steps from the initial states file are used during current integration */
	OpenSim::PropertyBool _useSpecifiedDtProp;
	bool &_useSpecifiedDt;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~ForwardTool();
	ForwardTool();
	ForwardTool(const std::string &aFileName, bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
	ForwardTool(const ForwardTool &aObject);
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
	ForwardTool&
		operator=(const ForwardTool &aForwardTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	const std::string &getControlsFileName() const { return _controlsFileName; }
	void setControlsFileName(const std::string &aFileName) { _controlsFileName = aFileName; }

	const std::string &getInitialStatesFileName() const { return _initialStatesFileName; }
	void setInitialStatesFileName(const std::string &aFileName) { _initialStatesFileName = aFileName; }

	bool getUseSpecifiedDt() const { return _useSpecifiedDt; }

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
	virtual bool run();

	//--------------------------------------------------------------------------
	// UTILITY (also used by the CMCTool)
	//--------------------------------------------------------------------------
	static void initializeExternalLoads(Model *aModel, 
		const std::string &aExternalLoadsFileName,
		const std::string &aExternalLoadsModelKinematicsFileName,
		const std::string &aExternalLoadsBody1,
		const std::string &aExternalLoadsBody2,
		double aLowpassCutoffFrequencyForLoadKinematics,
		ForceApplier **rRightForceApp=0,
		ForceApplier **rLeftForceApp=0,
		TorqueApplier **rRightTorqueApp=0,
		TorqueApplier **rLeftTorqueApp=0);

//=============================================================================
};	// END of class ForwardTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ForwardTool_h__


