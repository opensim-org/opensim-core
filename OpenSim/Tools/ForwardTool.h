#ifndef __ForwardTool_h__
#define __ForwardTool_h__
// ForwardTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyDblArray.h>
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

class AbstractBody;
class ForceApplier;
class TorqueApplier;
class LinearSpring;
class TorsionalSpring;
class ModelIntegrand;

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
protected:
	// BASIC INPUT
	/** Name of the controls file. */
	PropertyStr _controlsFileNameProp;
	std::string &_controlsFileName;
	/** Name of the states file.  The states file must at a minimum contain the
	initial states for a simulation.  If a complete set of states is available,
	the time stamps can be used to specify the integration steps and corrective
	springs, which allow perturbations, can be added to the simulation. */
	PropertyStr _statesFileNameProp;
	std::string &_statesFileName;
	/** If true, the time steps from the states file are used during
	current integration. */
	OpenSim::PropertyBool _useSpecifiedDtProp;
	bool &_useSpecifiedDt;

	// EXTERNAL LOADS
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
	/** Flag indicating whether or not to output corrective spring loads and other
	quantities. */
	OpenSim::PropertyBool _outputDetailedResultsProp;
	bool &_outputDetailedResults;

	// FOOT CONTACT EVENT TIMES
	/** Flag indicating wether or not to turn on a linear corrective spring for the right foot. */
	OpenSim::PropertyBool _body1LinSpringActiveProp;
	bool &_body1LinSpringActive;
	/** Flag indicating wether or not to turn on a torsional corrective spring for the right foot. */
	OpenSim::PropertyBool _body1TorSpringActiveProp;
	bool &_body1TorSpringActive;
	/** Flag indicating wether or not to turn on a linear corrective spring for the left foot. */
	OpenSim::PropertyBool _body2LinSpringActiveProp;
	bool &_body2LinSpringActive;
	/** Flag indicating wether or not to turn on a torsional corrective spring for the left foot. */
	OpenSim::PropertyBool _body2TorSpringActiveProp;
	bool &_body2TorSpringActive;
	/** Time at which the torsional spring comes on for body1 (if the spring is active). */
	PropertyDbl _body1TorSpringTimeOnProp;
	double &_body1TorSpringTimeOn;
	/** Time at which the torsional spring turns off for body1 (if the spring is active). */
	PropertyDbl _body1TorSpringTimeOffProp;
	double &_body1TorSpringTimeOff;
	/** Time at which the torsional spring turns on for body2 (if the spring is active). */
	PropertyDbl _body2TorSpringTimeOnProp;
	double &_body2TorSpringTimeOn;
	/** Time at which the torsional spring turns off for body2 (if the spring is active). */
	PropertyDbl _body2TorSpringTimeOffProp;
	double &_body2TorSpringTimeOff;

	// CORRECTIVE SPRING PARAMETERS
	/** Rise time for scaling functions for the torsional corrective springs. */
	PropertyDbl _tauProp;
	double &_tau;
	/** Scaling rise-time for the body1 torsional spring turning on. */
	PropertyDbl _tauBody1OnProp;
	double &_tauBody1On;
	/** Scaling rise-time for the body1 torsional spring turning off. */
	PropertyDbl _tauBody1OffProp;
	double &_tauBody1Off;
	/** Scaling rise-time for the body2 torsional spring turning on. */
	PropertyDbl _tauBody2OnProp;
	double &_tauBody2On;
	/** Scaling rise-time for the body2 torsional spring turning off. */
	PropertyDbl _tauBody2OffProp;
	double &_tauBody2Off;
	/** Force magnitude above which the linear springs start to scale in to effect. */
	PropertyDbl _springTransitionStartForceProp;
	double &_springTransitionStartForce;
	/** Force magnitude below which the linear springs start to scale out of effect. */
	PropertyDbl _springTransitionEndForceProp;
	double &_springTransitionEndForce;
	/** Force magnitude below which no corrective linear spring forces are applied.
	This allows an unperturbed forward integration to execute with minimal drift. */
	PropertyDbl _forceThresholdProp;
	double &_forceThreshold;
	/** Torque mangnitude below which no corrective torsional spring foreces are applied.
	This allows an unperturbed forward integration to execute with minimal drift. */
	PropertyDbl _torqueThresholdProp;
	double &_torqueThreshold;
	/** Stiffness for linear corrective springs. */
	PropertyDblArray _kLinProp;
	Array<double> &_kLin;
	/** Damping for linear corrective springs. */
	PropertyDblArray _bLinProp;
	Array<double> &_bLin;
	/** Stiffness for torsional corrective springs. */
	PropertyDblArray _kTorProp;
	Array<double> &_kTor;
	/** Damping for torsional corrective springs. */
	PropertyDblArray _bTorProp;
	Array<double> &_bTor;

	// INTERNAL WORK ARRAYS
	/** Model integrand.  Make it a pointer so we can print results from a separate function. */
	ModelIntegrand *_integrand;
	/** Storage for the input states. */
	Storage *_yStore;
	/** Flag indicating whether or not to write to the results (GUI will set this to false). */
	bool _printResultFiles;
	/** Pointer to the linear and torsional corrective springs. */
	LinearSpring *_body1Lin, *_body2Lin;
	TorsionalSpring *_body1Tor, *_body2Tor;

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
protected:
	//void constructCorrectiveSprings(ForceApplier *aRightGRFApp,ForceApplier *aLeftGRFApp);
	LinearSpring*
		addLinearCorrectiveSpring(const Storage &aQStore,const Storage &aUStore,const ForceApplier &aAppliedForce);
	TorsionalSpring*
		addTorsionalCorrectiveSpring(const Storage &aQStore,const Storage &aUStore,AbstractBody *aBody,
		double aTauOn,double aTimeOn,double aTauOff,double aTimeOff);

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

	const std::string &getStatesFileName() const { return _statesFileName; }
	void setStatesFileName(const std::string &aFileName) { _statesFileName = aFileName; }

	bool getUseSpecifiedDt() const { return _useSpecifiedDt; }
	void setUseSpecifiedDt(bool aUseSpecifiedDt) { _useSpecifiedDt = aUseSpecifiedDt; }

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
	void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

	Storage *getStateStorage();

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run();
	void printResults();

	//--------------------------------------------------------------------------
	// UTILITY
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


