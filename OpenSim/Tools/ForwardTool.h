#ifndef __ForwardTool_h__
#define __ForwardTool_h__
// ForwardTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyDblVec3.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Model/LinearSpring.h>
#include <OpenSim/Simulation/Model/TorsionalSpring.h>

#include "osimToolsDLL.h"

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim { 

class Body;
class PrescribedForce;
class ControlSet;


//=============================================================================
//=============================================================================
/**
 * A concrete tool for perfroming forward dynamics simulations
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
	/** Flag indicating whether or not to turn on a linear corrective spring for the right foot. */
	OpenSim::PropertyBool _body1LinSpringActiveProp;
	bool &_body1LinSpringActive;
	/** Flag indicating whether or not to turn on a torsional corrective spring for the right foot. */
	OpenSim::PropertyBool _body1TorSpringActiveProp;
	bool &_body1TorSpringActive;
	/** Flag indicating whether or not to turn on a linear corrective spring for the left foot. */
	OpenSim::PropertyBool _body2LinSpringActiveProp;
	bool &_body2LinSpringActive;
	/** Flag indicating whether or not to turn on a torsional corrective spring for the left foot. */
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
	PropertyDblVec3 _kLinProp;
	SimTK::Vec3 &_kLin;
	/** Damping for linear corrective springs. */
	PropertyDblVec3 _bLinProp;
	SimTK::Vec3 &_bLin;
	/** Stiffness for torsional corrective springs. */
	PropertyDblVec3 _kTorProp;
	SimTK::Vec3 &_kTor;
	/** Damping for torsional corrective springs. */
	PropertyDblVec3 _bTorProp;
	SimTK::Vec3 &_bTor;

	/** Pointer to the linear and torsional corrective springs. */
	LinearSpring *_body1Lin, *_body2Lin;
	TorsionalSpring *_body1Tor, *_body2Tor;

	// INTERNAL WORK ARRAYS

	/** Storage for the input states. */
	Storage *_yStore;
	/** Flag indicating whether or not to write to the results (GUI will set this to false). */
	bool _printResultFiles;

    /** pointer to the simulation Manager */
    Manager* _manager;

	/*** Private place to save some deserializtion info in case needed later */
	std::string _parsingLog;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~ForwardTool();
	ForwardTool();
	ForwardTool(const std::string &aFileName,bool aUpdateFromXMLNode=true,bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
	ForwardTool(const ForwardTool &aObject);
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ForwardTool&
		operator=(const ForwardTool &aForwardTool);
#endif

	virtual void updateFromXMLNode();
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

    void setManager( Manager& m );
    const Manager& getManager() const; 

	const std::string &getStatesFileName() const { return _statesFileName; }
	void setStatesFileName(const std::string &aFileName) { _statesFileName = aFileName; }

	bool getUseSpecifiedDt() const { return _useSpecifiedDt; }
	void setUseSpecifiedDt(bool aUseSpecifiedDt) { _useSpecifiedDt = aUseSpecifiedDt; }

	// External loads get/set
	const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
	void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }
	const std::string &getExternalLoadsModelKinematicsFileName() const { return _externalLoadsModelKinematicsFileName; }
	void setExternalLoadsModelKinematicsFileName(const std::string &aFileName) { _externalLoadsModelKinematicsFileName = aFileName; }
	double getLowpassCutoffFrequencyForLoadKinematics() const { return _lowpassCutoffFrequencyForLoadKinematics; }
	void setLowpassCutoffFrequencyForLoadKinematics(double aLowpassCutoffFrequency) { _lowpassCutoffFrequencyForLoadKinematics = aLowpassCutoffFrequency; }
	void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;
	void printResults();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	static double Step(double t, double t0, double t1); 
    static double SigmaUp(double tau,double to,double t);
    static double SigmaDn(double tau,double to,double t);

	void loadStatesStorage (std::string& statesFileName, Storage*& rYStore) const; 
	const std::string& getParsingLog() { return _parsingLog; };
protected:
	void setDesiredStatesForControllers(Storage& rYStore);
	int determineInitialTimeFromStatesStorage(double &rTI);
	void InitializeSpecifiedTimeStepping(Storage *aYStore, Manager& aManager);
	void addCorrectiveSprings(SimTK::State &s, const Storage *aYStore, const PrescribedForce *aBody1Force,const PrescribedForce *aBody2Force);
	LinearSpring* addLinearCorrectiveSpring(SimTK::State &s, const Storage &aQStore,const Storage &aUStore,const PrescribedForce &aAppliedForce);
	TorsionalSpring* addTorsionalCorrectiveSpring(SimTK::State &s, const Storage &aQStore,const Storage &aUStore, Body *aBody,
		double aTauOn,double aTimeOn,double aTauOff,double aTimeOff);
private:

//=============================================================================
};	// END of class ForwardTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ForwardTool_h__


