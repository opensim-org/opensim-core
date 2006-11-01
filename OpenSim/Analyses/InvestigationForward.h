#ifndef __InvestigationForward_h__
#define __InvestigationForward_h__
// InvestigationForward.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/Model/Investigation.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include "Analyses.h"
#include "suAnalysesDLL.h"

#ifdef SWIG
	#ifdef SUANALYSES_API
		#undef SUANALYSES_API
		#define SUANALYSES_API
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
class SUANALYSES_API InvestigationForward: public Investigation
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
	virtual ~InvestigationForward();
	InvestigationForward();
	InvestigationForward(const std::string &aFileName);
	InvestigationForward(DOMElement *aElement);
	InvestigationForward(const InvestigationForward &aObject);
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
	InvestigationForward&
		operator=(const InvestigationForward &aInvestigationForward);
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
	void initializeExternalLoads();

//=============================================================================
};	// END of class InvestigationForward

}; //namespace
//=============================================================================
//=============================================================================

#endif // __InvestigationForward_h__


