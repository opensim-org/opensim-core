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

//=============================================================================
};	// END of class InvestigationForward

}; //namespace
//=============================================================================
//=============================================================================

#endif // __InvestigationForward_h__


