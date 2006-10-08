#ifndef __InvestigationIK_h__
#define __InvestigationIK_h__
// InvestigationIK.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Applications/Workflow/workflowDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Simulation/Model/Investigation.h>

#ifdef SWIG
	#ifdef workflow_API
		#undef workflow_API
		#define workflow_API
	#endif
#endif

namespace OpenSim {

class XMLDocument;
class SimmMarkerSet;
class SimmCoordinateSet;
class SimmIKTrialParamsSet;


//=============================================================================
//=============================================================================
/**
 * An investigation class for the IK solver.
 *
 * @author Eran Guendelman
 * @version 1.0
 */
class workflow_API InvestigationIK: public Investigation
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:

	// marker set for updating markers in model before doing IK
	PropertyObj _markerSetProp;
	SimmMarkerSet &_markerSet;

	// coordinate set for updating coordinates in model before doing IK
	PropertyObj _coordinateSetProp;
	SimmCoordinateSet &_coordinateSet;

	// parameters for the set of IK trials to perform
	PropertyObj _IKTrialParamsSetProp;
	SimmIKTrialParamsSet &_IKTrialParamsSet;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~InvestigationIK();
	InvestigationIK();
	InvestigationIK(const std::string &aFileName);
	InvestigationIK(DOMElement *aElement);
	InvestigationIK(const InvestigationIK &aObject);
	virtual OpenSim::Object* copy() const;
	virtual OpenSim::Object* copy(DOMElement *aElement) const;

	/* Register types to be used when reading an InvestigationIK object from xml file. */
	static void registerTypes();
private:
	void setNull();
	void setupProperties();
	void constructCorrectiveSprings();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	InvestigationIK&
		operator=(const InvestigationIK &aInvestigationIK);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	SimmMarkerSet& getMarkerSet()
	{
	   return _markerSet;
	}

	SimmCoordinateSet &getCoordinateSet() const
	{
		return _coordinateSet;
	}

	SimmIKTrialParamsSet& getIKTrialParamsSet()
	{
		return _IKTrialParamsSet;
	}

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual void run();

//=============================================================================
};	// END of class InvestigationIK
//=============================================================================
} // namespace

#endif // __InvestigationIK_h__
