#ifndef __PerturbationTool_h__
#define __PerturbationTool_h__
// PerturbationTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include "osimToolsDLL.h"
#include "ForwardTool.h"
#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim { 

class ForceApplier;
class LinearSpring;
class TorsionalSpring;

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API PerturbationTool: public ForwardTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	// PERTURBATION PARAMETERS
	/** Perturbation time window. */
	PropertyDbl _pertWindowProp;
	double &_pertWindow;
	/** Time increment between perturbation windows. */
	PropertyDbl _pertIncrementProp;
	double &_pertIncrement;
	/** Magnitude of perturbation. */
	PropertyDbl _pertDFProp;
	double &_pertDF;
	/** Actuators to perturb. */
	PropertyStrArray _actuatorsToPerturbProp;
	Array<std::string> &_actuatorsToPerturb;
	/** Whether or not to perturb gravity. */
	PropertyBool _perturbGravityProp;
	bool &_perturbGravity;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PerturbationTool();
	PerturbationTool(const std::string &aFileName);
	virtual ~PerturbationTool();
	// Copy constrctor and virtual copy 
	PerturbationTool(const PerturbationTool &aObject);
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	PerturbationTool&
		operator=(const PerturbationTool &aPerturbationTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION;
	virtual void printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class PerturbationTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __PerturbationTool_h__


