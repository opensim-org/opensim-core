#ifndef __IKTool_h__
#define __IKTool_h__
// IKTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Simulation/Model/AbstractTool.h>

#ifdef SWIG
	#ifdef OSIMTOOLS_API
		#undef OSIMTOOLS_API
		#define OSIMTOOLS_API
	#endif
#endif

namespace OpenSim {

class IKTaskSet;
class IKTrialSet;


//=============================================================================
//=============================================================================
/**
 * An investigation class for the IK solver.
 *
 * @author Eran Guendelman
 * @version 1.0
 */
class OSIMTOOLS_API IKTool: public AbstractTool
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
private:
	// tasks used to specify IK weights
	PropertyObj _ikTaskSetProp;
	IKTaskSet &_ikTaskSet;

	// the set of IK trials to perform
	PropertyObj _IKTrialSetProp;
	IKTrialSet &_IKTrialSet;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~IKTool();
	IKTool();
	IKTool(const std::string &aFileName, Model* guiModel=0);
	IKTool(const IKTool &aObject);
	virtual OpenSim::Object* copy() const;

	/* Register types to be used when reading an IKTool object from xml file. */
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
	IKTool&
		operator=(const IKTool &aIKTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------

	IKTrialSet& getIKTrialSet()
	{
		return _IKTrialSet;
	}

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual void run();

//=============================================================================
};	// END of class IKTool
//=============================================================================
} // namespace

#endif // __IKTool_h__
