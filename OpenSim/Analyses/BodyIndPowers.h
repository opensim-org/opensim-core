#ifndef _BodyIndPowers_h_
#define _BodyIndPowers_h_
// BodyIndPowers.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include "suAnalysesDLL.h"
#include "BodyIndAcc.h"


//=============================================================================
//=============================================================================
/**
 * A class for computing induced segmental powers.
 *
 *
 * @author Frank C. Anderson, Saryn R. Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class SUANALYSES_API BodyIndPowers : public BodyIndAcc
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	Storage **_powerStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	BodyIndPowers(AbstractModel *aModel);
	BodyIndPowers(AbstractModel *aModel,Storage *aStates,Storage *aControls,char *aBaseName,
		char *aDir=NULL,char *aExtension=NULL);
	virtual ~BodyIndPowers();
private:
	void constructDescription();
	void constructColumnLabels();
	void allocateStoragePointers();
	void allocateStorage();
	void deleteStorage();


public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setStorageCapacityIncrements(int aIncrement);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	int computeBodyPowers();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void sumPowerResults();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class BodyIndPowers

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyIndPowers_h__
