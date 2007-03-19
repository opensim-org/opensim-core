#ifndef _BodyIndAccCOM_h_
#define _BodyIndAccCOM_h_
// BodyIndAccCOM.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn R. Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include "osimAnalysesDLL.h"
#include "IndAcc.h"
#include "BodyIndAcc.h"


//=============================================================================
//=============================================================================
/**
 * A class for computing induced accelerations of the COM of a specified set of
 * the body segments of a model.
 *
 * @author Frank C. Anderson & Saryn R. Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;
class AbstractBody;

class OSIMANALYSES_API BodyIndAccCOM : public BodyIndAcc 
{
//=============================================================================
// DATA
//=============================================================================
private:
	Storage *_aeCOMBodyStore;
	Storage *_veCOMBodyStore;
	Storage *_peCOMBodyStore;
	Storage *_posStore;
	Storage *_velStore;
	int _aN;
	AbstractBody* *_aBodyList;

//=============================================================================
// METHODS
//=============================================================================
public:
	BodyIndAccCOM(AbstractModel *aModel, int aN=0, AbstractBody* aBodyList[] = NULL);
	BodyIndAccCOM(AbstractModel *aModel,Storage *aStates,Storage *aControls,
		char *aBaseName,char *aDir=NULL,char *aExtension=NULL,
		int aN=0,AbstractBody* aBodyList[]=NULL);
	virtual ~BodyIndAccCOM();
private:
	void constructDescription();
	void constructColumnLabels();
	void setBodyList(AbstractBody* aBodyList[]);
	void allocateBodyStorage();
	void deleteBodyStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setStorageCapacityIncrements(int aIncrement);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	int computeBodyCOMAccelerations();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void sumBodyCOMAccelerationResults();

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class BodyIndAccCOM

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyIndAccCOM_h__
