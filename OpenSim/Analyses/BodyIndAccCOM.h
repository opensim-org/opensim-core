#ifndef _BodyIndAccCOM_h_
#define _BodyIndAccCOM_h_
// BodyIndAccCOM.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn R. Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "suAnalysesDLL.h"
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

class SUANALYSES_API BodyIndAccCOM : public BodyIndAcc 
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
	int *_aBodyList;

//=============================================================================
// METHODS
//=============================================================================
public:
	BodyIndAccCOM(Model *aModel, int aN=0, int aBodyList[] = NULL);
	BodyIndAccCOM(Model *aModel,Storage *aStates,Storage *aControls,
		char *aBaseName,char *aDir=NULL,char *aExtension=NULL,
		int aN=0,int aBodyList[]=NULL);
	virtual ~BodyIndAccCOM();
private:
	void constructDescription();
	void constructColumnLabels();
	void setBodyList(int aBodyList[]);
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
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class BodyIndAccCOM

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __BodyIndAccCOM_h__
