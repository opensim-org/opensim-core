#ifndef _IndContactPowers_h_
#define _IndContactPowers_h_
// IndContactPowers.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include "suAnalysesDLL.h"
#include "IndAcc.h"


//=============================================================================
//=============================================================================
/**
 * A class for computing the powers delivered to contact elements induced by
 * the individual actuators of a model.  This analysis depends on a valid
 * contact force decomposition and the velocities of the contact points,
 * both expressed in the global frame.
 *
 * This class is derived from IndContactPowers, which holds model
 * states and the contact force decomposition.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class SUANALYSES_API IndContactPowers : public IndAcc
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Storge for the velocities of the contact points. */
	Storage *_velStore;
	/** Storage for the computed induced contact powers. */
	Storage **_pwrStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	IndContactPowers(Storage *aContactVelocities,
		Model *aModel,Storage *aStates,Storage *aControls,char *aBaseName,
		char *aDir=NULL,char *aExtension=NULL);
	virtual ~IndContactPowers();
private:
	void setNull();
	void constructDescription();
	void constructColumnLabels();
	void allocateStoragePointers();
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// CONTACT VELOCITIES
	Storage* getContactVelocities();

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	void computeContactPowers();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class IndContactPowers

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __IndContactPowers_h__
