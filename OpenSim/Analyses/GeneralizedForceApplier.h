#ifndef _GeneralizedForceApplier_h_
#define _GeneralizedForceApplier_h_
// GeneralizedForceApplier.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, May Q. Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "suAnalysesDLL.h"
#include "Contact.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying a generalized force during a
 * simulation.
 *
 * @author Frank C. Anderson, Saryn Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;
class AbstractCoordinate;

class SUANALYSES_API GeneralizedForceApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which generalized coordinate. */
	AbstractCoordinate *_genCoord;
	/** Generalized force to be applied. */
	double _generalizedForce;
	/** Factor that generalized force will be scaled by. */
	double _scaleFactor;
	/** Storage containing generalized force to be applied */
	Storage* _generalizedForceStorage;
	/** Index of collumn in Storage where generalized force is stored */
	int _genForceIndex;
	/** Flag to indicate whether or not to record the loads that are applied
	during an integration.  Recording these loads takes a lot of memory as they
	are stored every time the derivatives are evaluated (e.g., 6 times per
	integration step). */
	bool _recordAppliedLoads;
	/** Storage for the generalized force that was actually applied during 
	the simulation */
	Storage *_appliedGeneralizedForceStore;
	/** variable for testing */
	double _aTSet;



//=============================================================================
// METHODS
//=============================================================================
public:
	GeneralizedForceApplier(AbstractModel *aModel);	
	GeneralizedForceApplier(AbstractModel *aModel,AbstractCoordinate *aGenCoord);
	GeneralizedForceApplier(AbstractModel *aModel,AbstractCoordinate *aGenCoord,double aGeneralizedForce);
	GeneralizedForceApplier(AbstractModel *aModel,AbstractCoordinate *aGenCoord,Storage* aGeneralizedForceStorage,int aIndex);
	virtual ~GeneralizedForceApplier();
private:
	void setNull();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setGeneralizedCoordinate(AbstractCoordinate *aGenCoord);
	AbstractCoordinate* getGeneralizedCoordinate() const;
	void setScaleFactor(double aFactor);
	double getScaleFactor() const;
	void setGeneralizedForce(double aGeneralizedForce);
	double getGeneralizedForce() const;
	void setGeneralizedForceStorage(Storage* aGeneralizedForceStorage);
	Storage* getGeneralizedForceStorage() const;
	void setGeneralizedForceIndex(int aIndex);
	int getGeneralizedForceIndex() const;
	void setRecordAppliedLoads(bool aTrueFalse);
	bool getRecordAppliedLoads() const;
	Storage* getAppliedGeneralizedForceStorage();
	void setStorageCapacityIncrements(int aIncrement);
	
	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		applyActuation(double aT,double *aX,double *aY);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(char *aBaseName,char *aDir=NULL,double aDT=-1.0,
		char *aExtension=".sto");

//=============================================================================
};	// END of class GeneralizedForceApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __GeneralizedForceApplier_h__
