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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "suAnalysesDLL.h"
#include "Contact.h"
#include "Decomp.h"


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

class SUANALYSES_API GeneralizedForceApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which generalized coordinate. */
	int _genCoord;
	/** Generalized force to be applied. */
	double _generalizedForce;
	/** Factor that generalized force will be scaled by. */
	double _scaleFactor;
	/** Storage containing generalized force to be applied */
	Storage* _generalizedForceStorage;
	/** Index of collumn in Storage where generalized force is stored */
	int _genForceIndex;
	/** Storage for the generalized force that was actually applied during 
	the simulation */
	Storage *_appliedGeneralizedForceStore;
	/** variable for testing */
	double _aTSet;



//=============================================================================
// METHODS
//=============================================================================
public:
	GeneralizedForceApplier(Model *aModel);	
	GeneralizedForceApplier(Model *aModel,int aGenCoord);
	GeneralizedForceApplier(Model *aModel,int aGenCoord,double aGeneralizedForce);
	GeneralizedForceApplier(Model *aModel,int aGenCoord,Storage* aGeneralizedForceStorage,int aIndex);
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
	void setGeneralizedCoordinate(int aGenCoord);
	int getGeneralizedCoordinate() const;
	void setScaleFactor(double aFactor);
	double getScaleFactor() const;
	void setGeneralizedForce(double aGeneralizedForce);
	double getGeneralizedForce() const;
	void setGeneralizedForceStorage(Storage* aGeneralizedForceStorage);
	Storage* getGeneralizedForceStorage() const;
	void setGeneralizedForceIndex(int aIndex);
	int getGeneralizedForceIndex() const;
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
