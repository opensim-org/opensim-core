#ifndef _Decomp_h_
#define _Decomp_h_
// Decomp.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * An abstract base class for supporting the decomposition of contact
 * forces.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMANALYSES_API Decomp : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
public:
	static const char *ADDON_COMPONENT_NAMES[];

protected:
	/** Number of action components. */
	int _nc;
	/** Number of independent action components. */
	int _nic;
	/** Number of contact points. */
	int _np;
	/** Index to the last actuator component. */
	int _cAct;
	/** Index to the gravity component. */
	int _cGrav;
	/** Index to the velocity component. */
	int _cVel;
	/** Index to the inertial component. */
	int _cIner;
	/** Index to the all actuators component. */
	int _cAllAct;
	/** Index to the all component. */
	int _cAll;

	/** Array of component names. */
	Array<std::string> _cNames;
	/** Contact threashold for when to perform a decomposition. */
	double _contactThreshold;
	/** Flag to indicate whether or not to use preset contact established
	settings.  If this flag is set to true, computation of contact forces
	is not performed in order to determine if contact has been established.
	Rather, it is assumed that the caller has preset the desired
	contact-established settings. */
	bool _usePresetContactEstablishedSettings;
	/** Array of flags to indicate whether or not contact has been established
	at each of the contact pooints. */
	bool *_contactEstablished;
	/** Contiguous memory allocation for the force decomposition results. */
	double *_fContig;
	/** Arrays for storing the current force decomposition. */
	double ***_f;
	/** Storage objects for storing the time history of the decomposition. */
	Storage **_fStore;
	/** Storage objects for storing the contact point accelerations */
	Storage **_cpaStore;
	/** Flag which indicates whether or not the contact point accelerations
	are being recorded */
	bool _recordContactPointAccelerations;

private:
	/** Flag which indicates whether or not the decomposition is NULL.  A
	NULL decomposition means no decompostion- all induced contact forces
	zero. */
	bool _useNullDecomposition;

//=============================================================================
// METHODS
//=============================================================================
public:
	Decomp(AbstractModel *aModel);
	Decomp(AbstractModel *aModel,char *aBaseName,char *aDir=NULL,
		char *aExtension=NULL);
	virtual ~Decomp();
private:
	void setNull();
	void initializeNumbers();
	void constructComponentNames();
	void constructDescription();
	void constructColumnLabels();
	void allocateElementVectors();
	void allocateStoragePointers();
	void allocateStorage();
	void deleteStorage();
	void createNullDecomposition();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// NUMBERS
	int getNumComponents();
	int getNumIndependentComponents();
	int getNumElements();
	// INDICES
	int getLastActuatorIndex();
	int getGravityIndex();
	int getVelocityIndex();
	int getInertialIndex();
	int getAllActuatorsIndex();
	int getAllIndex();
	// CONTACT THREASHOLD
	void setContactThreshold(double aThreshold);
	double getContactThreshold();
	// CONTACT ESTABLISHED
	void setUsePresetContactEstablishedSettings(bool aTrueFalse);
	bool getUsePresetContactEstablishedSettings() const;
	void setContactEstablished(int aIndex,bool aTrueFalse);
	bool getContactEstablished(int aIndex) const;
	// RECORD CONTACT POINT ACCELERATIONS
	void setRecordContactPointAccelerations(bool aTrueFalse);
	bool getRecordContactPointAccelerations();
	// NAMES
	std::string getComponentName(int aC);
	// STORAGE
	virtual void setStorageCapacityIncrements(int aIncrement);
	const Storage* getDecomposition(int aC) const;
	// NULL DECOMPOSITION
	bool getUseNullDecomposition();

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	virtual void compute(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY) = 0;

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void sum();

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,
		int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,double *aX,double *aY,
		void *aClientData=NULL);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");
private:
	virtual int
		readDecomposition(char *aBaseName,char *aDir=NULL,
		char *aExtension=NULL);


//=============================================================================
};	// END of class Decomp

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __Decomp_h__
