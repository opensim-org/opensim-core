#ifndef _MuscleAnalysis_h_
#define _MuscleAnalysis_h_
// MuscleAnalysis.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Katherine Holzbaur, Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


#ifdef SWIG
	#ifdef OSIMANALYSES_API
		#undef OSIMANALYSES_API
		#define OSIMANALYSES_API
	#endif
#endif
//=============================================================================
//=============================================================================
/**
 * A class for recording the muscle actuator information for a model
 * during a simulation.
 *
 * @author Katherine Holzbaur, Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMANALYSES_API MuscleAnalysis : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Pennation angle storage. */
	Storage *_pennationAngleStore;
	/** Muscle-tendon length storage. */
	Storage *_lengthStore;
	/** Fiber length storage. */
	Storage *_fiberLengthStore;
	/** Normalized fiber length storage. */
	Storage *_normalizedFiberLengthStore;
	/** Tendon length storage. */
	Storage *_tendonLengthStore;
	/** Force applied by the muscle. */
	Storage *_forceStore;
	/** Force in the muscle fibers. */
	Storage *_fiberForceStore;
	/** Active force in the muscle fibers. */
	Storage *_activeFiberForceStore;
	/** Passive force in the muscle fibers. */
	Storage *_passiveFiberForceStore;
	/** Active force in the muscle fibers along tendon. */
	Storage *_activeFiberForceAlongTendonStore;
	/** Passive force in the muscle fibers along tendon. */
	Storage *_passiveFiberForceAlongTendonStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	MuscleAnalysis(Model *aModel=0);
	MuscleAnalysis(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	MuscleAnalysis(const MuscleAnalysis &aObject);
	virtual Object* copy() const;
	virtual ~MuscleAnalysis();
private:
	void setNull();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	MuscleAnalysis& operator=(const MuscleAnalysis &aMuscleAnalysis);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model *aModel);
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getPennationAngleStorage() const { return _pennationAngleStore; }
	Storage* getMuscleTendonLengthStorage() const { return _lengthStore; }
	Storage* getFiberLengthStorage() const { return _fiberLengthStore; }
	Storage* getNormalizedFiberLengthStorage() const { return _normalizedFiberLengthStore; }
	Storage* getTendonLegthStorage() const { return _tendonLengthStore; }
	Storage* getForceStorage() const { return _forceStore; }
	Storage* getFiberForceStorage() const { return _fiberForceStore; }
	Storage* getActiveFiberForceStorage() const { return _activeFiberForceStore; }
	Storage* getPassiveFiberForceStorage() const { return _passiveFiberForceStore; }
	Storage* getActiveFiberForceAlongTendonStorage() const { return _activeFiberForceAlongTendonStore; }
	Storage* getPassiveFiberForceAlongTendonStorage() const { return _passiveFiberForceAlongTendonStore; }

	//--------------------------------------------------------------------------
	// ANALYSIS
	//--------------------------------------------------------------------------
	virtual int
		begin(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		step(double *aXPrev,double *aYPrev,double *aYPPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
	virtual int
		end(int aStep,double aDT,double aT,
		double *aX,double *aY,double *aYP=NULL,double *aDYDT=NULL,void *aClientData=NULL);
protected:
	virtual int
		record(double aT,double *aX,double *aY);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

	OPENSIM_DECLARE_DERIVED(MuscleAnalysis,Analysis)
//=============================================================================
};	// END of class MuscleAnalysis

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MuscleAnalysis_h__
