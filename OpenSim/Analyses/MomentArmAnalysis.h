#ifndef _MomentArmAnalysis_h_
#define _MomentArmAnalysis_h_
// MomentArmAnalysis.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Simulation/Model/AbstractMuscle.h>
#include "osimAnalysesDLL.h"

#ifdef SWIG
	#ifdef OSIMANALYSES_API
		#undef OSIMANALYSES_API
		#define OSIMANALYSES_API
	#endif
#endif


namespace OpenSim {


//=============================================================================
//=============================================================================
/**
 * A class for recording the muscle actuator information for a model
 * during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMANALYSES_API MomentArmAnalysis : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

	// STRUCT FOR PAIRING MOMENT ARM STORAGE OBJECTS WITH THEIR
	// ASSOCIATED GENERALIZED COORDINATE
	typedef struct {
		AbstractCoordinate *q;
		Storage *store;
	}  StorageCoordinatePair;

	/** List of muscles for which to compute moment arms.  The key word "all" 
	is used to indicate that the analysis should be run for all muscles. */
	PropertyStrArray _muscleListProp;

	/** List of generalized coordinates for which to compute moment arms. */
	PropertyStrArray _coordinateListProp;

	/** Work array for holding the list of muscles.  This array differs from
	the property above in that the key work "all" e */
	Array<std::string> _muscleList;

	/** Work array for holding the list of coordinates. */
	Array<std::string> _coordinateList;

	/** Array of active storage and coordinate pairs. */
	ArrayPtrs<StorageCoordinatePair> _momentArmStorageArray;

	/** Array of active muscles. */
	ArrayPtrs<AbstractMuscle> _muscleArray;

//=============================================================================
// METHODS
//=============================================================================
public:
	MomentArmAnalysis(Model *aModel=0);
	MomentArmAnalysis(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	MomentArmAnalysis(const MomentArmAnalysis &aObject);
	virtual Object* copy() const;
	virtual ~MomentArmAnalysis();
private:
	void setNull();
	void setupProperties();
	void constructColumnLabels();
	void allocateStorageObjects();
	void updateStorageObjects();

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	MomentArmAnalysis& operator=(const MomentArmAnalysis &aMomentArmAnalysis);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model *aModel);
	void setStorageCapacityIncrements(int aIncrement);
	const ArrayPtrs<StorageCoordinatePair>& getMomentArmStorageArray() const { return _momentArmStorageArray; }
	void setMuscles(Array<std::string>& aMuscles);
	void setCoordinates(Array<std::string>& aCoordinates);
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

	OPENSIM_DECLARE_DERIVED(MomentArmAnalysis,Analysis)	

//=============================================================================
};	// END of class MomentArmAnalysis

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MomentArmAnalysis_h__
