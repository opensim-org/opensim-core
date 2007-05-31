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

class OSIMANALYSES_API MomentArmAnalysis : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** List of muscles for which to compute moment arms.  If the list
	is left blank, moment arms are computed for all muscles. */
	PropertyStrArray _muscleListProp;

protected:
	/** Array of storage pointers for the moment arms. */
	ArrayPtrs<Storage> _momentArmStorageArray;

	/** Work array for holding the list of muscles.  This is used
	in the event the user-specified muscle list is empty, which implies
	that all muscles should be analyzed. */
	Array<std::string> _muscleList;

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
	void allocateStorage();

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
	const ArrayPtrs<Storage>& getMomentArmStorageArray() const { return _momentArmStorageArray; }

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

//=============================================================================
};	// END of class MomentArmAnalysis

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __MomentArmAnalysis_h__
