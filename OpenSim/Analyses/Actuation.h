#ifndef _Actuation_h_
#define _Actuation_h_
// Actuation.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
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
 * A class for recording the basic actuator information for a model
 * during a simulation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMANALYSES_API Actuation : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	/** Number of actuators. */
	int _na;
	/** Work array for storing forces, speeds, or powers. */
	double *_fsp;
	/** Force storage. */
	Storage *_forceStore;
	/** Speed storage. */
	Storage *_speedStore;
	/** Power storage. */
	Storage *_powerStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	Actuation(AbstractModel *aModel=0);
	Actuation(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	Actuation(const Actuation &aObject);
	virtual Object* copy() const;
	virtual ~Actuation();
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
	Actuation& operator=(const Actuation &aActuation);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getForceStorage() const;
	Storage* getSpeedStorage() const;
	Storage* getPowerStorage() const;
	// MODEL
	virtual void setModel(AbstractModel *aModel);

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
};	// END of class Actuation

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __Actuation_h__
