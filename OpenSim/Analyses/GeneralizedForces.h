#ifndef _GeneralizedForces_h_
#define _GeneralizedForces_h_
// GeneralizedForces.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class for recording the joint torques of the generalized coordinates
 * of a model during a simulation.
 *
 * @author Frank C. Anderson & Saryn Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class Model;

class OSIMANALYSES_API GeneralizedForces : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	double *_dqdt;
	double *_dudt;
	double *_zero_aY;
	double *_gravGenForces;
	double *_velGenForces;
	double *_actuatorGenForces;
	double *_contactGenForces;
	Storage *_gravGenForcesStore;
	Storage *_velGenForcesStore;
	Storage *_actuatorGenForcesStore;
	Storage *_contactGenForcesStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	GeneralizedForces(Model *aModel=0);
	GeneralizedForces(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	GeneralizedForces(const GeneralizedForces &aObject);
	virtual Object* copy() const;
	virtual ~GeneralizedForces();
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	GeneralizedForces& operator=(const GeneralizedForces &aGeneralizedForces);
#endif
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
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getGravGenForcesStorage();
	Storage* getVelGenForcesStorage();
	Storage* getActuatorGenForcesStorage();
	Storage* getContactGenForcesStorage();

	virtual void setModel(Model *aModel);
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
};	// END of class GeneralizedForces

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __GeneralizedForces_h__
