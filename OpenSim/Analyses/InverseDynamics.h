#ifndef _InverseDynamics_h_
#define _InverseDynamics_h_
// InverseDynamics.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Eran Guendelman
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "osimAnalysesDLL.h"
#include <OpenSim/Simulation/Model/Analysis.h>
#include <SimTKcommon.h>


//=============================================================================
//=============================================================================
/**
 */
namespace OpenSim { 

class Model;
class GCVSplineSet;

class OSIMANALYSES_API InverseDynamics : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
private:

protected:
	Storage *_storage;

	Array<double> _dydt;
	Array<int> _accelerationIndices;

	SimTK::Matrix _performanceMatrix;
	SimTK::Vector _performanceVector;
	SimTK::Matrix _constraintMatrix;
	SimTK::Vector _constraintVector;
	SimTK::Vector _lapackWork;

	GCVSplineSet *_uSet;

//=============================================================================
// METHODS
//=============================================================================
public:
	InverseDynamics(Model *aModel=0);
	// Copy constrctor and virtual copy 
	InverseDynamics(const InverseDynamics &aObject);
	virtual Object* copy() const;
	virtual ~InverseDynamics();
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	InverseDynamics& operator=(const InverseDynamics &aInverseDynamics);
#endif
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();
	void deleteStorage();
	void computeAcceleration(double aT,double *aX,double *aY,double *aF,double *rAccel) const;

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getStorage();

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
		record(double aT,double *aX,double *aY,double *aDYDT);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	virtual int
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class InverseDynamics

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __InverseDynamics_h__
