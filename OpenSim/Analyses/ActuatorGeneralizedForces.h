#ifndef _ActuatorGeneralizedForces_h_
#define _ActuatorGeneralizedForces_h_
// ActuatorGeneralizedForces.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include "osimAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class for recording the generalized force due to an actuator or set of actuators..
 *
 * @author Frank C. Anderson & Saryn Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class AbstractActuator;

class OSIMANALYSES_API ActuatorGeneralizedForces : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
public:


protected:
	double *_dqdt;
	double *_dudt;
	Array<AbstractActuator*> _actuatorList;
	double *_actuatorGenForces;
	Storage *_actuatorGenForcesStore;

	// Properties
	PropertyStrArray	_propActuatorNames;
	// REFERENCES
	Array<std::string>&	_actuatorNames;	
//=============================================================================
// METHODS
//=============================================================================
public:
	ActuatorGeneralizedForces(AbstractModel *aModel=0);
	ActuatorGeneralizedForces(const std::string &aFileName);
	// Copy constrctor and virtual copy 
	ActuatorGeneralizedForces(const ActuatorGeneralizedForces &aObject);
	virtual Object* copy() const;
	virtual ~ActuatorGeneralizedForces();
private:
	void setNull();
	void setupProperties();
	void constructDescription();
	void constructColumnLabels();
	void allocateStorage();

public:
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	ActuatorGeneralizedForces& operator=(const ActuatorGeneralizedForces &aActuatorGeneralizedForces);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// STORAGE
	void setStorageCapacityIncrements(int aIncrement);
	Storage* getActuatorGenForcesStorage();

	virtual void setModel(AbstractModel *aModel);
	void setActuatorList(const Array<std::string>& aActuatorNames);
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
};	// END of class ActuatorGeneralizedForces

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ActuatorGeneralizedForces_h__
