#ifndef _ActuatorGeneralizedForces_h_
#define _ActuatorGeneralizedForces_h_
// ActuatorGeneralizedForces.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson & Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "suAnalysesDLL.h"


//=============================================================================
//=============================================================================
/**
 * A class for recording the generalized force due to an actuator or set of actuators..
 *
 * @author Frank C. Anderson & Saryn Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class SUANALYSES_API ActuatorGeneralizedForces : public Analysis 
{
//=============================================================================
// DATA
//=============================================================================
public:


protected:
	double *_dqdt;
	double *_dudt;
	Array<int> _actuatorList;
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
	ActuatorGeneralizedForces(Model *aModel=0);
	ActuatorGeneralizedForces(const std::string &aFileName);
	ActuatorGeneralizedForces(DOMElement *aElement);
	// Copy constrctor and virtual copy 
	ActuatorGeneralizedForces(const ActuatorGeneralizedForces &aObject);
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
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

	virtual void setModel(Model *aModel);
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
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class ActuatorGeneralizedForces

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ActuatorGeneralizedForces_h__
