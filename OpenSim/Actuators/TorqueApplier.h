#ifndef _TorqueApplier_h_
#define _TorqueApplier_h_
// TorqueApplier.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, May Q. Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/VectorFunction.h>
#include <OpenSim/Common/FunctionSet.h>
#include "osimActuatorsDLL.h"
#include <OpenSim/Analyses/Contact.h>


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external torques during a
 * simulation.
 *
 * @author Frank C. Anderson, May Q. Liu
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMACTUATORS_API TorqueApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which body segment. */
	AbstractBody* _body;
	/** Torque to be applied. */
	double _torque[3];
	/** Vector function containing torque to be applied (t,x,y,z). */
	VectorFunction* _torqueFunction;
	/** Flag to set reference frame of input torque */
	bool _inputTorquesInGlobalFrame;
	/** Flag to indicate whether or not to record the loads that are applied
	during an integration.  Recording these loads takes a lot of memory as they
	are stored every time the derivatives are evaluated (e.g., 6 times per
	integration step). */
	bool _recordAppliedLoads;
	/** Storage for the torque that was actually applied during the simulation */
	Storage *_appliedTorqueStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	TorqueApplier(Model *aModel,AbstractBody *aBody);
	TorqueApplier(Model *aModel,AbstractBody *bodyFrom,AbstractBody *bodyTo,
		Storage *torqueData,int txNum, int tyNum, int tzNum);
	virtual ~TorqueApplier();
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
	void setBody(AbstractBody *aBody);
	AbstractBody* getBody() const;
	void setTorque(double aTorque[3]);
	void getTorque(double rPoint[3]) const;

	void setTorqueFunction(VectorFunction* aTorqueFunction);
	VectorFunction* getTorqueFunction() const;

	void setRecordAppliedLoads(bool aTrueFalse);
	bool getRecordAppliedLoads() const;
	Storage* getAppliedTorqueStorage();
	void setStorageCapacityIncrements(int aIncrement);

	virtual void reset(); 

	void setInputTorquesInGlobalFrame(bool aTrueFalse);
	bool getInputTorquesInGlobalFrame() const;
	
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
		printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class TorqueApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __TorqueApplier_h__
