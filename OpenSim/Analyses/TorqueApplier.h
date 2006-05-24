#ifndef _TorqueApplier_h_
#define _TorqueApplier_h_
// TorqueApplier.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, May Q. Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Tools/VectorFunction.h>
#include <OpenSim/Tools/FunctionSet.h>
#include "suAnalysesDLL.h"
#include "Contact.h"
#include "Decomp.h"


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

class SUANALYSES_API TorqueApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which body segment. */
	int _body;
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
	TorqueApplier(Model *aModel,int aBody);
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
	void setBody(int aBody);
	int getBody() const;
	void setTorque(double aTorque[3]);
	void getTorque(double rPoint[3]) const;

	void setTorqueFunction(VectorFunction* aTorqueFunction);
	VectorFunction* getTorqueFunction() const;

	void setRecordAppliedLoads(bool aTrueFalse);
	bool getRecordAppliedLoads() const;
	Storage* getAppliedTorqueStorage();
	void setStorageCapacityIncrements(int aIncrement);

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
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class TorqueApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __TorqueApplier_h__
