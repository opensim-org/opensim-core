#ifndef _ForceApplier_h_
#define _ForceApplier_h_
// ForceApplier.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, May Q. Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/VectorFunction.h>
#include <OpenSim/Common/FunctionSet.h>
#include "osimActuatorsDLL.h"
#include <OpenSim/Analyses/Contact.h>


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external forces during a
 * simulation.
 *
 * @author Frank C. Anderson, May Q. Liu
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMACTUATORS_API ForceApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which body segment. */
	AbstractBody* _body;
	/** Point of force application. */
	double _point[3];
	/** Force to be applied. */
	double _force[3];
	/** VectorFunction containing points of force application (t,x,y,z). */
	VectorFunction* _pointFunction;
	/** VectorFunction containing force to be applied (t,x,y,z). */
	VectorFunction* _forceFunction;
	/** Flag to set reference frame of input force */
	bool _inputForcesInGlobalFrame;
	/** Flag to indicate whether or not to record the loads that are applied
	during an integration.  Recording these loads takes a lot of memory as they
	are stored every time the derivatives are evaluated (e.g., 6 times per
	integration step). */
	bool _recordAppliedLoads;
	/** Storage for the force that was actually applied during the simulation */
	Storage *_appliedForceStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	ForceApplier(Model *aModel,AbstractBody *aBody);	
	ForceApplier(Model *aModel,AbstractBody *bodyFrom,AbstractBody *bodyTo,
		Storage *forceData,int fxNum,int fyNum,int fzNum,
		int pxNum,int pyNum,int pzNum,Storage *aQStore,Storage *aUStore);
	virtual ~ForceApplier();
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
	void setPoint(double aPoint[3]);
	void getPoint(double rPoint[3]) const;
	void setForce(double aForce[3]);
	void getForce(double rPoint[3]) const;

	void setForceFunction(VectorFunction* aForceFunction);
	VectorFunction* getForceFunction() const;
	void setPointFunction(VectorFunction* aPointFunction);
	VectorFunction* getPointFunction() const;

	void setInputForcesInGlobalFrame(bool aTrueFalse);
	bool getInputForcesInGlobalFrame() const;

	void setRecordAppliedLoads(bool aTrueFalse);
	bool getRecordAppliedLoads() const;
	Storage* getAppliedForceStorage();
	void setStorageCapacityIncrements(int aIncrement);

	virtual void reset(); 
	
	void computePointFunction(Storage *aQStore,Storage *aUStore,
		VectorFunction &aPGlobal);

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
};	// END of class ForceApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ForceApplier_h__
