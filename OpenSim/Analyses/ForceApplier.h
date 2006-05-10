#ifndef _ForceApplier_h_
#define _ForceApplier_h_
// ForceApplier.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, May Q. Liu
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>
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
 * A derivatives callback used for applying external forces during a
 * simulation.
 *
 * @author Frank C. Anderson, May Q. Liu
 * @version 1.0
 */
namespace OpenSim { 

class SUANALYSES_API ForceApplier : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Which body segment. */
	int _body;
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
	/** Storage for the force that was actually applied during the simulation */
	Storage *_appliedForceStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	ForceApplier(Model *aModel,int aBody);	
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
	void setBody(int aBody);
	int getBody() const;
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

	Storage* getAppliedForceStorage();
	void setStorageCapacityIncrements(int aIncrement);

	virtual void reset(); 
	
	void
		computePointFunction(Storage *aQStore,Storage *aUStore,
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
		printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");


//=============================================================================
};	// END of class ForceApplier

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ForceApplier_h__
