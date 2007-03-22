#ifndef _TorsionalSpring_h_
#define _TorsionalSpring_h_
// TorsionalSpring.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/VectorFunction.h>
#include "osimActuatorsDLL.h"
#include <OpenSim/Analyses/Contact.h>
#include "TorqueApplier.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external torques during a
 * simulation.  The magnitude of the torque is calculated based on the 
 * deviation in angular position and velocity of the body from a prescribed
 * position and velocity.
 *
 * @author Frank C. Anderson, Saryn Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractBody;

class OSIMACTUATORS_API TorsionalSpring : public TorqueApplier 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Vector function containing target orientation of the body. */
	VectorFunction *_targetPosition;
	/** Vector function containing target angular velocity of the body. */
	VectorFunction *_targetVelocity;
	/** Function containing values for the time-dependent torque scaling factor. */
	Function *_scaleFunction;
	/** Scale factor that pre-multiplies the applied torque */
	double _scaleFactor;
	/** Stiffness. */
	double _k[3];
	/** Damping. */
	double _b[3];
	/** Flag indicating whether or not to store the applied torques. */
	bool _storeTorques;

//=============================================================================
// METHODS
//=============================================================================
public:
	TorsionalSpring(Model *aModel, AbstractBody *aBody);
	virtual ~TorsionalSpring();
private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setTargetPosition(VectorFunction* aPosFunction);
	VectorFunction* getTargetPosition() const;
	void setTargetVelocity(VectorFunction* aVelFunction);
	VectorFunction* getTargetVelocity() const;
	void setKValue(double aK[3]);
	void getKValue(double aK[3]);
	void setBValue(double aB[3]);
	void getBValue(double aB[3]);
	void setScaleFunction(Function* _scaleFunction);
	Function* getScaleFunction() const;
	void setScaleFactor(double aScaleFactor);
	double getScaleFactor();
	void setStoreTorques(bool aTrueFalse);
	bool getStoreTorques();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void computeTargetFunctions(Storage *aQStore,Storage *aUStore);
	
	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		applyActuation(double aT,double *aX,double *aY);

//=============================================================================
};	// END of class TorsionalSpring

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __TorsionalSpring_h__
