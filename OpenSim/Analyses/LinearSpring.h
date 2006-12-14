#ifndef _LinearSpring_h_
#define _LinearSpring_h_
// LinearSpring.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Tools/Function.h>
#include <OpenSim/Tools/VectorFunction.h>
#include "suAnalysesDLL.h"
#include "Contact.h"
#include "ForceApplier.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying external forces during a
 * simulation.
 *
 * @author Frank C. Anderson, Saryn R. Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;
class AbstractBody;

class SUANALYSES_API LinearSpring : public ForceApplier 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Vector function containing the target position of the point expressed
	in the global reference frame. */
	VectorFunction *_targetPosition;
	/** Vector function containing the target velocity of the point expressed
	in  the global reference frame. */
	VectorFunction *_targetVelocity;
	/** Function containing values for the time-dependent scaling factor. */
	Function *_scaleFunction;
	/** Scale factor that pre-multiplies the applied torque */
	double _scaleFactor;
	/** Stiffness. */
	double _k[3];
	/** Damping. */
	double _b[3];
	/** Flag indicating whether or not to store the applied forces. */
	bool _storeForces;

//=============================================================================
// METHODS
//=============================================================================
public:
	LinearSpring(AbstractModel *aModel, AbstractBody *aBody);	
	virtual ~LinearSpring();
private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setTargetPosition(VectorFunction* aTargetVelocity);
	VectorFunction* getTargetPosition() const;
	void setTargetVelocity(VectorFunction* aTargetVelocity);
	VectorFunction* getTargetVelocity() const;
	void setKValue(double aK[3]);
	void getKValue(double aK[3]);
	void setBValue(double aB[3]);
	void getBValue(double aB[3]);
	void setScaleFunction(Function* _scaleFunction);
	Function* getScaleFunction() const;
	void setScaleFactor(double aScaleFactor);
	double getScaleFactor();
	void setStoreForces(bool aTrueFalse);
	bool getStoreForces();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	void
		computePointAndTargetFunctions(Storage *aQStore,Storage *aUStore,
		VectorFunction &aPGlobal);

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		applyActuation(double aT,double *aX,double *aY);

//=============================================================================
};	// END of class LinearSpring

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __LinearSpring_h__
