#ifndef _ActuatedModelIntegCallback_h_
#define _ActuatedModelIntegCallback_h_
// ActuatedModelIntegCallback.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/IntegCallback.h>
#include "ActuatedModel_SDFast.h"


//=============================================================================
//=============================================================================
/**
 * A integration callback used to simulate a slip.  This is done by changing
 * the friction coefficients of contact forces during a simulation.
 * Specifically, between the start and end time of the callback
 * (see Callback::setStartTime() and Callback::setEndTime()), the
 * coefficients of friction for all contact forces is set to a specified
 * value (muSlip).  Otherwise, the coefficients of friction for all
 * contact forces is set to a normal value (mu).
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class ActuatedModelIntegCallback : public IntegCallback 
{
//=============================================================================
// DATA
//=============================================================================
private:


//=============================================================================
// METHODS
//=============================================================================
public:
	ActuatedModelIntegCallback(ActuatedModel_SDFast *aModel);
	virtual ~ActuatedModelIntegCallback();
private:
	void setNull();

public:

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual int
		step(double *aXPrev,double *aYPrev,int aStep,double aDT,double aT,
		double *aX,double *aY,void *aClientData=NULL);

//=============================================================================
};	// END of class ActuatedModelIntegCallback

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ActuatedModelIntegCallback_h__
