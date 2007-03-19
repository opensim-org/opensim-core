#ifndef _GeneralizedForceAtv_h_
#define _GeneralizedForceAtv_h_
// GeneralizedForceAtv.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/Storage.h>
#include "GeneralizedForce.h"


//=============================================================================
//=============================================================================
/**
 * An actuator that exerts a generalized force and that incorporates
 * activation dynamics.  It has 1 control (excitation) and 1 state
 * (activation level).
 *
 * Controls: excitation
 * States: activation
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class OSIMACTUATORS_API GeneralizedForceAtv : public GeneralizedForce 
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Rise time of activation. */
	PropertyDbl _propRiseTime;
	/** Fall time of activation. */
	PropertyDbl _propFallTime;

	// REFERENCES
	double &_riseTime;
	double &_fallTime;

	/** Activation level (state 0). */
	double _a;

//=============================================================================
// METHODS
//=============================================================================
public:
	GeneralizedForceAtv(std::string aQName="");
	GeneralizedForceAtv(const GeneralizedForceAtv &aActuator);
	virtual ~GeneralizedForceAtv();
	virtual Object* copy() const;
private:
	void setNull();
	void setupProperties();
	void copyData(const GeneralizedForceAtv &aActuator);

public:

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	GeneralizedForceAtv&
		operator=(const GeneralizedForceAtv &aActuator);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// RISE TIME
	void setRiseTime(double aRiseTime);
	double getRiseTime() const;
	// FALL TIME
	void setFallTime(double aFallTime);
	double getFallTime() const;

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const double aX[],double aDT);
	virtual void computeActuation();
	virtual void computeStateDerivatives(double rDYDT[]);

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	virtual void updateFromXMLNode();

//=============================================================================
};	// END of class GeneralizedForceAtv

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __GeneralizedForceAtv_h__
