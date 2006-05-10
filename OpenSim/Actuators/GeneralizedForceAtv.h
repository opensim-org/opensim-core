#ifndef _GeneralizedForceAtv_h_
#define _GeneralizedForceAtv_h_
// GeneralizedForceAtv.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "Actuators.h"
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/PropertyDbl.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Simulation/Model/GeneralizedForce.h>


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

class RDACTUATORS_API GeneralizedForceAtv : public GeneralizedForce 
{
//=============================================================================
// DATA
//=============================================================================
private:
	static const std::string X_NAME;
	static const std::string Y_NAME;
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
	GeneralizedForceAtv(int aQID=-1,int aNX=1,int aNY=1,int aNYP=0);
	GeneralizedForceAtv(DOMElement *aElement,
		int aNX=1,int aNY=1,int aNYP=0);
	GeneralizedForceAtv(const GeneralizedForceAtv &aActuator);
	virtual ~GeneralizedForceAtv();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
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
	// STATES
	virtual int getNY() const;
	virtual const std::string& getStateName(int aIndex) const;
	virtual int getStateIndex(const std::string &aName) const;
	virtual void setStates(const double aX[]);
	virtual void setState(int aIndex,double aValue);
	virtual void setState(const std::string &aName,double aValue);
	virtual void getStates(double rX[]) const;
	virtual double getState(int aIndex) const;
	virtual double getState(const std::string &aName) const;
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
