#ifndef _ActuatorPerturbation_h_
#define _ActuatorPerturbation_h_
// ActuatorPerturbation.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn R. Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "osimAnalysesDLL.h"
#include "Contact.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for perturbing the actuator forces during a
 * simulation.
 *
 * @author Frank C. Anderson, Saryn R. Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class AbstractModel;

class OSIMANALYSES_API ActuatorPerturbation : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
public:
	/** Perturbation types. See setPerturbation(). */
	enum PertType {SCALE, DELTA, CONSTANT};

protected:
	/** Which actuator. */
	AbstractActuator *_actuator;
	/** Negative force flag **/
	bool _allowNegForce;
	/** Force perturbation. */
	double _perturbation;
	/** Type of perturbation */
	PertType _perturbationType;
	/** Nominal actuator force. */
	double _force;
	/** Storage for holding nominal and perturbed force. */
	Storage *_forceStore;

//=============================================================================
// METHODS
//=============================================================================
public:
	ActuatorPerturbation(AbstractModel *aModel);
	virtual ~ActuatorPerturbation();
private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setActuator(AbstractActuator *aActuator);
	AbstractActuator* getActuator() const;
	void setAllowNegForce(bool aTrueFalse);
	bool getAllowNegForce() const;
	void setPerturbation(PertType aPerturbationType, double aPerturbation);
	double getPerturbation() const;
	PertType getPerturbationType() const;
	Storage* getForceStorage();

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void reset(); 

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		computeActuation(double aT,double *aX,double *aY);
	virtual void
		applyActuation(double aT,double *aX,double *aY);

//=============================================================================
};	// END of class ActuatorPerturbation

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ActuatorPerturbation_h__
