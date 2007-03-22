#ifndef _GeneralizedForcePerturbation_h_
#define _GeneralizedForcePerturbation_h_
// GeneralizedForcePerturbation.h
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
#include <OpenSim/Common/GCVSpline.h>
#include "Contact.h"


//=============================================================================
//=============================================================================
/**
 * A derivatives callback used for applying a generalized force during a
 * simulation.
 *
 * @author Frank C. Anderson, Saryn R. Goldberg
 * @version 1.0
 */
namespace OpenSim { 

class Model;
class AbstractCoordinate;

class OSIMANALYSES_API GeneralizedForcePerturbation : public DerivCallback 
{
//=============================================================================
// DATA
//=============================================================================
public:

protected:
	/** Which generalized coordinate. */
	AbstractCoordinate *_genCoord;
	/** Perturbation to be applied */
	double _perturbation;
	/** Scaling factor to be applied to perturbation */
	double _scaleFactor;
   /** The function defining the generalized force to be applied **/
	GCVSpline *_genForceSpline;


//=============================================================================
// METHODS
//=============================================================================
public:
	GeneralizedForcePerturbation(Model *aModel);
	GeneralizedForcePerturbation(Model *aModel, GCVSpline *_aSpline);
	virtual ~GeneralizedForcePerturbation();
private:
	void setNull();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	void setGenCoord(AbstractCoordinate *aCoord);
	AbstractCoordinate *getGenCoord() const;

	void setScaleFactor(double aScaleFactor);
	double getScaleFactor() const;
	void setPerturbation(double aPerturbation);
	double getPerturbation() const;

	
	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	virtual void
		computeActuation(double aT,double *aX,double *aY);
	virtual void
		applyActuation(double aT,double *aX,double *aY);

//=============================================================================
};	// END of class GeneralizedForcePerturbation

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __GeneralizedForcePerturbation_h__
