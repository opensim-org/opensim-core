#ifndef _GeneralizedForcePerturbation_h_
#define _GeneralizedForcePerturbation_h_
// GeneralizedForcePerturbation.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson, Saryn R. Goldberg
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/Model/DerivCallback.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include "suAnalysesDLL.h"
#include <OpenSim/Tools/GCVSpline.h>
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

class AbstractModel;
class AbstractCoordinate;

class SUANALYSES_API GeneralizedForcePerturbation : public DerivCallback 
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
	GeneralizedForcePerturbation(AbstractModel *aModel);
	GeneralizedForcePerturbation(AbstractModel *aModel, GCVSpline *_aSpline);
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
