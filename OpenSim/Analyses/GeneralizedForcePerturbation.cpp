// GeneralizedForcePerturbation.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Frank C. Anderson and Saryn Goldberg
// 
// NOTE:  When you use this callBack to make a perturbation the actuator force,
//        this change in the force IS NOT recoreded in the state file.  If you
//			 want to run an induced accleration analysis using results from a 
//			 perturbation, you must first alter the states fiel to accurately
//			 reflect the changes made to the forces.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Tools/rdMath.h>
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/AbstractCoordinate.h>
#include <OpenSim/Tools/GCVSpline.h>
#include "GeneralizedForcePerturbation.h"

//=============================================================================
// CONSTANTS
//=============================================================================



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________


using namespace OpenSim;
/**
 * Destructor.
 */
GeneralizedForcePerturbation::~GeneralizedForcePerturbation()
{
}
//_____________________________________________________________________________
/**
 * Construct a derivative callback instance for perturbing a generalized
 * force during an integration.
 * When this default constructor is used, the perturbation for to the
 * generalized force is set to be a unit torque.  This can then be scaled
 * if the user wants to apply a constant torque to a generalized coordinate.
 *
 * @param aModel AbstractModel for which actuator forces are to be perturbed.
 */
GeneralizedForcePerturbation::
GeneralizedForcePerturbation(AbstractModel *aModel) :
	DerivCallback(aModel)
{
	setNull();

	// BASE-CLASS MEMBER VARIABLES
	setType("GeneralizedForcePerturbation");
}

/**
 * Construct a derivative callback instance for perturbing a generalized
 * force during an integration.
 * When this constructor is used, the Storage sent in by the user is used
 * to define a function object. This function is used to define the 
 * generalized force that is applied during the simulation.
 *
 * @param aModel AbstractModel for which actuator forces are to be perturbed.
 * @param aFunction Function that defines the generalized force to be applied.
 */
GeneralizedForcePerturbation::
GeneralizedForcePerturbation(AbstractModel *aModel, GCVSpline *_aSpline) :
	DerivCallback(aModel)
{
	setNull();

	_genForceSpline = _aSpline;

	// BASE-CLASS MEMBER VARIABLES
	setType("GeneralizedForcePerturbation");
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set member variables to approprate NULL values.
 */
void GeneralizedForcePerturbation::
setNull()
{
	_genCoord = NULL;
	_perturbation = 0.0;
	_genForceSpline = NULL;
	_scaleFactor = 1.0;
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// ACTUATOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set which generalized coordinate the force perturbation should be made.
 *
 * @param aCoord Pointer to the generalized coordinate.
 */
void GeneralizedForcePerturbation::
setGenCoord(AbstractCoordinate *aCoord)
{
	_genCoord = aCoord;
}
//_____________________________________________________________________________
/**
 * Get which generalized coordinate the force perturbation should be made.
 *
 * @return Generalized coordiante to which force is applied..
 */
AbstractCoordinate *GeneralizedForcePerturbation::
getGenCoord() const
{
	return(_genCoord);
}

//-----------------------------------------------------------------------------
// PERTURBATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the scale factor by which the perturbation will be multiplied.
 *
 * @param aScaleFactor Factor by which perturbation is multiplied
 */
void GeneralizedForcePerturbation::
setScaleFactor(double aScaleFactor)
{
	_scaleFactor = aScaleFactor;
	printf("scale factor set\n");
}
//_____________________________________________________________________________
/**
 * Get the scale factor by which the perturbation will be multiplied.
 *
 * @return  Factor by which perturbation is multiplied
 */
double GeneralizedForcePerturbation::
getScaleFactor() const
{
	return(_scaleFactor);
}
//_____________________________________________________________________________
/**
 * Set the perturbation.  This is only neccesary if you want to apply
 * a constant perturbation that does not come from a spline.
 *
 * @param aPerturbation Value of constatn perturbation
 */
void GeneralizedForcePerturbation::
setPerturbation(double aPerturbation)
{
	_perturbation = aPerturbation;
}
//_____________________________________________________________________________
/**
 * Get the pertubation value.
 *
 * @return  Factor by which perturbation is multiplied
 */
double GeneralizedForcePerturbation::
getPerturbation() const
{
	return(_perturbation);
}


//-----------------------------------------------------------------------------
// FORCE STORAGE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

//=============================================================================
// CALLBACKS
//=============================================================================
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been computed by the model.
 *
 * The perturbation generalized force is applied to the generalized coordinate.
 *
 * @param aT Normalized time.
 * @param aX Controls.
 * @param aY States.
 */
void GeneralizedForcePerturbation::
computeActuation(double aT,double *aX,double *aY)
{
	if(_model==NULL) {
		printf("GeneralizedForcePerturbation.computeActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	// COMPUTE PERTURBED GENERALIZED FORCE

	if((aT>=getStartTime()) && (aT<getEndTime())){
		if( _genForceSpline != NULL){
			_perturbation = _scaleFactor * 
				_genForceSpline->evaluate(0,aT*_model->getTimeNormConstant());
//			printf("time = %f\tspline perturbation = %f\n",aT*_model->getTimeNormConstant(),_perturbation);
		}
		else {
			_perturbation = _scaleFactor * _perturbation;
//			printf("constant perturbation = %f\n",_perturbation);
		}
	}	
}
//_____________________________________________________________________________
/**
 * Callback called right after actuation has been applied by the model.
 *
 * The nominal atuator force is restored.
 */
void GeneralizedForcePerturbation::
applyActuation(double aT,double *aX,double *aY)
{
	if(_model==NULL) {
		printf("ActuatorPerturbation.applyActuation: WARN- no model.\n");
		return;
	}
	if(!getOn()) return;

	if((aT>=getStartTime()) && (aT<getEndTime())){
		_model->getDynamicsEngine().applyGeneralizedForce(*_genCoord, _perturbation);	
//		printf("time = %f\t apply perturbation = %f\n",aT*_model->getTimeNormConstant(),_perturbation);
	}
}





