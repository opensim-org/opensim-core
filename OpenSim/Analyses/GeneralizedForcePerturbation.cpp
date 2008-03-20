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
/* Copyright (c)  2006 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/Model/AbstractCoordinate.h>
#include <OpenSim/Common/GCVSpline.h>
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
 * @param aModel Model for which actuator forces are to be perturbed.
 */
GeneralizedForcePerturbation::
GeneralizedForcePerturbation(Model *aModel) :
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
 * @param aModel Model for which actuator forces are to be perturbed.
 * @param aFunction Function that defines the generalized force to be applied.
 */
GeneralizedForcePerturbation::
GeneralizedForcePerturbation(Model *aModel, GCVSpline *_aSpline) :
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





