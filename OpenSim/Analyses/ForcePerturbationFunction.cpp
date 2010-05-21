// ForcePerturbationFunction.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Jack Middleton 
// 
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/StateFunction.h>
#include "ActuatorPerturbationIndependent.h"
#include "ActuatorPerturbation.h"
#include "ForcePerturbationFunction.h"


using namespace std;
namespace OpenSim {

   ForcePerturbationFunction::ForcePerturbationFunction() :
        _isPerturbed(false) {

        _perturbationMethod = 0;
        _actuator = 0;
        _unperturbedForce = 0;
   }

   ForcePerturbationFunction::ForcePerturbationFunction( Actuator* act, ActuatorPerturbationIndependent* perturb ):
        _actuator(act),
        _isPerturbed(false) {

        _perturbationMethod = perturb;
		_unperturbedForce = 0;
   }
   ForcePerturbationFunction::ForcePerturbationFunction( const ForcePerturbationFunction &forceFunction ) :
        _unperturbedForce(forceFunction._unperturbedForce),
        _actuator(forceFunction._actuator),
        _perturbationMethod(forceFunction._perturbationMethod),
        _isPerturbed(forceFunction._isPerturbed) {

   }


//_____________________________________________________________________________
/**
 * Clone
 *
 */

   Object*  ForcePerturbationFunction::copy() const {

       ForcePerturbationFunction *object = new ForcePerturbationFunction(*this);
       return(object);
    }


   void  ForcePerturbationFunction::setUnperturbedForce( GCVSpline* forceSpline) { _unperturbedForce = forceSpline; }

   double ForcePerturbationFunction::calcValue( const SimTK::State& s)  const {

   if( !_perturbationMethod ) {
      cout << "Warning no Perturbation object set for " << _actuator->getName() << " actuator force set to zero" << endl;
      return 0;
   }

   // COMPUTE PERTURBED FORCE
   double force = _unperturbedForce->calcValue( SimTK::Vector(1, s.getTime()) );



   if( _isPerturbed &&
       (s.getTime()>=_perturbationMethod->getStartTime()) && 
       (s.getTime()<=_perturbationMethod->getEndTime())){

	   if(_perturbationMethod->getPerturbationType()         == ActuatorPerturbationIndependent::SCALE) {
   	       force = force + _perturbationMethod->getPerturbationSize()*force;
	    } else if(_perturbationMethod->getPerturbationType() == ActuatorPerturbationIndependent::DELTA) {
	       force = force + _perturbationMethod->getPerturbationSize();
	    } else if(_perturbationMethod->getPerturbationType() == ActuatorPerturbationIndependent::CONSTANT) {
	       force = _perturbationMethod->getPerturbationSize();
	    } else {
	       printf("ForcePerturbationFunction.calcValue :WARN - unrecognized perturbationType.\n");
        }
   }

   	    // MAKE CORRECTION IF PERTRUBED FORCE IS NEGATIVE AND NEG FORCE FLAG IS SET
   if( !_perturbationMethod->getAllowNegForce()  && force < 0.0)	{
       force = 0; 
   }

   return( force );

}

};  // end namespace OpenSim

