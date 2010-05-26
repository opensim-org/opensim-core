#ifndef _Force_Perturbation_Function_h_
#define _Force_Perturbation_Function_h_
// ForcePerturbationFunc.h
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


namespace OpenSim {
using namespace std;

class ActuatorPerturbationIndependent;

class ForcePerturbationFunction : public StateFunction {

   private:
   bool _isPerturbed;
   Actuator*  _actuator;
   ActuatorPerturbationIndependent *_perturbationMethod;
   GCVSpline* _unperturbedForce;

   public:
   ForcePerturbationFunction();
   ForcePerturbationFunction( const ForcePerturbationFunction &forceFunction );
   ForcePerturbationFunction( Actuator* act, ActuatorPerturbationIndependent* perturb );
   virtual Object* copy() const; 

   virtual void setIsPerturbed( bool flag ) { _isPerturbed = flag; }
   virtual void setUnperturbedForce( GCVSpline* forceSpline);
   virtual const Actuator& getActuator() const { return *_actuator; }
   virtual Actuator& updActuator() { return  *_actuator; }

   virtual double calcValue( const SimTK::State& s)  const;

};
}; // end namespace
#endif // #ifndef __Force_Perturbation_Function_

