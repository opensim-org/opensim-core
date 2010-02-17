#include "SimTKsimbody.h"
#include "OpenSimForceSubsystem.h"
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/Ligament.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Analyses/ActuatorPerturbation.h>


using namespace OpenSim;
using namespace std;

OpenSimForceSubsystemRep::OpenSimForceSubsystemRep( Model* model) :  
        SimTK::Subsystem::Guts( "OpenSimForceSubsystem", "2.0"), 
        _model(model){}

  int&  OpenSimForceSubsystemRep::updStep(const SimTK::State& s) const {
     return (SimTK::Value<int>::downcast(s.updCacheEntry( getMySubsystemIndex(), _stepIndex)).upd() ); 
  }
  int  OpenSimForceSubsystemRep::getStep(const SimTK::State& s) const {
     return (SimTK::Value<int>::downcast(s.getCacheEntry( getMySubsystemIndex(), _stepIndex)).get() ); 
  }

  int OpenSimForceSubsystemRep::realizeSubsystemTopologyImpl(SimTK::State& s) const {

	  _stepIndex =  s.allocateCacheEntry( getMySubsystemIndex(), SimTK::Stage::Instance, new SimTK::Value<int>(0) );

      Actuator *act;
	  for(int i=0;i<_model->getForceSet().getSize();i++)  {
          act = dynamic_cast<Actuator*>( &_model->getForceSet().get(i) ); 
          if( act ) act->initStateCache(s, getMySubsystemIndex(), *_model );
			 else {
				 Ligament* lig = dynamic_cast<Ligament*>( &_model->getForceSet().get(i) ); 
				 if( lig ) lig->initStateCache(s, getMySubsystemIndex(), *_model );
			 }
	  }

      return 0;
  }
  int OpenSimForceSubsystemRep::realizeSubsystemVelocityImpl(const SimTK::State& s) const {

      // If we are doing perturbation, set the forces for each actuator 
      if( _model->getPerturbForcesEnabled() ) {
          _model->getPerturbation().setForces(s);
      }
      return 0;
  }

  
  int OpenSimForceSubsystemRep::realizeSubsystemAccelerationImpl(const SimTK::State& s) const {
/*
      Actuator *act;
std::cout << " q= " << s.getQ()  << std::endl;
std::cout << " u= " << s.getU()  << std::endl;
std::cout << " qdot= " << s.getQDot()  << std::endl;
//std::cout << " udot= " << s.getUDot()  << std::endl;
	  for(int i=0;i<_model->getForceSet().getSize();i++)  {
          act = dynamic_cast<Actuator*>( _model->getForceSet().get(i) ); 
          if( act ) {
              printf(" t=%16.14g ", s.getTime() ); 
              std::cout << act->getName() << " Force = " <<   act->getForce(s)  << std::endl;
          }
	  }
*/
	  _model->updForceSet().computeStateDerivatives( s );
	  return 0;
  }

  OpenSimForceSubsystem::OpenSimForceSubsystem( SimTK::MultibodySystem& sys, Model* model) {
        adoptSubsystemGuts( rep = new OpenSimForceSubsystemRep( model)  );
        int subIndex = sys.adoptSubsystem( *this );
  }

