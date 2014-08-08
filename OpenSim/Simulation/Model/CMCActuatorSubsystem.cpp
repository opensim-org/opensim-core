/* -------------------------------------------------------------------------- *
 *                     OpenSim:  CMCActuatorSubsystem.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "CMCActuatorSubsystem.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <math.h>
#include <stdio.h>
using namespace OpenSim;
using namespace SimTK;
using namespace std;

void CMCActuatorSubsystemRep::setCompleteState(const SimTK::State& state) {
    _completeState = state;
}
const SimTK::State& CMCActuatorSubsystemRep::getCompleteState() const{
    return(_completeState);
}
FunctionSet* CMCActuatorSubsystemRep::getCoordinateTrajectories() const {
   return( _qSet);
}
FunctionSet* CMCActuatorSubsystemRep::getSpeedTrajectories() const {
   return( _uSet);
}

void CMCActuatorSubsystemRep::setCoordinateCorrections(const double* aCorrections) {
    int i ;
    int size = _qCorrections.getSize();
    for(i=0;i<size;i++) {
         _qCorrections[i] = aCorrections[i];
    }
  
}

void CMCActuatorSubsystemRep::setSpeedCorrections(const double* aCorrections) {
    int i ;
    int size = _uCorrections.getSize();
    for(i=0;i<size;i++) {
         _uCorrections[i] = aCorrections[i];
    }
}
  
void CMCActuatorSubsystemRep::setCoordinateTrajectories(FunctionSet *aSet) {
    // ERROR CHECKING
    if(aSet == NULL) {
        string msg = "CMCActuatorSubsystemRep.setCoordinateTrajectories:";
        msg += " ERR- NULL function set.\n";
        throw( Exception(msg,__FILE__,__LINE__) );
    }
    if(aSet->getSize() != _model->getNumCoordinates()) {
        string msg = "CMCActuatorSubsystemRep.setCoordinateTrajectories:";
        msg += " ERR- incorrect number of trajectories.\n";
        throw( Exception(msg,__FILE__,__LINE__) );
    }

    _qSet = aSet;
}
void CMCActuatorSubsystemRep::setSpeedTrajectories(FunctionSet *aSet) {
    // ERROR CHECKING
    if(aSet == NULL) {
        string msg = "CMCActuatorSubsystemRep.setSpeedTrajectories:";
        msg += " ERR- NULL function set.\n";
        throw( Exception(msg,__FILE__,__LINE__) );
    }
    if(aSet->getSize() != _model->getNumSpeeds()) {
        string msg = "CMCActuatorSubsystemRep.setSpeedTrajectories:";
        msg += " ERR- incorrect number of trajectories.\n";
        throw( Exception(msg,__FILE__,__LINE__) );
    }

    _uSet = aSet;
}
   CMCActuatorSubsystemRep::CMCActuatorSubsystemRep(Model* model) 
       : SimTK::Subsystem::Guts( "CMCActuatorSubsystem", "2.0"),
       _holdCoordinatesConstant(false),
       _holdTime(0.0),
       _qSet(NULL),
       _uSet(NULL),
       _model(model) {

       _qCorrections.setSize(_model->getNumCoordinates() );
       _uCorrections.setSize(_model->getNumSpeeds() );
       _qWork.setSize(_model->getNumCoordinates());
       _uWork.setSize(_model->getNumSpeeds());
   }
   CMCActuatorSubsystemRep* CMCActuatorSubsystemRep::cloneImpl() const { return new CMCActuatorSubsystemRep(*this); }

  CMCActuatorSubsystemRep::~CMCActuatorSubsystemRep() { }

  int CMCActuatorSubsystemRep::realizeSubsystemTopologyImpl(State& s) const {
     // Make sure the CMC Actuator subsystem has the same number of Z's as
     // the model as a whole so we don't miss any of them. This will include
     // some that aren't muscle states, but who cares?
     Vector z(_model->getMultibodySystem().getDefaultState().getNZ());
     s.allocateZ( getMySubsystemIndex(), z );
     return 0;
  }
  void CMCActuatorSubsystemRep::holdCoordinatesConstant( double t ) {
      _holdCoordinatesConstant = true;
      _holdTime = t;
  }
  void CMCActuatorSubsystemRep::releaseCoordinates() {
       _holdCoordinatesConstant = false;
  }
  Model*  CMCActuatorSubsystemRep::getModel() const {
       return( _model);
  }

  int CMCActuatorSubsystemRep::realizeSubsystemDynamicsImpl(const State& s) const  {

     Vector& q = _model->getMultibodySystem().getMatterSubsystem().updQ( const_cast<SimTK::State&>(_completeState) );
     Vector& u = _model->getMultibodySystem().getMatterSubsystem().updU( const_cast<SimTK::State&>(_completeState) );

     /* set generalized coordinates and speeds from spline sets */
    int i;
    int nq = _model->getNumCoordinates();
    int nu = _model->getNumSpeeds();
    double t;

    if(_holdCoordinatesConstant) {
         t = _holdTime;
    } else  {
         t = s.getTime();
    }

    _qSet->evaluate(_qWork,0,t);
    if(_uSet!=NULL) {
        _uSet->evaluate(_uWork,0,t);
    } else {
        _qSet->evaluate(_uWork,1,t);
    }

    for(i=0;i<nq;i++) q[i] = _qWork[i] + _qCorrections[i];
    for(i=0;i<nu;i++) u[i] = _uWork[i] + _uCorrections[i];


     /* copy  muscle states computed from the actuator system to the muscle states
        for the complete system  then compute forces*/
     const_cast<SimTK::State&>(_completeState).updZ() = s.getZ();
     const_cast<SimTK::State&>(_completeState).updTime() = t;

     _model->getMultibodySystem().realize(_completeState, SimTK::Stage::Acceleration);

     /* copy 1st derivatives of muscle states from complete system to actuator system */ 
     s.updZDot() = _completeState.getZDot();

/*
    cout << "_qWork=" << _qWork << endl;
    cout << "_uWork=" << _uWork << endl;
    cout << "q=" << q << endl;
    cout << "u=" << u << endl;
    cout << "actuatorStates=" << s.getZ() << endl;
    cout << " CMCrealize:Dynamics  time=" <<  s.getTime(); 
    cout << " Actuator dydt=" << _completeState.getZDot() << endl;
  
*/
     return 0;
  }      

   CMCActuatorSubsystem::CMCActuatorSubsystem(CMCActuatorSystem& system, Model* model) {
       adoptSubsystemGuts( rep = new CMCActuatorSubsystemRep(model) );
       system.adoptSubsystem( *this );
   }
   void CMCActuatorSubsystem::holdCoordinatesConstant( double t ) {
       rep->holdCoordinatesConstant(t);
   }
   void CMCActuatorSubsystem::releaseCoordinates() {
       rep->releaseCoordinates();
   }
