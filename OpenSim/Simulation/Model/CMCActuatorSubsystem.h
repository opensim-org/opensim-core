#ifndef CMC_ACTUATOR_SUBSYSTEM_H_
#define CMC_ACTUATOR_SUBSYSTEM_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CMCActuatorSubsystem.h                      *
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
//
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "SimTKsimbody.h"
#include "SimTKcommon/internal/SubsystemGuts.h"
#include "SimTKcommon/internal/SystemGuts.h"

using namespace SimTK;

namespace OpenSim {

class Model;

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

// Excluding this from Doxygen. -Sam Hamner
    /// @cond
class CMCActuatorSystemRep : public System::Guts {
    public:
    CMCActuatorSystemRep() : SimTK::System::Guts( "CMCActuatorSystem", "2.0") {}
    
    // Required by the System::Guts interface.
    /*virtual*/ CMCActuatorSystemRep* cloneImpl() const 
    {   return new CMCActuatorSystemRep(*this); }

    // This system doesn't have constraints, prescribed motion, or events so
    // we don't need to implement any of the related System::Guts methods.

    SimTK_DOWNCAST( CMCActuatorSystemRep, System::Guts );
};

class CMCActuatorSystem : public System {
   public:
   explicit CMCActuatorSystem() {
       adoptSystemGuts( new CMCActuatorSystemRep() );
       DefaultSystemSubsystem defsub(*this);
   }

   SimTK_PIMPL_DOWNCAST( CMCActuatorSystem, System );
};

class CMCActuatorSubsystemRep : public Subsystem::Guts {

  public:
  CMCActuatorSubsystemRep( Model* model);

  CMCActuatorSubsystemRep* cloneImpl() const;
  ~CMCActuatorSubsystemRep();

  void setCompleteState(const SimTK::State& s);
  const SimTK::State& getCompleteState() const;

  int realizeSubsystemTopologyImpl(State& s) const;
  int realizeSubsystemDynamicsImpl(const State& s) const;
  void setSpeedCorrections(const double corrections[] );
  void setCoordinateCorrections(const double corrections[] );
  void setSpeedTrajectories(FunctionSet *aSet);
  void setCoordinateTrajectories(FunctionSet *aSet);
  FunctionSet* getCoordinateTrajectories() const ;
  FunctionSet* getSpeedTrajectories() const ;
  Model* getModel() const ;
  
  void holdCoordinatesConstant( double t );
  void releaseCoordinates();

  SimTK::State  _completeState;
  Model*        _model;
  bool          _holdCoordinatesConstant;
  double        _holdTime;
  Array<double> _qCorrections;
  Array<double> _uCorrections;

  mutable Array<double> _qWork;
  mutable Array<double> _uWork;

  /** Prescribed trajectories of the generalized coordinates. */
  FunctionSet *_qSet;
  /** Prescribed trajectories of the generalized speeds. */
  FunctionSet *_uSet;

  SimTK_DOWNCAST( CMCActuatorSubsystemRep, Subsystem::Guts );
};

class OSIMSIMULATION_API CMCActuatorSubsystem : public Subsystem {
    public:
    explicit CMCActuatorSubsystem( CMCActuatorSystem& sys, Model* model);

    CMCActuatorSubsystemRep *rep;

    void holdCoordinatesConstant( double t );
    void releaseCoordinates();
	void setCompleteState(const SimTK::State& s)  {
        rep->setCompleteState( s );
    }
    const SimTK::State& getCompleteState() const {
        return( rep->getCompleteState());
    }

    void  setSpeedTrajectories(FunctionSet *aSet) {
        rep->setSpeedTrajectories( aSet );
    }
     void setCoordinateTrajectories(FunctionSet *aSet)  {
        rep->setCoordinateTrajectories( aSet );
    }
    FunctionSet* getSpeedTrajectories() const {
        return( rep->getSpeedTrajectories());
    }
     FunctionSet* getCoordinateTrajectories() const {
        return( rep->getCoordinateTrajectories() );
    }
    void setSpeedCorrections(const double *corrections ) {
            rep->setSpeedCorrections( corrections );
    }
    void setCoordinateCorrections(const double *corrections ) {
            rep->setCoordinateCorrections( corrections );
    }
    Model* getModel() const {
        return( rep->getModel() );
    }

    SimTK_PIMPL_DOWNCAST(CMCActuatorSubsystem, Subsystem);
};

/// @endcond

} // namespace OpenSim
#endif // CMC_ACTUATOR_SUBSYSTEM_H_


