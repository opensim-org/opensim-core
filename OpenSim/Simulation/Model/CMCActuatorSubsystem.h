

#ifndef CMC_ACTUATOR_SUBSYSTEM_H_
#define CMC_ACTUATOR_SUBSYSTEM_H_
// ActuatorSubsystem.h
// //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// /*
// * Copyright (c)  2008, Stanford University. All rights reserved. 
// * Use of the OpenSim software in source form is permitted provided that the following
// * conditions are met:
// *   1. The software is used only for non-commercial research and education. It may not
// *     be used in relation to any commercial activity.
// *   2. The software is not distributed or redistributed.  Software distribution is allowed 
// *     only through https://simtk.org/home/opensim.
// *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
// *      presentations, or documents describing work in which OpenSim or derivatives are used.
// *   4. Credits to developers may not be removed from executables
// *     created from modifications of the source.
// *   5. Modifications of source code must retain the above copyright notice, this list of
// *     conditions and the following disclaimer. 
// * 
// *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
// *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
// *  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
// *  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// *  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// *  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
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


