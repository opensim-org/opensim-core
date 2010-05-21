#ifndef OPENSIM_FORCE_SUBSYSTEM_H_
#define OPENSIM_FORCE_SUBSYSTEM_H_
// OpenSimForceSubsystem.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "SimTKcommon/internal/SubsystemGuts.h"
#include "SimTKsimbody.h"

#ifdef WIN32
#pragma warning( disable : 4275 )
#endif
namespace OpenSim {

class Model;

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class OpenSimForceSubsystemRep : public SimTK::Subsystem::Guts {

   public:
   OpenSimForceSubsystemRep( Model* model);

   OpenSimForceSubsystemRep* cloneImpl() const { return new OpenSimForceSubsystemRep(*this); }
   ~OpenSimForceSubsystemRep() { }

  int realizeSubsystemTopologyImpl(SimTK::State& s) const;
  int realizeSubsystemAccelerationImpl(const SimTK::State& s) const;
  
  int getStep(const SimTK::State& s ) const;
  int& updStep(const SimTK::State& s ) const;

private:

  mutable SimTK::CacheEntryIndex _stepIndex;

  Model* _model;

  SimTK_DOWNCAST( OpenSimForceSubsystemRep, SimTK::Subsystem::Guts );
};
/// @endcond

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class OSIMSIMULATION_API OpenSimForceSubsystem : public SimTK::Subsystem {
    public:
    int getStep(const SimTK::State& s ) const { return rep->getStep(s); }
    int& updStep(const SimTK::State& s ) const  { return rep->updStep(s); }

    explicit OpenSimForceSubsystem( SimTK::MultibodySystem& sys, Model* model);

    OpenSimForceSubsystemRep *rep;
  
	SimTK_PIMPL_DOWNCAST(OpenSimForceSubsystem, SimTK::Subsystem);
};
/// @endcond

} // namespace OpenSim
#endif // FORCE_SUBSYSTEM_H_
