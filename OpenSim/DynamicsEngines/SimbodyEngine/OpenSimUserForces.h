#ifndef __OpenSimUserForces_h__
#define __OpenSimUserForces_h__
//-----------------------------------------------------------------------------
// File:     OpenSimUserForces.h
// Purpose:  Applying all the accumulated actuator and contact forces in OpenSim.
// Author:   Frank C. Anderson
//-----------------------------------------------------------------------------
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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

// INCLUDES
#include <iostream>
#include <string>
#include "osimSimbodyEngineDLL.h"
#include <SimTKsimbody.h>
#include "SimbodyEngine.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for applying forces accumulated in OpenSim to an underlying
 * Simbody multibody system.  The accumulated forces are obtained
 * from the SimbodyEngine class.
 *
 * @authors Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API OpenSimUserForces :
	public SimTK::Force::Custom::Implementation
{
//=============================================================================
// DATA
//=============================================================================
private:
	SimbodyEngine *_engine;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION AND DESTRUCTION
	OpenSimUserForces(SimbodyEngine *aEngine);

   // CALC FORCES (Called by Simbody)
	void calcForce(const SimTK::State& state,
		SimTK::Vector_<SimTK::SpatialVec>& bodyForces,SimTK::Vector_<SimTK::Vec3>& particleForces,
		SimTK::Vector& mobilityForces) const;   

   // CALC POTENTIAL ENERGY (Called by Simbody)
	SimTK::Real calcPotentialEnergy(const SimTK::State& state) const;   

};

//-----------------------------------------------------------------------------
#endif // __OpenSimUserForces_h__
//-----------------------------------------------------------------------------

} // end of namespace OpenSim
