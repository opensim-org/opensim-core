//-----------------------------------------------------------------------------
// File:     SimbodyOpenSimUserForces.cpp
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <iostream>
#include <string>
#include <math.h>
#include "SimbodyOpenSimUserForces.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Construct a force subsystem for accumulating and applying OpenSim actuator
 * forces to an underlying Simbody multibody system.
 *
 * @param aMatterSubsystem Matter subsystem for which and to which actuator
 * forces are to be applied.
 */
SimbodyOpenSimUserForces::SimbodyOpenSimUserForces(SimbodyEngine *aEngine)
{
	_engine = aEngine;
}


//=============================================================================
// CALC
//=============================================================================
//_____________________________________________________________________________
/**
 * Callback method called by Simbody when it requests the applied forces.
 * This method is called after the dynamics stage is realized.
 *
 * @param matter Matter subsystem. Should match the matter member variable.
 * @param state Current state of the Simbody multibody system.
 * @param bodyForces Vector of forces and torques applied to the bodies.
 * @param particleForces Vector of forces applied to particles.
 * @param mobilityForces Array of generalized forces.
 * @param pe For forces with a potential energy?
 */
void SimbodyOpenSimUserForces::
calc(const SimTK::MatterSubsystem& matter,const SimTK::State& state,
	SimTK::Vector_<SimTK::SpatialVec>& bodyForces,SimTK::Vector_<SimTK::Vec3>& particleForces,
	SimTK::Vector& mobilityForces,SimTK::Real& pe) const
{
	//cout<<"SimbodyOpenSimUserForces.calc: forces coming in..."<<endl;
	//cout<<_engine->getBodyForces()<<endl;
	//cout<<_engine->getMobilityForces()<<endl;

	bodyForces += _engine->getBodyForces();
	mobilityForces += _engine->getMobilityForces();

	//cout<<"SimbodyOpenSimUserForces.calc: forces going out..."<<endl;
	//cout<<_engine->getBodyForces()<<endl;
	//cout<<_engine->getMobilityForces()<<endl;
}