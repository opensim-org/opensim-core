//-----------------------------------------------------------------------------
// File:     SimbodyOpenSimUserForces.cpp
// Purpose:  Applying all the accumulated actuator and contact forces in OpenSim.
// Author:   Frank C. Anderson
//-----------------------------------------------------------------------------
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
	cout<<"SimbodyOpenSimUserForces.calc: forces coming in..."<<endl;
	cout<<_engine->getBodyForces()<<endl;
	cout<<_engine->getMobilityForces()<<endl;

	bodyForces += _engine->getBodyForces();
	mobilityForces += _engine->getMobilityForces();

	cout<<"SimbodyOpenSimUserForces.calc: forces going out..."<<endl;
	cout<<_engine->getBodyForces()<<endl;
	cout<<_engine->getMobilityForces()<<endl;
}