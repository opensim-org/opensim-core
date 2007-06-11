#ifndef __SimbodyOpenSimUserForces_h__
#define __SimbodyOpenSimUserForces_h__
//-----------------------------------------------------------------------------
// File:     SimbodyOpenSimUserForces.h
// Parent:   GeneralForceElements
// Purpose:  Accumulates and applies all the actuator and contact forces in OpenSim.
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

// INCLUDES
#include <iostream>
#include <string>
#include "osimSimbodyEngineDLL.h"
#include <SimTKsimbody.h>

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
class OSIMSIMBODYENGINE_API SimbodyOpenSimUserForces :
	public SimTK::GeneralForceElements::UserForce 
{
//=============================================================================
// DATA
//=============================================================================
private:
	OpenSim::SimbodyEngine *_engine;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION AND DESTRUCTION
	explicit SimbodyOpenSimUserForces();
	SimTK::GeneralForceElements::UserForce* clone() const { return new SimbodyOpenSimUserForces(*this); }

   // CALC (Called by Simbody)
	void calc(const SimTK::MatterSubsystem& matter,const SimTK::State& state,
		SimTK::Vector_<SimTK::SpatialVec>& bodyForces,SimTK::Vector_<SimTK::Vec3>& particleForces,
		SimTK::Vector& mobilityForces,SimTK::Real& pe) const;   
};

//-----------------------------------------------------------------------------
#endif // __SimbodyOpenSimUserForces_h__
//-----------------------------------------------------------------------------

} // end of namespace OpenSim
