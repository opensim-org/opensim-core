#ifndef __SimbodyForceAccumulator_h__
#define __SimbodyForceAccumulator_h__
//-----------------------------------------------------------------------------
// File:     SimbodyAccumulatorForceSubsystem.h
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
 * A force subsystem for accumulating and applying OpenSim actuator
 * forces to an underlying Simbody multibody system.
 *
 * @authors Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMBODYENGINE_API SimbodyForceAccumulator :
	public SimTK::GeneralForceElements::UserForce 
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** Vector of spatial vectors containing the forces and torques applied to
	each of the bodies in the matter subsystem. */
	SimTK::Vector_<SimTK::SpatialVec> _bodyForces;

	/** Vector of mobility forces applied to each of the generalized
	coordinates in the matter subsystem. */
	SimTK::Vector _mobilityForces;


//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION AND DESTRUCTION
	explicit SimbodyForceAccumulator();
	SimTK::GeneralForceElements::UserForce* clone() const { return new SimbodyForceAccumulator(*this); }

	// RESET & RESIZE
	void resize(const SimTK::SimbodyMatterSubsystem& aMatter);
	void reset();

	// ACCUMULATE
	void accumulateStationForce(const SimTK::SimbodyMatterSubsystem& aMatter,
		SimTK::State &aState,SimTK::BodyId aBodyId,
		const SimTK::Vec3& aStation,const SimTK::Vec3& aForce);
	void accumulateBodyTorque(const SimTK::SimbodyMatterSubsystem& aMatter,
		SimTK::State &aState,SimTK::BodyId aBodyId,
		const SimTK::Vec3& aTorque);
	void accumulateGeneralizedForce(const SimTK::SimbodyMatterSubsystem& aMatter,
		SimTK::State& aState,SimTK::BodyId aBodyId,int aAxis,SimTK::Real& aForce);

   // CALC (Called by Simbody)
	void calc(const SimTK::MatterSubsystem& matter,const SimTK::State& state,
		SimTK::Vector_<SimTK::SpatialVec>& bodyForces,SimTK::Vector_<SimTK::Vec3>& particleForces,
		SimTK::Vector& mobilityForces,SimTK::Real& pe) const;
   
};

//-----------------------------------------------------------------------------
#endif // __SimbodyForceAccumulator_h__
//-----------------------------------------------------------------------------

} // end of namespace OpenSim
