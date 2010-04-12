#ifndef __CustomActuator_h__
#define __CustomActuator_h__

// CustomActuator.h
// Author: Peter Eastman
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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

#include "Actuator.h"

namespace OpenSim {

class Body;
class Coordinate;
class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * This is an abstract subclass of Actuator which allows you to define new Actuator
 * types by overriding methods, rather than having to create a new SimTK::Force subclass.
 */
class OSIMSIMULATION_API CustomActuator : public Actuator
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** These temporarily store the vectors of forces while they are being computed. */
	friend class ForceAdapter;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	CustomActuator();
	CustomActuator(const CustomActuator &aActuator);
	OPENSIM_DECLARE_DERIVED(CustomActuator, Actuator);

protected:
	/**
	 * Subclasses must implement this method to compute the forces that should be applied to bodies
	 * and generalized speeds.
	 * This is invoked by ForceAdapter to perform the force computation.
	 */
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const = 0;

	/**
	 * Subclasses may optionally override this method to compute a contribution to the potential
	 * energy of the system.  The default implementation returns 0, which is appropriate for forces
	 * that do not contribute to potential energy.
	 */
	virtual double computePotentialEnergy(const SimTK::State& state) const;
	/**
	 * Apply a force to a particular body.  This method applies only a force, not a torque, which is
	 * equivalent to applying the force at the body's center of mass.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aBody    the body to apply the force to
	 * @param aForce   the force to apply, specified in the inertial frame
	 * @param bodyForces  the current set of system bodyForces this force is added to
	 */
	void applyForce(const SimTK::State &s, const OpenSim::Body &aBody, 
					const SimTK::Vec3& aForce, SimTK::Vector_<SimTK::SpatialVec> &bodyForces) const;
	/**
	 * Apply a force to a particular body.  Based on what point the force is applied at, this method
	 * automatically determines the torque produced by the force and applies that as well.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aBody    the body to apply the force to
	 * @param aPoint   the point at which to apply the force, specifieid in the inertial frame
	 * @param aForce   the force to apply, specified in the body's frame
	 * @param bodyForces  the current set of system bodyForces this force is added to 
	 */
	void applyForceToPoint(const SimTK::State &s, const OpenSim::Body &aBody, const SimTK::Vec3& aPoint, 
						   const SimTK::Vec3& aForce, SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const;
	/**
	 * Apply a torque to a particular body.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aBody    the body to apply the force to
	 * @param aTorque  the torque to apply, specified in the inertial frame
	 * @param bodyForces  the current set of system bodyForces this force is added to 
	 */
	void applyTorque(const SimTK::State &s, const OpenSim::Body &aBody, 
					 const SimTK::Vec3& aTorque, SimTK::Vector_<SimTK::SpatialVec> &bodyForces) const;
	/**
	 * Apply a generalized force.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aCoord  the generalized coordinate to apply the force to
	 * @param aForce  the force to apply
	 * @param generalizedForces  the current set of system generalizedForces this force is added to 
	 */
	void applyGeneralizedForce(const SimTK::State &s, const Coordinate &aCoord, 
							   double aForce, SimTK::Vector &generalizedForces) const;

	virtual	void createSystem(SimTK::MultibodySystem& system) const;
	
//=============================================================================
};	// END of class CustomActuator
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __CustomActuator_h__


