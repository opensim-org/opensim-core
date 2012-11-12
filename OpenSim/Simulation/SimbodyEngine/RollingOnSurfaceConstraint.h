#ifndef __RollingOnSurfaceConstraint_h__
#define __RollingOnSurfaceConstraint_h__
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  RollingOnSurfaceConstraint.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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


// INCLUDE
#include <string>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyDblVec.h>
#include "UnilateralConstraint.h"
#include "Body.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a collection of rolling-without-slipping and 
 * non-penetration constraints on a surface.  
 * The underlying Constraints in Simbody are:
 *		PointInPlane to oppose penetration into the ground (unitlaterally)
 *		ConstantAngle about the normal to the enforce no spinning
 *		NoSlip1D along one axis of the plane
 *      NoSlip1D along the other axis
 *
 * mu = Coulomb Friction Coeefficient
 *
 * Each of these constraints have condition dependent on the reaction forces generate
 * collectively:
 *   PointInPlane normal force (Fn) must be positive (in the direction of the normal)
 *   ConstantAngle the reaction torque cannnot exceed contactRadius*mu*Fn
 *   Both NoSlip conditions are treated together, the magnitude the combined reaction 
 *   forces (in the plane) cannot exceed mu*Fn

 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API RollingOnSurfaceConstraint 
:   public UnilateralConstraint {
OpenSim_DECLARE_CONCRETE_OBJECT(RollingOnSurfaceConstraint, 
                                UnilateralConstraint);

//=============================================================================
// DATA
//=============================================================================
protected:

	OpenSim_DECLARE_PROPERTY(rolling_body, std::string, "Specify the rolling body for this constraint.");

	OpenSim_DECLARE_PROPERTY(surface_body, std::string, "Specify the body containing the surface (plane) that the rolling body rolls on.");

	OpenSim_DECLARE_PROPERTY(surface_normal, SimTK::Vec3, "Surface normal direction in the surface body.");

	OpenSim_DECLARE_PROPERTY(surface_height, double, "Surface height in the direction of the normal in the surface body.");

	OpenSim_DECLARE_PROPERTY(friction_coefficient, double, "Coulomb friction coefficient for rolling on the surface.");

	OpenSim_DECLARE_PROPERTY(contact_radius, double, "A guess at the area of contact approximated by a circle of radius.");

	/** First body is the rolling body. */
	Body *_rollingBody;

	/** Second body describes the surface body. */
	Body *_surfaceBody;

	/** Get the indices of underlying constraints to access from Simbody */
	std::vector<SimTK::ConstraintIndex> _indices;

	/**  This cache acts a temporary hold for the constraint conditions when time has not changed */
	std::vector<bool> _defaultUnilateralConditions;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	RollingOnSurfaceConstraint();
	virtual ~RollingOnSurfaceConstraint();

    // ModelComponent interface.
	void connectToModel(Model& aModel) OVERRIDE_11;

	/**
	 * Create the SimTK::Constraints: which implements this RollingOnSurfaceConstraint.
	 */
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	/**
	 * Populate the the SimTK::State: with defaults for the RollingOnSurfaceConstraint.
	 */
	void initStateFromProperties(SimTK::State& state) const OVERRIDE_11;
	/**
	 * Given an existing SimTK::State set defaults for the RollingOnSurfaceConstraint.
	 */
	void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;

	//SET 
	void setRollingBodyByName(std::string aBodyName);
	void setSurfaceBodyByName(std::string aBodyName);
	virtual void setContactPointForInducedAccelerations(const SimTK::State &s, SimTK::Vec3 point);

	// Methods that makes this a unilateral constraint
	virtual std::vector<bool> unilateralConditionsSatisfied(const SimTK::State& state);

	virtual bool isDisabled(const SimTK::State& state) const;

	// Unilateral conditions are automatically satisfied if constraint is not disabled
	virtual bool setDisabled(SimTK::State& state, bool isDisabled);

	// This method allows finer granularity over the subconstraints according to imposed behavior
	bool setDisabled(SimTK::State& state, bool isDisabled, std::vector<bool> shouldBeOn);

	// Set whether constraint is enabled or disabled but use cached values for unilateral conditions
	// instead of automatic reevaluation
	bool setDisabledWithCachedUnilateralConditions(bool isDisabled, SimTK::State& state) 
		{ return setDisabled(state, isDisabled, _defaultUnilateralConditions); };

	virtual void calcConstraintForces(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
									  SimTK::Vector& mobilityForces) const;

	/** 
	 * Methods to query a Constraint forces (defaults to the Lagrange mulipliers) applied
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	//virtual Array<std::string> getRecordLabels() const;
	/**
	 * Given SimTK::State object extract all the values necessary to report constraint forces (multipliers)
	 * Subclasses can override to report force, application location frame, etc. used in conjunction
	 * with getRecordLabels and should return same size Array
	 */
	//virtual Array<double> getRecordValues(const SimTK::State& state) const;

private:

	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class RollingOnSurfaceConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __RollingOnSurfaceConstraint_h__


