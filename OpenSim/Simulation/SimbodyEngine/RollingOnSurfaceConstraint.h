#ifndef __RollingOnSurfaceConstraint_h__
#define __RollingOnSurfaceConstraint_h__

// RollingOnSurfaceConstraint.h
// Author: Ajay Seth
/*
 * Copyright (c) 2009, Stanford University. All rights reserved. 
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


// INCLUDE
#include <string>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
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
class OSIMSIMULATION_API RollingOnSurfaceConstraint : public UnilateralConstraint  
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Specify the rolling body for this constraint. */
	PropertyStr _rollingBodyNameProp;
	std::string& _rollingBodyName;

	/** Specify the body containing the surface (plane) that the rolling body rolls on. */
	PropertyStr _surfaceBodyNameProp;
	std::string& _surfaceBodyName;

	/** Surface normal direction in the surface body */
	PropertyDblVec3 _surfaceNormalProp;
	SimTK::Vec3& _surfaceNormal;

	/** Surface height in the direction of the normal in the surface body */
	PropertyDbl _surfaceHeightProp;
	double& _surfaceHeight;

	/** Coulomb friction coefficient for rolling on the surface */
	PropertyDbl _coulombFrictionCoefficientProp;
	double& _coulombFrictionCoefficient;

	/** A guess at the area of contact approximated by a circle of radius: */
	PropertyDbl _surfaceContactRadiusProp;
	double& _surfaceContactRadius;

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
	RollingOnSurfaceConstraint(const RollingOnSurfaceConstraint &aConstraint);
	virtual ~RollingOnSurfaceConstraint();
	virtual Object* copy() const;
	RollingOnSurfaceConstraint& operator=(const RollingOnSurfaceConstraint &aConstraint);
	void copyData(const RollingOnSurfaceConstraint &aConstraint);
	virtual void setup(Model& aModel);

	/**
	 * Create the SimTK::Constraints: which implements this RollingOnSurfaceConstraint.
	 */
	void createSystem(SimTK::MultibodySystem& system) const;
	/**
	 * Populate the the SimTK::State: with defaults for the RollingOnSurfaceConstraint.
	 */
	void initState(SimTK::State& state) const;
	/**
	 * Given an existing SimTK::State set defaults for the RollingOnSurfaceConstraint.
	 */
	void setDefaultsFromState(const SimTK::State& state);

	//SET 
	void setRollingBodyByName(std::string aBodyName);
	void setSurfaceBodyByName(std::string aBodyName);
	void setContactPointOnSurfaceBody(const SimTK::State &s, SimTK::Vec3 point);

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

	virtual void calcConstraintForces(const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInParent, 
									  SimTK::Vector& mobilityForces);

private:

	void setNull();
	void setupProperties();

//=============================================================================
};	// END of class RollingOnSurfaceConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __RollingOnSurfaceConstraint_h__


