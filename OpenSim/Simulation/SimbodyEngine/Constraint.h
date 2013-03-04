#ifndef OPENSIM_CONSTRAINT_H_
#define OPENSIM_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Constraint.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A parent class for implementing a Simbody Constraint.
 * Specific constraints should be derived from this class. 
 *
 * @author Frank C. Anderson
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Constraint : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Constraint, ModelComponent);

//=============================================================================
// PROPERTY
//=============================================================================
public:

	OpenSim_DECLARE_PROPERTY(isDisabled, bool, "Flag indicating whether the constraint is disabled or not. Disabled means that the constraint is not active in subsequent dynamics realization");

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//-------------------------------------------------------------------------
	Constraint();
	virtual ~Constraint();

	virtual void updateFromConstraint(SimTK::State& s, const Constraint &aConstraint);
	virtual bool isDisabled(const SimTK::State& s) const;
	virtual bool setDisabled(SimTK::State& s, bool isDisabled);

	virtual void calcConstraintForces(const SimTK::State& s, SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
									  SimTK::Vector& mobilityForces) const;

	/** 
	 * Methods to query a Constraint forces (defaults to the Lagrange mulipliers) applied
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	virtual Array<std::string> getRecordLabels() const;
	/**
	 * Given SimTK::State object extract all the values necessary to report constraint forces (multipliers)
	 * Subclasses can override to report force, application location frame, etc. used in conjunction
	 * with getRecordLabels and should return same size Array
	 */
	virtual Array<double> getRecordValues(const SimTK::State& state) const;

	virtual void scale(const ScaleSet& aScaleSet) {};

	/**
	* This method specifies the interface that a constraint must implement
	* in order to be used by the Induced Accelerations Analysis
	*/
	virtual void setContactPointForInducedAccelerations(const SimTK::State &s, SimTK::Vec3 point){
		throw Exception("This constraint does not implement setContactPointForInducedAccelerations");
	}

protected:
    // ModelComponent interface.
	void connectToModel(Model& aModel) OVERRIDE_11;
	void initStateFromProperties(SimTK::State& state) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;
	int getNumStateVariables() const  OVERRIDE_11 { return 0; };

	/** ID for the constraint in Simbody. */
	SimTK::ConstraintIndex _index;

private:
	void setNull();
	void constructProperties();

	friend class SimbodyEngine;

//=============================================================================
};	// END of class Constraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONSTRAINT_H_


