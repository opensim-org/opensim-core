#ifndef OPENSIM_LIGAMENT_H_
#define OPENSIM_LIGAMENT_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Ligament.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2013 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/ScaleSet.h>
#include "Model.h"
#include "Force.h"

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

class GeometryPath;

//=============================================================================
//=============================================================================
/**
 * A class implementing a ligament. The path of the ligament is
 * stored in a GeometryPath object.
 */
class OSIMSIMULATION_API Ligament : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(Ligament, Force);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/    OpenSim_DECLARE_UNNAMED_PROPERTY(GeometryPath, 
		"the set of points defining the path of the ligament");
    OpenSim_DECLARE_PROPERTY(resting_length, double,
		"resting length of the ligament");
	OpenSim_DECLARE_PROPERTY(pcsa_force, double,
		"force magnitude that scales the force-length curve");
	OpenSim_DECLARE_PROPERTY(force_length_curve, Function,
		"Function representing the force-length behavior of the ligament");
    /**@}**/


//==============================================================================
// PUBLIC METHODS
//==============================================================================
	Ligament();

    // Uses default (compiler-generated) destructor, copy constructor, and copy
    // assignment operator.

	//--------------------------------------------------------------------------
	// GET
	//--------------------------------------------------------------------------
	// Properties
	const GeometryPath& getGeometryPath() const 
    {   return get_GeometryPath(); }
	GeometryPath& updGeometryPath() 
    {   return upd_GeometryPath(); }
	virtual bool hasGeometryPath() const { return true;};
	virtual double getLength(const SimTK::State& s) const;
	virtual double getRestingLength() const 
    {   return get_resting_length(); }
	virtual bool setRestingLength(double aRestingLength);
	virtual double getMaxIsometricForce() const 
    {   return get_pcsa_force(); }
	virtual bool setMaxIsometricForce(double aMaxIsometricForce);
	virtual const Function& getForceLengthCurve() const 
    {   return get_force_length_curve(); }
	virtual bool setForceLengthCurve(const Function& aForceLengthCurve);

	// computed variables
	const double& getTension(const SimTK::State& s) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const;
	virtual void computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

	//--------------------------------------------------------------------------
	// SCALE
	//--------------------------------------------------------------------------
	virtual void preScale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void scale(const SimTK::State& s, const ScaleSet& aScaleSet);
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);


	//--------------------------------------------------------------------------
	// Display
	//--------------------------------------------------------------------------
	virtual const VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s) const;

protected:
    /** Override this method if you would like to calculate a color for use
    when the %Ligament's path is displayed in the visualizer. You do not have 
    to invoke the base class ("Super") method, just replace it completely. This
    method will be invoked during realizeDynamics() so the supplied \a state has 
    already been realized through Stage::Velocity and you can access time, 
    position, and velocity dependent quantities. You must \e not attempt to 
    realize the passed-in \a state any further since we are already in the 
    middle of realizing here. Return SimTK::Vec3(SimTK::NaN) if you want to 
    leave the color unchanged (that's what the base class implementation does).

    @param[in] state    
        A SimTK::State already realized through Stage::Velocity. Do not 
        attempt to realize it any further.
    @returns 
        The desired color for the path as an RGB vector with each
        component ranging from 0 to 1, or NaN to indicate that the color
        should not be changed. **/
    virtual SimTK::Vec3 computePathColor(const SimTK::State& state) const;

    // Implement ModelComponent interface.
    /** Extension of parent class method; derived classes may extend further. **/
	void connectToModel(Model& aModel) OVERRIDE_11;
    /** Extension of parent class method; derived classes may extend further. **/
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    /** Extension of parent class method; derived classes may extend further. **/
    void realizeDynamics(const SimTK::State& state) const OVERRIDE_11;

	//Force reporting
	/** 
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	OpenSim::Array<std::string> getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName());
		return labels;
	}
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
		OpenSim::Array<double> values(1);
		values.append(getTension(state));
		return values;
	}

private:
	void constructProperties();

//=============================================================================
};	// END of class Ligament
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_LIGAMENT_H_
