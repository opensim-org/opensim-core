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
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>
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
	virtual VisibleObject* getDisplayer() const;
	virtual void updateDisplayer(const SimTK::State& s);

protected:
    // Implement ModelComponent interface.
	void connectToModel(Model& aModel) OVERRIDE_11;
	void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;

private:
	void constructProperties();

//=============================================================================
};	// END of class Ligament
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_LIGAMENT_H_
