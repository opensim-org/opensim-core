#ifndef OPENSIM_LIGAMENT_H_
#define OPENSIM_LIGAMENT_H_

// Ligament.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS: Peter Loan
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
	virtual double computeMomentArm(SimTK::State& s, Coordinate& aCoord) const;
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
	void setup(Model& aModel) OVERRIDE_11;
	void createSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	void initState(SimTK::State& s) const OVERRIDE_11;

private:
	void constructProperties();

//=============================================================================
};	// END of class Ligament
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_LIGAMENT_H_
