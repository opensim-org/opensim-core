#ifndef OPENSIM_PRESCRIBED_FORCE_H_
#define OPENSIM_PRESCRIBED_FORCE_H_
// Force.h
// Author: Peter Eastman
/*
 * Copyright (c)  2008-12, Stanford University. All rights reserved. 
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
#include "OpenSim/Simulation/osimSimulationDLL.h"
#include "OpenSim/Common/PropertyStr.h"
#include "OpenSim/Common/PropertyBool.h"
#include "OpenSim/Common/PropertyObj.h"
#include "OpenSim/Common/Function.h"
#include "OpenSim/Common/NaturalCubicSpline.h"
#include "OpenSim/Common/FunctionSet.h"
#include "Force.h"

namespace OpenSim {

class Model;
class FunctionSet;
class Storage;

/** This applies to a body a force and/or torque that is fully specified as a 
function of time. It is defined by three sets of functions, all of which are 
optional:

  - Three functions that specify the (x,y,z) components of a force vector
    to apply (at a given point) as a function of time. If these functions are 
    not provided, no force is applied.

  - Three functions that specify the (x,y,z) components of a point location at 
    which the force should be applied. If these functions are not provided, the 
    force is applied at the body's origin (not necessarily the body's center 
    of mass).

  - Three functions that specify the (x,y,z) components of a pure torque 
    vector to apply. This is in addition to any torque resulting from the 
    applied force. If these functions are not provided, no additional torque 
    is applied.

@author Peter Eastman, Matt DeMers
**/
class OSIMSIMULATION_API PrescribedForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(PrescribedForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations **/
    /**@{**/
    /** "body" property is a string containing the name of the body to which
    the force will be applied. **/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(body, std::string,
		"Name of the body the force is applied to.");
    /** "pointIsGlobal" property is a flag indicating whether the point
    calculated by the Functions in pointFunctions are returned in the global
    frame rather than in the body frame which is the default. **/
    OpenSim_DECLARE_PROPERTY(pointIsGlobal, bool,
		"Flag indicating whether the point (specified in pointFunctions) "
        "is in global frame.");
    /** "forceIsGlobal" property is a flag indicating whether the force and
    torque returned by the Functions in forceFunctions and torqueFunctions,
    resp., are returned in the global frame (the default). Otherwise they
    are returned in the body frame. **/
    OpenSim_DECLARE_PROPERTY(forceIsGlobal, bool,
		"Flag indicating whether the quantities (specified in "
        "force/torqueFunctions) is in global frame.");
    /** These are three functions providing the x,y,z measure numbers of the
    force vector being applied to the body. The coordinate frame in which
    this vector is interpreted depends on the "forceIsGlobal" property. **/
    // Would have been better for this to be a list property. 
    OpenSim_DECLARE_PROPERTY(forceFunctions, FunctionSet,
		"Three functions describing the force to be applied.");
    /** These are three functions providing the x,y,z measure numbers of the
    point at which the force should be applied. The coordinate frame in which
    this position vector is interpreted depends on the "pointIsGlobal" 
    property. **/
    // Would have been better for this to be a list property. 
    OpenSim_DECLARE_PROPERTY(pointFunctions, FunctionSet,
		"Three functions describing the location at which the force "
        "is applied.");
    /** These are three functions providing the x,y,z measure numbers of the
    torque vector being applied to the body. The coordinate frame in which
    this vector is interpreted depends on the "torqueIsGlobal" property. **/
    // Would have been better for this to be a list property. 
    OpenSim_DECLARE_PROPERTY(torqueFunctions, FunctionSet,
		"Three functions describing the torque the PrescribedForce applies.");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
	/**
	 * Construct a PrescribedForce. By default, the force, torque, and point 
     * functions are all unspecified, meaning that it applies no force or 
     * torque.  To specify them, call setForceFunctions(), setTorqueFunctions(),
	 * and setPointFunctions().
	 *
	 * @param body     the body to apply the force to
	 */
	explicit PrescribedForce(OpenSim::Body* body=0);
    /** Construct from an XML element. **/
	explicit PrescribedForce(SimTK::Xml::Element& aNode);

    // default destructor, copy constructor, copy assignment

	/** Copy in properties from XML. **/
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	void setBodyName(const std::string& aBodyName) { setProperty_body(aBodyName); }
	const std::string& getBodyName() const { return getProperty_body(); }

	/**
	 * Set the functions which specify the force to apply.  By default the 
     * force is specified in inertial coordinates.
	 * This can be changed by calling setForceIsInGlobalFrame().
	 *
	 * All of the Function objects should have been allocated on the heap with 
     * the "new" operator. This object takes over ownership of them, and will 
     * delete them when it is deleted itself.
	 *
	 * @param forceX   a function of time which calculates the X component of 
     *                      the force to apply
	 * @param forceY   a function of time which calculates the Y component of 
     *                      the force to apply
	 * @param forceZ   a function of time which calculates the Z component of 
     *                      the force to apply
	 */
	void setForceFunctions(Function* forceX, Function* forceY, Function* forceZ);
	const FunctionSet& getForceFunctions() const { return getProperty_forceFunctions(); }
	FunctionSet& updForceFunctions() { return updProperty_forceFunctions(); }
	void getForceFunctionNames(OpenSim::Array<std::string>& aFunctionNames) {
			getForceFunctions().getNames(aFunctionNames);
	}
	void setForceFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
		const OpenSim::Storage& kineticsStore);
	void clearForceFunctions() { updForceFunctions().setSize(0); }
	/**
	 * Set the functions which specify the point at which to apply the force.  
     * By default the point is specified in the body's local coordinates.  
     * This can be changed by calling setPointIsInGlobalFrame().
	 *
	 * All of the Function objects should have been allocated on the heap with 
     * the "new" operator. This object takes over ownership of them, and will 
     * delete them when it is deleted itself.
	 *
	 * @param pointX   a function of time which calculates the X coordinate of 
     *                      the point at which to apply the force
	 * @param pointY   a function of time which calculates the Y coordinate of 
     *                      the point at which to apply the force
	 * @param pointZ   a function of time which calculates the Z coordinate of 
     *                      the point at which to apply the force
	 */
	void setPointFunctions(Function* pointX, Function* pointY, Function* pointZ);
	const FunctionSet& getPointFunctions() const { return getProperty_pointFunctions(); }
	FunctionSet& updPointFunctions() { return updProperty_pointFunctions(); }
	void getPointFunctionNames(OpenSim::Array<std::string>& aFunctionNames){
			getPointFunctions().getNames(aFunctionNames);
	}
	void setPointFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
		const OpenSim::Storage& kineticsStore) ;
	void clearPointFunctions() { updPointFunctions().setSize(0); }
	/**
	 * Set the functions which specify the torque to apply. By default the 
     * torque is specified in inertial coordinates.
	 * This can be changed by calling setForceIsInGlobalFrame().
	 *
	 * All of the Function objects should have been allocated on the heap with 
     * the "new" operator. This object takes over ownership of them, and will 
     * delete them when it is deleted itself.
	 *
	 * @param torqueX   a function of time which calculates the X component of 
     *                      the torque to apply
	 * @param torqueY   a function of time which calculates the Y component of 
     *                      the torque to apply
	 * @param torqueZ   a function of time which calculates the Z component of 
     *                      the torque to apply
	 */
	void setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ);
	const FunctionSet& getTorqueFunctions() const { return getProperty_torqueFunctions(); }
	FunctionSet& updTorqueFunctions() { return updProperty_torqueFunctions(); }
	void getTorqueFunctionNames(OpenSim::Array<std::string>& aFunctionNames){
		getTorqueFunctions().getNames(aFunctionNames);
	}
	void setTorqueFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
		const OpenSim::Storage& kineticsStore);
	void clearTorqueFunctions() { updTorqueFunctions().setSize(0); }

	/** Get whether the force and torque are specified in inertial coordinates 
    or in the body's local coordinates. **/
    bool getForceIsInGlobalFrame() const {return getProperty_forceIsGlobal();}
	/** Set whether the force and torque are specified in inertial coordinates 
    or in the body's local coordinates. **/
	void setForceIsInGlobalFrame(bool isGlobal) 
    {   setProperty_forceIsGlobal(isGlobal); }
	/** Get whether the point is specified in inertial coordinates or in the 
    body's local coordinates. **/
    bool getPointIsInGlobalFrame() const {return getProperty_pointIsGlobal();}
	/** Set whether the point is specified in inertial coordinates or in the 
    body's local coordinates. **/
	void setPointIsInGlobalFrame(bool isGlobal)
    {   setProperty_pointIsGlobal(isGlobal); }

	/** Get the body that the prescribed force is acting upon. **/
	const OpenSim::Body& getBody() const {assert(_body); return *_body; }

	/** Convenience method to evaluate the prescribed force functions at
    an arbitrary time. Returns zero if there aren't three functions defined. **/
	SimTK::Vec3 getForceAtTime(double aTime) const;
	/** Convenience method to evaluate the prescribed force application point
    functions at an arbitrary time. Returns zero if there aren't three 
    functions defined. **/
	SimTK::Vec3 getPointAtTime(double aTime) const;
	/** Convenience method to evaluate the prescribed torque functions at
    an arbitrary time. Returns zero if there aren't three functions defined. **/
	SimTK::Vec3 getTorqueAtTime(double aTime) const;

	/**
	 * Methods used for reporting
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const;
	/**
	 * Given SimTK::State object extract all the values necessary to report 
     * forces, application location frame, etc. used in conjunction with 
     * getRecordLabels() and should return same size Array.
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const;


protected:
	/** ModelComponent interface. **/ 
	void setup(Model& model) OVERRIDE_11;
	/** Force interface. **/
	virtual void computeForce
       (const SimTK::State&                state, 
	    SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
        SimTK::Vector&                     generalizedForces) const OVERRIDE_11;

//==============================================================================
// DATA
//==============================================================================
private:
	OpenSim::Body *_body; // a shallow reference; don't delete

private:
	void setNull();
	void constructProperties();

//=============================================================================
};	// END of class PrescribedForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PRESCRIBED_FORCE_H_
