#ifndef __PrescribedForce_h__
#define __PrescribedForce_h__
// Force.h
// Author: Peter Eastman
/*
 * Copyright (c)  2008, Stanford University. All rights reserved. 
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
#include "OpenSim/Common/Function.h"
#include "OpenSim/Common/NaturalCubicSpline.h"
#include "CustomForce.h"
#include "OpenSim/Common/FunctionSet.h"

namespace OpenSim {

class Model;
class FunctionSet;
class Storage;
/**
 * This applies a force and/or torque to a body which is fully specified as a function of time.  It
 * is defined by three sets of functions, all of which are optional:
 *
 * - Three functions that specify a force to apply as a function of time.  If these functions are not
 * provided, no force is applied.
 *
 * - Three functions that specify the point at which the force should be applied.  The resulting torque
 * is automatically calculated and applied.  If these functions are not provided, the force is applied
 * at the body's center of mass, and therefore produces no torque.
 *
 * - Three functions that specify a torque to apply.  This is in addition to any torque resulting from
 * the force.  If these functions are not provided, no additional torque is applied.
 *
 * @author Peter Eastman, Matt DeMers
 */
class OSIMSIMULATION_API PrescribedForce : public CustomForce
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** The body that the Prescribed force is applied to. */
	PropertyStr _bodyNameProp;
	std::string& _bodyName;

	PropertyBool _forceIsGlobalProp;
	bool &_forceIsGlobal;

	PropertyBool _pointIsGlobalProp;
	bool &_pointIsGlobal;

	/* Functions representing force, applicationPoint, and torque in x,y,z directions */
	PropertyObj _forceFunctionSetProp;
	FunctionSet &_forceFunctionSet;

	PropertyObj _pointFunctionSetProp;
	FunctionSet &_pointFunctionSet;

	PropertyObj _torqueFunctionSetProp;
	FunctionSet &_torqueFunctionSet;

private:
	OpenSim::Body *_body;
	Function* _forceX;
	Function* _forceY;
	Function* _forceZ;
	Function* _pointX;
	Function* _pointY;
	Function* _pointZ;
	Function* _torqueX;
	Function* _torqueY;
	Function* _torqueZ;
	Array<NaturalCubicSpline> splines;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	/**
	 * Construct a PrescribedForce.  By default, the force, torque, and point functions are all unspecified,
	 * meaning that it applies no force or torque.  To specify them, call setForceFunctions(), setTorqueFunctions(),
	 * and setPointFunctions().
	 *
	 * @param body     the body to apply the force to
	 */
	PrescribedForce(OpenSim::Body* body=0);
	PrescribedForce(const PrescribedForce& force);
	PrescribedForce(DOMElement* aNode);
	~PrescribedForce();
	virtual Object* copy() const;

	virtual void setup(Model& model);
	virtual void setupFromXML();

	void setBodyName(const std::string& aBodyName) { _bodyName = aBodyName; };
	const std::string& getBodyName() const { return (_bodyName); }

	/**
	 * Set the functions which specify the force to apply.  By default the force is specified in inertial coordinates.
	 * This can be changed by calling setForceIsInGlobalFrame().
	 *
	 * All of the Function objects should have been allocated on the heap with the "new" operator.
	 * This object takes over ownership of them, and will delete them when it is deleted itself.
	 *
	 * @param forceX   a function of time which calculates the X component of the force to apply
	 * @param forceY   a function of time which calculates the Y component of the force to apply
	 * @param forceZ   a function of time which calculates the Z component of the force to apply
	 */
	void setForceFunctions(Function* forceX, Function* forceY, Function* forceZ);
	const FunctionSet& getForceFunctions() const { return _forceFunctionSet; };
	FunctionSet& updForceFunctions() { return _forceFunctionSet; };
	void getForceFunctionNames(OpenSim::Array<std::string>& aFunctionNames) {
			_forceFunctionSet.getNames(aFunctionNames);
	};
	void setForceFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
		const OpenSim::Storage& kineticsStore);
	void clearForceFunctions() { _forceFunctionSet.setSize(0); };
	/**
	 * Set the functions which specify the point at which to apply the force.  By default the point is specified in
	 * the body's local coordinates.  This can be changed by calling setPointIsInGlobalFrame().
	 *
	 * All of the Function objects should have been allocated on the heap with the "new" operator.
	 * This object takes over ownership of them, and will delete them when it is deleted itself.
	 *
	 * @param pointX   a function of time which calculates the X coordinate of the point at which to apply the force
	 * @param pointY   a function of time which calculates the Y coordinate of the point at which to apply the force
	 * @param pointZ   a function of time which calculates the Z coordinate of the point at which to apply the force
	 */
	void setPointFunctions(Function* pointX, Function* pointY, Function* pointZ);
	const FunctionSet& getPointFunctions() const { return _pointFunctionSet; };
	FunctionSet& updPointFunctions() { return _pointFunctionSet; };
	void getPointFunctionNames(OpenSim::Array<std::string>& aFunctionNames){
			_pointFunctionSet.getNames(aFunctionNames);
	}
	void setPointFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
		const OpenSim::Storage& kineticsStore) ;
	void clearPointFunctions() { _pointFunctionSet.setSize(0); };
	/**
	 * Set the functions which specify the torque to apply.  By default the torque is specified in inertial coordinates.
	 * This can be changed by calling setForceIsInGlobalFrame().
	 *
	 * All of the Function objects should have been allocated on the heap with the "new" operator.
	 * This object takes over ownership of them, and will delete them when it is deleted itself.
	 *
	 * @param torqueX   a function of time which calculates the X component of the torque to apply
	 * @param torqueY   a function of time which calculates the Y component of the torque to apply
	 * @param torqueZ   a function of time which calculates the Z component of the torque to apply
	 */
	void setTorqueFunctions(Function* torqueX, Function* torqueY, Function* torqueZ);
	const FunctionSet& getTorqueFunctions() const { return _torqueFunctionSet; };
	FunctionSet& updTorqueFunctions() { return _torqueFunctionSet; };
	void getTorqueFunctionNames(OpenSim::Array<std::string>& aFunctionNames){
		_torqueFunctionSet.getNames(aFunctionNames);
	}
	void setTorqueFunctionNames(const OpenSim::Array<std::string>& aFunctionNames, 
		const OpenSim::Storage& kineticsStore);
	void clearTorqueFunctions() { _torqueFunctionSet.setSize(0); };
	/**
	 * Setup the functions from a specific type of storage file.  
	 *
	 * @param anExternalLoadsFileName	the name of a .mot or .sto file.  The file must contain 10 data
	 * columns labeled time, torqueX, torqueY, torqueZ, forceX, forceY, forceZ, pointX, pointY, pointZ
	 */
	void setFunctionsFromFile(std::string anExternalLoadsFileName);
	/**
	 * Get whether the force and torque are specified in inertial coordinates or in the body's local coordinates.
	 */
	bool getForceIsInGlobalFrame() const;
	/**
	 * Set whether the force and torque are specified in inertial coordinates or in the body's local coordinates.
	 */
	void setForceIsInGlobalFrame(bool isGlobal);
	/**
	 * Get whether the point is specified in inertial coordinates or in the body's local coordinates.
	 */
	bool getPointIsInGlobalFrame() const;
	/**
	 * Set whether the point is specified in inertial coordinates or in the body's local coordinates.
	 */
	void setPointIsInGlobalFrame(bool isGlobal);

	/** Get the body that the prescribed force is acting upon */
	const OpenSim::Body& getBody() const {return *_body; }

	/**
	 * Conevenince methods to access prescribed force functions
	 */
	SimTK::Vec3 getForceAtTime(double aTime) const;
	SimTK::Vec3 getPointAtTime(double aTime) const;
	SimTK::Vec3 getTorqueAtTime(double aTime) const;

	/**
	 * Methods used for reporting
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const;
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const;

#ifndef SWIG
	PrescribedForce& operator=(const PrescribedForce &aJoint);
#endif
	OPENSIM_DECLARE_DERIVED(PrescribedForce, Force);

protected:
	/**
	 * Compute the force.
	 */
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const;

private:
	void setNull();
	void setupProperties();
	void copyData(const PrescribedForce& orig);
//=============================================================================
};	// END of class PrescribedForce
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PrescribedForce_h__
