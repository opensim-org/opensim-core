#ifndef OPENSIM_FREE_JOINT_H_
#define OPENSIM_FREE_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  FreeJoint.h                            *
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
#include "Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a Free joint.  The underlying implementation 
 * in Simbody is a MobilizedBody::Free.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API FreeJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(FreeJoint, Joint);

private:
	static const int _numMobilities = 6;
//=============================================================================
// DATA
//=============================================================================
protected:

	/** Flag to use Euler angles to parameterize rotation of the body  */
	//PropertyBool _useEulerAnglesProp;
	//bool &_useEulerAngles;

//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	FreeJoint();
	// Convenience Constructor
	FreeJoint(const std::string &name, Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
		  Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, 
		  /*bool useEulerAngles=true,*/ bool reverse=false);

	virtual ~FreeJoint();

	int numCoordinates() const OVERRIDE_11  { return _numMobilities; }

protected:
	// ModelComponent interface.
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
    void initStateFromProperties(SimTK::State& s) const OVERRIDE_11;
    void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;

private:
	SimTK::MobilizedBodyIndex _masslessBodyIndex;
	void setNull();
	void createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform);
//=============================================================================
};	// END of class FreeJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FREE_JOINT_H_


