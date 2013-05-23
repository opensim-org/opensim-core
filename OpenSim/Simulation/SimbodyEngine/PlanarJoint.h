#ifndef OPENSIM_PLANAR_JOINT_H_
#define OPENSIM_PLANAR_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  SliderJoint.h                           *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A PlanarJoint provides three DoFs: rotation about the the common Z of the
 * parent and child joint frames, X and Y translation in the parent body's 
 * joint frame. The underlying Simbody implementation is a 
 * MobilizedBody::Planar. 
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API PlanarJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(PlanarJoint, Joint);

private:
	static const int _numMobilities = 3;
//=============================================================================
// DATA
//=============================================================================
protected:

	/** Slider has no additional properties*/


//=============================================================================
// METHODS
//=============================================================================
public:
	// CONSTRUCTION
	PlanarJoint();

	// Convenience constructor
	PlanarJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
					OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, 
				    bool reverse=false);

	int numCoordinates() const { return _numMobilities; }

protected:
	/** Model component interface */
	void connectToModel(Model& aModel) OVERRIDE_11;
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

private:
	void createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform);

//=============================================================================
};	// END of class PlanarJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PLANAR_JOINT_H_


