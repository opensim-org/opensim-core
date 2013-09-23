#ifndef OPENSIM_UNIVERSAL_JOINT_H_
#define OPENSIM_UNIVERSAL_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  UniversalJoint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Tim Dorn                                                        *
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
 * A class implementing a Universal joint.  The underlying implementation 
 * in Simbody is a MobilizedBody::Universal. Universal provides two DoF:
 * rotation about the x axis of the joint frames, followed by a rotation 
 * about the new y axis.
 *
 * @author Tim Dorn
 * @version 1.0
 */
class OSIMSIMULATION_API UniversalJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(UniversalJoint, Joint);

private:
    static const int _numMobilities = 2;
//=============================================================================
// DATA
//=============================================================================
protected:

    /** UniversalJoint has no additional properties*/


//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    UniversalJoint();
    // Convenience constructor
    UniversalJoint(const std::string &name, OpenSim::Body& parent, SimTK::Vec3 locationInParent, SimTK::Vec3 orientationInParent,
                    OpenSim::Body& body, SimTK::Vec3 locationInBody, SimTK::Vec3 orientationInBody, 
                    bool reverse=false);
    virtual ~UniversalJoint();

    int numCoordinates() const OVERRIDE_11 { return _numMobilities; }


protected:
    void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;

private:
    void createMobilizedBody(SimTK::Transform parentTransform, SimTK::Transform childTransform);
//=============================================================================
};	// END of class UniversalJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_UNIVERSAL_JOINT_H_


