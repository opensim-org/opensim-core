#ifndef OPENSIM_WELD_JOINT_H_
#define OPENSIM_WELD_JOINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  WeldJoint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
A class implementing a Weld joint. The underlying implementation in Simbody is
a SimTK::MobilizedBody::Weld. There is no relative motion of bodies joined by
a weld. Weld joints are often used to create composite bodies from
smaller simpler bodies. You can also get the reaction force at the weld in the
usual manner.

@author Ajay Seth
*/
class OSIMSIMULATION_API WeldJoint : public Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(WeldJoint, Joint);
//=============================================================================
// METHODS
//=============================================================================
public:
    /** Use Joint's constructors. @see Joint */
    using Joint::Joint;

protected:
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

//=============================================================================
};  // END of class WeldJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_WELD_JOINT_H_
