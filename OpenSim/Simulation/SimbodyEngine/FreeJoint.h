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

A class implementing a Free joint.  The underlying implementation
in Simbody is a SimTK::MobilizedBody::Free.
Free joint allows unrestricted motion with three rotations and three translations.
Rotations are modeled similiarly to BallJoint -using quaternions with no
singulaties- while the translational generalized coordinates are XYZ
Translations along the parent axis. Generalized speeds are equal to the computed
angular velocities (\f$\vec{u} = \vec{\omega}\f$), not a differentiation of
position (\f$\vec{u} \neq \dot{\vec{q}}\f$).

\image html freeJoint.gif

@author Ajay Seth
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
    FreeJoint(const std::string &name,
        const PhysicalFrame& parent,
        const SimTK::Vec3& locationInParent,
        const SimTK::Vec3& orientationInParent,
        const PhysicalFrame& child,
        const SimTK::Vec3& locationInChild,
        const SimTK::Vec3& orientationInChild,
        bool reverse = false);

    virtual ~FreeJoint();

    int numCoordinates() const override  { return _numMobilities; }

protected:
    // ModelComponent interface.
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendInitStateFromProperties(SimTK::State& s) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

private:
    SimTK::MobilizedBodyIndex _masslessBodyIndex;
    void setNull();
//=============================================================================
};  // END of class FreeJoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FREE_JOINT_H_
