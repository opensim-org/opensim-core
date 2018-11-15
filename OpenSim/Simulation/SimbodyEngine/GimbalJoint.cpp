/* -------------------------------------------------------------------------- *
 *                        OpenSim:  GimbalJoint.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "GimbalJoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include "simbody/internal/MobilizedBody_Gimbal.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void GimbalJoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    createMobilizedBody<MobilizedBody::Gimbal>(system);
}

void GimbalJoint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(s))
        return;

    double xangle = getCoordinate(GimbalJoint::Coord::Rotation1X).getDefaultValue();
    double yangle = getCoordinate(GimbalJoint::Coord::Rotation2Y).getDefaultValue();
    double zangle = getCoordinate(GimbalJoint::Coord::Rotation3Z).getDefaultValue();
    Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);

    //GimbalJoint* mutableThis = const_cast<GimbalJoint*>(this);
    getChildFrame().getMobilizedBody().setQToFitRotation(s, r);
}

void GimbalJoint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    // Override default behavior in case of quaternions.
    const MultibodySystem&        system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
        Rotation r = getChildFrame().getMobilizedBody().getBodyRotation(state);

        Vec3 angles = r.convertRotationToBodyFixedXYZ();
        updCoordinate(GimbalJoint::Coord::Rotation1X).setDefaultValue(angles[0]);
        updCoordinate(GimbalJoint::Coord::Rotation2Y).setDefaultValue(angles[1]);
        updCoordinate(GimbalJoint::Coord::Rotation3Z).setDefaultValue(angles[2]);
    }
}
