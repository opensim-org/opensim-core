/* -------------------------------------------------------------------------- *
 *                          OpenSim:  FreeJoint.cpp                           *
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

//=============================================================================
// INCLUDES
//=============================================================================



//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// Simbody Model building.
//=============================================================================

void FreeJoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);
    createMobilizedBody<MobilizedBody::Free>(system);
}

void FreeJoint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(s)){
        int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.

        const CoordinateSet& coordinateSet = get_CoordinateSet();

        double xangle = coordinateSet.get(zero).getDefaultValue();
        double yangle = coordinateSet.get(1).getDefaultValue();
        double zangle = coordinateSet.get(2).getDefaultValue();
        Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);
        Vec3 t(coordinateSet.get(3).getDefaultValue(),
            coordinateSet.get(4).getDefaultValue(),
            coordinateSet.get(5).getDefaultValue());

        //FreeJoint* mutableThis = const_cast<FreeJoint*>(this);
        getChildFrame().getMobilizedBody().setQToFitTransform(s, Transform(r, t));
    }
}

void FreeJoint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    // Override default behavior in case of quaternions.
    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
        Rotation r = getChildFrame().getMobilizedBody().getMobilizerTransform(state).R();
        Vec3 t = getChildFrame().getMobilizedBody().getMobilizerTransform(state).p();

        Vec3 angles = r.convertRotationToBodyFixedXYZ();
        int zero = 0; // Workaround for really ridiculous Visual Studio 8 bug.
        
        const CoordinateSet& coordinateSet = get_CoordinateSet();
 
        coordinateSet.get(zero).setDefaultValue(angles[0]);
        coordinateSet.get(1).setDefaultValue(angles[1]);
        coordinateSet.get(2).setDefaultValue(angles[2]);
        coordinateSet.get(3).setDefaultValue(t[0]); 
        coordinateSet.get(4).setDefaultValue(t[1]); 
        coordinateSet.get(5).setDefaultValue(t[2]);
    }
}
