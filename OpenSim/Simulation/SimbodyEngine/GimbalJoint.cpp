/* -------------------------------------------------------------------------- *
 *                        OpenSim:  GimbalJoint.cpp                           *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "GimbalJoint.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace SimTK;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
GimbalJoint::~GimbalJoint()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
GimbalJoint::GimbalJoint() : Joint()
{
    setAuthors("Tim Dorn, Ajay Seth");
    constructCoordinates();
}
//_____________________________________________________________________________
/**
 * Convenience Constructor.
 */
GimbalJoint::GimbalJoint(const std::string &name, OpenSim::Body& parent, 
                     Vec3 locationInParent, Vec3 orientationInParent,
                     OpenSim::Body& body, Vec3 locationInBody, Vec3 orientationInBody, 
                     bool reverse) :
            Joint(name, parent, locationInParent,orientationInParent,
                    body, locationInBody, orientationInBody, reverse)
{
    setAuthors("Tim Dorn, Ajay Seth");
    constructCoordinates();
}

//=============================================================================
// Simbody Model building.
//=============================================================================
//_____________________________________________________________________________
void GimbalJoint::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    createMobilizedBody<MobilizedBody::Gimbal>(system);
}

void GimbalJoint::extendInitStateFromProperties(SimTK::State& s) const
{
    Super::extendInitStateFromProperties(s);

    const MultibodySystem& system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (matter.getUseEulerAngles(s))
        return;

    const CoordinateSet& coordinateSet = get_CoordinateSet();

    double xangle = coordinateSet[0].getDefaultValue();
    double yangle = coordinateSet[1].getDefaultValue();
    double zangle = coordinateSet[2].getDefaultValue();
    Rotation r(BodyRotationSequence, xangle, XAxis, yangle, YAxis, zangle, ZAxis);
<<<<<<< HEAD
    
    GimbalJoint* mutableThis = const_cast<GimbalJoint*>(this);
    matter.getMobilizedBody(getChildBody().getMobilizedBodyIndex()).setQToFitRotation(s, r);
=======
	
	GimbalJoint* mutableThis = const_cast<GimbalJoint*>(this);
	getChildBody().getMobilizedBody().setQToFitRotation(s, r);
>>>>>>> master
}

void GimbalJoint::extendSetPropertiesFromState(const SimTK::State& state)
{
    Super::extendSetPropertiesFromState(state);

    // Override default behavior in case of quaternions.
    const MultibodySystem&        system = _model->getMultibodySystem();
    const SimbodyMatterSubsystem& matter = system.getMatterSubsystem();
    if (!matter.getUseEulerAngles(state)) {
<<<<<<< HEAD
        Rotation r = matter.getMobilizedBody(getChildBody().getMobilizedBodyIndex()).getBodyRotation(state);
=======
		Rotation r = getChildBody().getMobilizedBody().getBodyRotation(state);
>>>>>>> master
        Vec3 angles = r.convertRotationToBodyFixedXYZ();
    
        const CoordinateSet& coordinateSet = get_CoordinateSet();

        coordinateSet[0].setDefaultValue(angles[0]);
        coordinateSet[1].setDefaultValue(angles[1]);
        coordinateSet[2].setDefaultValue(angles[2]);
    }
}
