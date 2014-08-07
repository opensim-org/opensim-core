/* -------------------------------------------------------------------------- *
 *                 OpenSim:  RegisterTypes_osimActuators.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimActuators.h"

#include "CoordinateActuator.h"
#include "PointActuator.h"
#include "TorqueActuator.h"
#include "PointToPointActuator.h"
#include "ClutchedPathSpring.h"

#include "Thelen2003Muscle.h"
#include "Thelen2003Muscle_Deprecated.h"

#include "Schutte1993Muscle_Deprecated.h"
#include "Delp1990Muscle_Deprecated.h"
#include "SpringGeneralizedForce.h"

#include "RigidTendonMuscle.h"

#include "ActiveForceLengthCurve.h"
#include "ForceVelocityCurve.h"
#include "ForceVelocityInverseCurve.h"
#include "TendonForceLengthCurve.h"
#include "FiberForceLengthCurve.h"
#include "FiberCompressiveForceLengthCurve.h"
#include "FiberCompressiveForceCosPennationCurve.h"
#include "MuscleFirstOrderActivationDynamicModel.h"
#include "MuscleSecondOrderActivationDynamicModel.h"

#include "MuscleFixedWidthPennationModel.h"

#include "Millard2012EquilibriumMuscle.h"
#include "Millard2012AccelerationMuscle.h"

// Awaiting new component architecture that supports subcomponents with states.
//#include "ConstantMuscleActivation.h"
//#include "ZerothOrderMuscleActivationDynamics.h"
//#include "FirstOrderMuscleActivationDynamics.h"

using namespace OpenSim;
using namespace std;

static osimActuatorsInstantiator instantiator;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Simulation library.
 */
OSIMACTUATORS_API void RegisterTypes_osimActuators()
{
    try {

        Object::registerType( CoordinateActuator() );
        Object::registerType( PointActuator() );
        Object::registerType( TorqueActuator() );
        Object::registerType( PointToPointActuator() );
        Object::registerType( ClutchedPathSpring() );

        Object::registerType( Thelen2003Muscle() );
        Object::registerType( Thelen2003Muscle_Deprecated() );
        Object::registerType( Schutte1993Muscle_Deprecated() );
        Object::registerType( Delp1990Muscle_Deprecated() );
        Object::registerType( SpringGeneralizedForce() );
        Object::registerType( RigidTendonMuscle() );

        Object::RegisterType( ActiveForceLengthCurve() );
        Object::RegisterType( ForceVelocityCurve() );
        Object::RegisterType( ForceVelocityInverseCurve() );
        Object::RegisterType( TendonForceLengthCurve() );
        Object::RegisterType( FiberForceLengthCurve() );
        Object::RegisterType( FiberCompressiveForceLengthCurve() );
        Object::RegisterType( FiberCompressiveForceCosPennationCurve() );

        Object::RegisterType(MuscleFirstOrderActivationDynamicModel());
        Object::RegisterType(MuscleFixedWidthPennationModel());

        Object::RegisterType(Millard2012EquilibriumMuscle());
        Object::RegisterType(Millard2012AccelerationMuscle());

        //Object::RegisterType( ConstantMuscleActivation() );
        //Object::RegisterType( ZerothOrderMuscleActivationDynamics() );
        //Object::RegisterType( FirstOrderMuscleActivationDynamics() );

        // OLD Versions
        //Associate an instance with old name to help deserialization
        // This has to be done after the new Type is registered
        Object::renameType("GeneralizedForce", "CoordinateActuator");
        Object::renameType("Force", "PointActuator");
        Object::renameType("Torque", "TorqueActuator");
        Object::renameType("Schutte1993Muscle", "Schutte1993Muscle_Deprecated");
        Object::renameType("Delp1990Muscle", "Delp1990Muscle_Deprecated");

        //Object::RenameType("Thelen2003Muscle", "Thelen2003Muscle_Deprecated");

    } catch (const std::exception& e) {
        std::cerr
                << "ERROR during osimActuators Object registration:\n"
                << e.what() << "\n";
    }
}

osimActuatorsInstantiator::osimActuatorsInstantiator()
{
    registerDllClasses();
}

void osimActuatorsInstantiator::registerDllClasses()
{
    RegisterTypes_osimActuators();
}
