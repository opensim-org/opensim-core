/* -------------------------------------------------------------------------- *
 * OpenSim Moco: RegisterTypes_osimMoco.cpp                                   *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "RegisterTypes_osimMoco.h"

#include "Components/AccelerationMotion.h"
#include "Components/ControlDistributor.h"
#include "Components/DiscreteForces.h"
#include "Components/StationPlaneContactForce.h"
#include "MocoBounds.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoControlBoundConstraint.h"
#include "MocoOutputBoundConstraint.h"
#include "MocoStateBoundConstraint.h"
#include "MocoFrameDistanceConstraint.h"
#include "MocoGoal/MocoAccelerationTrackingGoal.h"
#include "MocoGoal/MocoAngularVelocityTrackingGoal.h"
#include "MocoGoal/MocoContactImpulseTrackingGoal.h"
#include "MocoGoal/MocoContactTrackingGoal.h"
#include "MocoGoal/MocoControlGoal.h"
#include "MocoGoal/MocoControlTrackingGoal.h"
#include "MocoGoal/MocoExpressionBasedParameterGoal.h"
#include "MocoGoal/MocoGoal.h"
#include "MocoGoal/MocoInitialActivationGoal.h"
#include "MocoGoal/MocoInitialForceEquilibriumDGFGoal.h"
#include "MocoGoal/MocoInitialVelocityEquilibriumDGFGoal.h"
#include "MocoGoal/MocoJointReactionGoal.h"
#include "MocoGoal/MocoMarkerFinalGoal.h"
#include "MocoGoal/MocoMarkerTrackingGoal.h"
#include "MocoGoal/MocoOrientationTrackingGoal.h"
#include "MocoGoal/MocoOutputGoal.h"
#include "MocoGoal/MocoPeriodicityGoal.h"
#include "MocoGoal/MocoStateTrackingGoal.h"
#include "MocoGoal/MocoStepLengthAsymmetryGoal.h"
#include "MocoGoal/MocoStepTimeAsymmetryGoal.h"
#include "MocoGoal/MocoSumSquaredStateGoal.h"
#include "MocoGoal/MocoTranslationTrackingGoal.h"
#include "MocoGoal/MocoGeneralizedForceTrackingGoal.h"
#include "MocoInverse.h"
#include "MocoParameter.h"
#include "MocoProblem.h"
#include "MocoScaleFactor.h"
#include "MocoStudy.h"
#include "MocoTrack.h"
#include "MocoTropterSolver.h"
#include "MocoWeightSet.h"
#include "ModelOperatorsDGF.h"
#include <exception>
#include <iostream>

using namespace OpenSim;

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimMocoInstantiator instantiator;
#endif

OSIMMOCO_API void RegisterTypes_osimMoco() {
    try {
        Object::registerType(MocoFinalTimeGoal());
        Object::registerType(MocoAverageSpeedGoal());
        Object::registerType(MocoWeight());
        Object::registerType(MocoWeightSet());
        Object::registerType(MocoStateTrackingGoal());
        Object::registerType(MocoMarkerTrackingGoal());
        Object::registerType(MocoMarkerFinalGoal());
        Object::registerType(MocoContactTrackingGoal());
        Object::registerType(MocoContactTrackingGoalGroup());
        Object::registerType(MocoContactImpulseTrackingGoal());
        Object::registerType(MocoContactImpulseTrackingGoalGroup());
        Object::registerType(MocoControlGoal());
        Object::registerType(MocoSumSquaredStateGoal());
        Object::registerType(MocoControlTrackingGoal());
        Object::registerType(MocoExpressionBasedParameterGoal());
        Object::registerType(MocoInitialActivationGoal());
        Object::registerType(MocoInitialVelocityEquilibriumDGFGoal());
        Object::registerType(MocoInitialForceEquilibriumDGFGoal());
        Object::registerType(MocoJointReactionGoal());
        Object::registerType(MocoOrientationTrackingGoal());
        Object::registerType(MocoTranslationTrackingGoal());
        Object::registerType(MocoAngularVelocityTrackingGoal());
        Object::registerType(MocoAccelerationTrackingGoal());
        Object::registerType(MocoPeriodicityGoalPair());
        Object::registerType(MocoPeriodicityGoal());
        Object::registerType(MocoOutputGoal());
        Object::registerType(MocoInitialOutputGoal());
        Object::registerType(MocoFinalOutputGoal());
        Object::registerType(MocoStepTimeAsymmetryGoal());
        Object::registerType(MocoStepLengthAsymmetryGoal());
        Object::registerType(MocoGeneralizedForceTrackingGoal());
        Object::registerType(MocoBounds());
        Object::registerType(MocoInitialBounds());
        Object::registerType(MocoFinalBounds());
        Object::registerType(MocoVariableInfo());
        Object::registerType(MocoScaleFactor());
        Object::registerType(MocoParameter());
        Object::registerType(MocoPhase());
        Object::registerType(MocoProblem());
        Object::registerType(MocoStudy());

        Object::registerType(MocoInverse());
        Object::registerType(MocoTrack());

        Object::registerType(MocoTropterSolver());

        Object::registerType(MocoControlBoundConstraint());
        Object::registerType(MocoOutputBoundConstraint());
        Object::registerType(MocoStateBoundConstraint());
        Object::registerType(MocoFrameDistanceConstraint());
        Object::registerType(MocoFrameDistanceConstraintPair());

        Object::registerType(MocoCasADiSolver());

        Object::registerType(ModOpReplaceMusclesWithDeGrooteFregly2016());
        Object::registerType(ModOpTendonComplianceDynamicsModeDGF());
        Object::registerType(ModOpIgnorePassiveFiberForcesDGF());
        Object::registerType(ModOpScaleActiveFiberForceCurveWidthDGF());

        Object::registerType(AckermannVanDenBogert2010Force());
        Object::registerType(MeyerFregly2016Force());
        Object::registerType(EspositoMiller2018Force());

        Object::registerType(DiscreteForces());
        Object::registerType(AccelerationMotion());
        Object::registerType(ControlDistributor());

    } catch (const std::exception& e) {
        std::cerr << "ERROR during osimMoco Object registration:\n"
                  << e.what() << std::endl;
    }
}

osimMocoInstantiator::osimMocoInstantiator() { registerDllClasses(); }

void osimMocoInstantiator::registerDllClasses() { RegisterTypes_osimMoco(); }
