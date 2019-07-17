#ifndef MOCO_OSIMMOCO_H
#define MOCO_OSIMMOCO_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: osimMoco.h                                                   *
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

#include "Common/TableProcessor.h"
#include "Components/ActivationCoordinateActuator.h"
#include "Components/DeGrooteFregly2016Muscle.h"
#include "Components/DiscreteForces.h"
#include "Components/ModelFactory.h"
#include "Components/PositionMotion.h"
#include "Components/SmoothSphereHalfSpaceForce.h"
#include "Components/StationPlaneContactForce.h"
#include "MocoBounds.h"
#include "MocoCasADiSolver/MocoCasADiSolver.h"
#include "MocoConstraint.h"
#include "MocoControlBoundConstraint.h"
#include "MocoCost/MocoControlCost.h"
#include "MocoCost/MocoControlTrackingCost.h"
#include "MocoCost/MocoJointReactionCost.h"
#include "MocoCost/MocoMarkerFinalCost.h"
#include "MocoCost/MocoMarkerTrackingCost.h"
#include "MocoCost/MocoOrientationTrackingCost.h"
#include "MocoCost/MocoStateTrackingCost.h"
#include "MocoCost/MocoTranslationTrackingCost.h"
#include "MocoInverse.h"
#include "MocoParameter.h"
#include "MocoProblem.h"
#include "MocoSolver.h"
#include "MocoStudy.h"
#include "MocoTrack.h"
#include "MocoTrajectory.h"
#include "MocoTropterSolver.h"
#include "MocoUtilities.h"
#include "MocoWeightSet.h"
#include "ModelOperators.h"
#include "ModelProcessor.h"
#include "RegisterTypes_osimMoco.h"

#endif // MOCO_OSIMMOCO_H
