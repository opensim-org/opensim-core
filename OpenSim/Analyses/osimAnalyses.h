#ifndef _osimAnalyses_h_
#define _osimAnalyses_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  osimAnalyses.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include <OpenSim/Common/FunctionSet.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/GCVSplineSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Object.h>

#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Model/ActivationFiberLengthMuscle.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/ConstraintSet.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include <OpenSim/Simulation/Model/ExternalForce.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/RollingOnSurfaceConstraint.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>

#include <OpenSim/Actuators/CoordinateActuator.h>

#include "Actuation.h"
#include "BodyKinematics.h"
#include "ForceReporter.h"
#include "InducedAccelerations.h"
#include "InducedAccelerationsSolver.h"
#include "InverseDynamics.h"
#include "JointReaction.h"
#include "Kinematics.h"
#include "MuscleAnalysis.h"
#include "PointKinematics.h"
#include "ProbeReporter.h"
#include "RegisterTypes_osimAnalyses.h"
#include "StatesReporter.h"
#include "StaticOptimization.h"
#include "StaticOptimizationTarget.h"
#include "osimAnalysesDLL.h"

#endif // _osimAnalyses_h_
