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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include "OutputReporter.h"
#include "Kinematics.h"
#include "Actuation.h"
#include "ForceReporter.h"
#include "PointKinematics.h"
#include "BodyKinematics.h"
#include "MuscleAnalysis.h"
#include "JointReaction.h"
#include "StaticOptimization.h"
#include "StatesReporter.h"
#include "InducedAccelerations.h"
#include "ProbeReporter.h"
#include "IMUDataReporter.h"
#include "RegisterTypes_osimAnalyses.h" // to expose RegisterTypes_Analyses

#endif // _osimAnalyses_h_
