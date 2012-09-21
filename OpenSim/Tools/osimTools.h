#ifndef _osimTools_h_
#define _osimTools_h_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  osimTools.h                            *
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

#include "ScaleTool.h"
#include "CMCTool.h"
#include "ForwardTool.h"
#include "AnalyzeTool.h"

#include "InverseKinematicsTool.h"
#include "GenericModelMaker.h"
#include "TrackingTask.h"
#include "MuscleStateTrackingTask.h"
#include "IKCoordinateTask.h"
#include "IKMarkerTask.h"
#include "IKTaskSet.h"
#include "MarkerPair.h"
#include "MarkerPairSet.h"
#include "MarkerPlacer.h"
#include "Measurement.h"
#include "MeasurementSet.h"
#include "ModelScaler.h"
#include "CMC.h"
#include "CMC_Point.h"
#include "CMC_Joint.h"
#include "SMC_Joint.h"
#include "CMC_TaskSet.h"
#include "CorrectionController.h"
#include "RegisterTypes_osimTools.h"	// to expose RegisterTypes_osimTools

#endif // _osimTools_h_
