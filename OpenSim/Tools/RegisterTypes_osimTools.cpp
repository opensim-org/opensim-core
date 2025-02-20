/* -------------------------------------------------------------------------- *
 *                   OpenSim:  RegisterTypes_osimTools.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimTools.h"

#include "ScaleTool.h"
//#include "IKTool.h"
//#include "FIKSTool.h"
#include "CMCTool.h"
#include "RRATool.h"
#include "ForwardTool.h"
#include "AnalyzeTool.h"
#include "InverseKinematicsTool.h"
#include "IMUInverseKinematicsTool.h"

#include "InverseDynamicsTool.h"

#include "GenericModelMaker.h"
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
#include "MuscleStateTrackingTask.h"

#include <string>
#include <iostream>
#include <exception>

using namespace std;
using namespace OpenSim;

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimToolsInstantiator instantiator;
#endif

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimTools library.
 */
OSIMTOOLS_API void RegisterTypes_osimTools()
{
  try {

    Object::registerType( ScaleTool() );
    //Object::registerType( IKTool() );
    Object::registerType( CMCTool() );
    Object::registerType( RRATool() );
    Object::registerType( ForwardTool() );
    Object::registerType( AnalyzeTool() );

    Object::registerType( GenericModelMaker() );
    Object::registerType( IKCoordinateTask() );
    Object::registerType( IKMarkerTask() );
    Object::registerType( IKTaskSet() );
    Object::registerType( MarkerPair() );
    Object::registerType( MarkerPairSet() );
    Object::registerType( MarkerPlacer() );
    Object::registerType( Measurement() );
    Object::registerType( MeasurementSet() );
    Object::registerType( ModelScaler() );

    Object::registerType( CorrectionController() );
    Object::registerType( CMC() );
    Object::registerType( CMC_Joint() );
    Object::registerType( CMC_Point() );
    Object::registerType( MuscleStateTrackingTask() );
    Object::registerType( CMC_TaskSet() );

    Object::registerType( SMC_Joint() );
    Object::registerType( OrientationWeightSet());
    Object::registerType( InverseKinematicsTool() );
    Object::registerType( IMUInverseKinematicsTool());
    Object::registerType( InverseDynamicsTool() );
    // Old versions
    Object::RenameType("rdCMC_Joint",   "CMC_Joint");
    Object::RenameType("rdCMC_Point",   "CMC_Point");
    Object::RenameType("rdCMC_TaskSet", "CMC_TaskSet");

    Object::RenameType("IKTool", "InverseKinematicsTool");

  } catch (const std::exception& e) {
    log_error("Error during osimTools Object registration: {}.", e.what());
    log_error("");
  }
}


osimToolsInstantiator::osimToolsInstantiator()
{
       registerDllClasses();
}

void osimToolsInstantiator::registerDllClasses()
{
       RegisterTypes_osimTools();
}
