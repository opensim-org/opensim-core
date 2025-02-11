/* -------------------------------------------------------------------------- *
 *                  OpenSim:  RegisterTypes_osimAnalyses.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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


#include <OpenSim/Common/Object.h>
#include "osimAnalyses.h"   // Add headers for new Analyses in osimAnalyses.h rather than here so they are available to the API.
#include <string>
#include <iostream>
#include <exception>

using namespace OpenSim;
using namespace std;

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimAnalysesInstantiator instantiator;
#endif
     
//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Analyses library.
 */
OSIMANALYSES_API void RegisterTypes_osimAnalyses()
{
  try {

    Object::registerType( Kinematics() );
    Object::registerType( Actuation());
    Object::registerType( PointKinematics() );
    Object::registerType( BodyKinematics() );
    Object::registerType( MuscleAnalysis() );

    Object::registerType( JointReaction() );
    Object::registerType( StaticOptimization() );
    Object::registerType( ForceReporter() );
    Object::registerType( StatesReporter() );
    Object::registerType( InducedAccelerations() );
    Object::RegisterType( ProbeReporter() );
    Object::RegisterType( IMUDataReporter() );
    
    Object::RegisterType( OutputReporter() );

  } catch (const std::exception& e) {
    log_error("Error during osimAnalyses Object registration: {}", e.what());
  }
}

osimAnalysesInstantiator::osimAnalysesInstantiator() 
{ 
        registerDllClasses(); 
} 
void osimAnalysesInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimAnalyses(); 
} 
