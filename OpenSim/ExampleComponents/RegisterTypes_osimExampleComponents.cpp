/* -------------------------------------------------------------------------- *
 *             OpenSim:  RegisterTypes_osimExampleComponents.cpp              *
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

#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimExampleComponents.h"
#include "ToyPropMyoController.h"
#include "ToyReflexController.h"
#include "HopperDevice.h"

#include <string>
#include <iostream>
#include <exception>

using namespace std;
using namespace OpenSim;

#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
   static osimExampleComponentsInstantiator osimExampleComponentsInstantiator;
#endif

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimExampleComponents library.
 */
OSIMEXAMPLECOMPONENTS_API void RegisterTypes_osimExampleComponents()
{
  try {

      Object::registerType( ToyReflexController() );
      Object::registerType( ToyPropMyoController() );
      Object::registerType( HopperDevice() );
    
  } catch (const std::exception& e) {
    std::cerr 
        << "ERROR during osimExampleComponents Object registration:\n"
        << e.what() << "\n";
  }
}


osimExampleComponentsInstantiator::osimExampleComponentsInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimExampleComponentsInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimExampleComponents(); 
} 
