/* -------------------------------------------------------------------------- *
 *                       OpenSim:  osimExpPluginDLL.cpp                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "osimExpPluginDLL.h"
#include <iostream>
#include "RegisterTypes_osimExpPlugin.h"


using namespace std;

//
// Define Plugin_Attach and Plugin_Detach below to be called by both Windows and Linux
//
static void Plugin_Attach()
{
    //cout<<"\n-------------------------------------------------------\n";
    //cout<<"Library Analyses...\n";
    //cout<<"-------------------------------------------------------\n\n";
}

static void Plugin_Detach()
{
}

//
// The code below handles both Windows and Linux library entrypoints
//
#if defined(_WIN32)
//=============================================================================
// DLL Main Entry Point
//=============================================================================
//_____________________________________________________________________________
/**
 * This routine is called when the dll is loaded I believe.
 */
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
                )
{
   switch (ul_reason_for_call)
   {
      case DLL_PROCESS_ATTACH:
         Plugin_Attach();
         break;

      case DLL_PROCESS_DETACH:
         Plugin_Detach();
         break;

      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
            break;
    }

    return TRUE;
}
#elif defined(__GNUC__)
static void __attribute__((constructor)) Shared_Object_Constructor()
{
   Plugin_Attach();
}
static void __attribute__((destructor)) Shared_Object_Destructor()
{
   Plugin_Detach();
}
#else
    #error "Unsupported compiler."
#endif
