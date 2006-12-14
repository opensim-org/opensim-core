// rdCMCDLL.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


//=============================================================================
// INCLUDES
//=============================================================================
#include "rdCMCDLL.h"
#include <iostream>
#include "RegisterTypes_rdCMC.h"

using namespace std;

//
// Define Plugin_Attach and Plugin_Detach below to be called by both windows and linux
//
static void Plugin_Attach()
{
   cout<<"\n-------------------------------------------------------\n";
   cout<<"Library rdCMC...\n";
   cout<<"Computed Muscle Control\n";
   cout<<"US Patent No. 6,750,866";
   RegisterTypes_rdCMC();
   cout<<"-------------------------------------------------------\n\n";
}

static void Plugin_Detach()
{
}

//
// The code below handles both windows and linux library entrypoints
//
#if defined(WIN32)
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
      case DLL_THREAD_ATTACH:
         Plugin_Attach();
         break;

      case DLL_PROCESS_DETACH:
      case DLL_THREAD_DETACH:
         Plugin_Detach();
         break;
    }

    return TRUE;
}
#elif defined(__linux__)
static void __attribute__((constructor)) Shared_Object_Constructor()
{
   Plugin_Attach();
}
static void __attribute__((destructor)) Shared_Object_Destructor()
{
   Plugin_Detach();
}
#endif
