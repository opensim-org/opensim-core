// AnalysesDLL.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Author:  Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//=============================================================================
// INCLUDES
//=============================================================================
#include "suAnalysesDLL.h"
#include <iostream>
#include "RegisterTypes_suAnalyses.h"


using namespace std;

//
// Define Plugin_Attach and Plugin_Detach below to be called by both windows and linux
//
static void Plugin_Attach()
{
	cout<<"\n-------------------------------------------------------\n";
	cout<<"Library Analyses...\n";
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
