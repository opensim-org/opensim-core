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

#ifdef WIN32

//=============================================================================
// DLL Main Entry Point
//=============================================================================
//_____________________________________________________________________________
/**
 *	This routine is called when the dll is loaded.
 */
BOOL APIENTRY DllMain( HANDLE hModule, 
                       DWORD  ul_reason_for_call, 
                       LPVOID lpReserved
					 )
{
    switch (ul_reason_for_call)
	{
		case DLL_PROCESS_ATTACH:
			cout<<"\n-------------------------------------------------------\n";
			cout<<"Library Analyses...\n";
			cout<<"-------------------------------------------------------\n\n";
			break;
		case DLL_THREAD_ATTACH:
			cout<<"\n-------------------------------------------------------\n";
			cout<<"Library Analyses...\n";
			cout<<"-------------------------------------------------------\n\n";
			break;
		case DLL_THREAD_DETACH:
			break;
		case DLL_PROCESS_DETACH:
			break;
    }

    return TRUE;
}

#endif