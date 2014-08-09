// osimSimmFileWriterDLL.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2008, Stanford University. All rights reserved.
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//=============================================================================
// INCLUDES
//=============================================================================
#include "osimSimmFileWriterDLL.h"
#include <iostream>


using namespace std;

//
// Define Plugin_Attach and Plugin_Detach below to be called by both windows and linux
//
static void Plugin_Attach()
{
    //cout<<"\n-------------------------------------------------------\n";
    //cout<<"Library osimSimmFileWriterDLL...\n";
    //cout<<"-------------------------------------------------------\n\n";
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
