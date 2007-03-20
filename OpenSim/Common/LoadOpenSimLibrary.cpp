// LoadOpenSimLibrary.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include "IO.h"
#include "LoadOpenSimLibrary.h"

#ifdef __linux__
// Current solution for linux compatibility is to remap LoadLibrary/GetProcAddress to dlopen/dlsym
// using macros.  Also use macros for portable handles.
#include <dlfcn.h>
// LoadLibrary used to be a macro for dlopen but we want to transparently support
// adding the "lib" prefix to library names when loading them, so made it a function
static void *LoadLibrary(const char *name) {
	void *lib = dlopen(name, RTLD_LAZY);
	if(!lib) {
		std::string libName = OpenSim::IO::GetFileNameFromURI(name);
		if(libName.size()<3 || libName.substr(0,3)!="lib") { // if it doesn't already have lib prefix
			libName = OpenSim::IO::getParentDirectory(name) + "lib" + libName;
			std::cout << "Loading " << name << " failed, trying " << libName << " (for linux compatibility)" << std::endl;
			lib = dlopen(libName.c_str(), RTLD_LAZY); 
		}
	}
	return lib;
}
#define GetProcAddress(handle, proc) dlsym(handle, proc)
#define LoadLibraryError() { char* err=dlerror(); if(err) cout<<"dlerror: "<<err<<endl; }
#else
#define LoadLibraryError()
#endif


using namespace OpenSim;
using namespace std;

//_____________________________________________________________________________
/**
 * A wrapper around Window's LoadLibrary that implements library naming
 * convention and loading policy on windows which follows:
 * If you're loading osimSimulation_D and other libraries that do not have a
 * trailing _D an _D is appended to the library file name.  If loading of that
 * fails, we revert to using the non _D file instead, if that fails we give
 * error and return 0. A reciprocal treatment for release libraries is
 * implemented. I tried to keep this function in the same file to try
 * to localize platform specific code. -Ayman
 *
 * @param lpLibFileName Name of the library without either the .lib or .dll
 * extension.
 * @return Pointer to the loaded library, NULL on error.
 */

OSIMCOMMON_API
OPENSIM_PORTABLE_HMODULE
WINAPI
OpenSim::LoadOpenSimLibrary(const std::string &lpLibFileName, bool verbose)
{
	string libraryExtension;
#ifdef __linux__
	libraryExtension=".so";
#endif
	string fixedLibFileName = IO::FixSlashesInFilePath(lpLibFileName);
	string actualLibFileName = fixedLibFileName + libraryExtension;
	static const string debugSuffix="_d";
	bool hasDebugSuffix = (IO::GetSuffix(fixedLibFileName,debugSuffix.size())==debugSuffix);

	OPENSIM_PORTABLE_HINSTANCE libraryHandle = NULL;

	// If we're in debug mode and a release library is specified, or we're in release
	// mode and a debug library is specified, we'll first try loading the debug library,
	// and then try loading the release library.
	bool tryDebugThenRelease = false;
#ifdef _DEBUG
	if(!hasDebugSuffix) {
		if(verbose) cout << "Will try loading debug library first" << endl;
		tryDebugThenRelease = true;
	}
#else
	if(hasDebugSuffix) {
		if(verbose) cout << "WARNING: Trying to load a debug library into release osimSimulation" << endl;
		tryDebugThenRelease = true;
	}
#endif

	if(tryDebugThenRelease) {
		if(hasDebugSuffix) IO::RemoveSuffix(fixedLibFileName,debugSuffix.size());
		string debugLibFileName = fixedLibFileName + debugSuffix + libraryExtension;
		string releaseLibFileName = fixedLibFileName + libraryExtension;
		if ((libraryHandle = LoadLibrary(debugLibFileName.c_str()))) {
			if(verbose) cout << "Loaded library " << debugLibFileName << endl;
		} else {
			LoadLibraryError();
			if(verbose) cout << "Loading of debug library " << debugLibFileName << " Failed. Trying " << releaseLibFileName << endl;
			if ((libraryHandle = LoadLibrary(releaseLibFileName.c_str()))) {
				if(verbose) cout << "Loaded library " << releaseLibFileName << endl;
			} else {
				LoadLibraryError();
				if(verbose) cout << "Failed to load either debug or release library " << releaseLibFileName << endl;
			}
		}
	} else {
		if ((libraryHandle = LoadLibrary(actualLibFileName.c_str()))) {
			if(verbose) cout << "Loaded library " << actualLibFileName << endl;
		} else {
			LoadLibraryError();
			if(verbose) cout << "Failed to load library " << actualLibFileName << endl;
		}
	}

	return libraryHandle;
}

OSIMCOMMON_API void
OpenSim::LoadOpenSimLibrary(const std::string &aLibraryName)
{
	OPENSIM_PORTABLE_HINSTANCE library = LoadOpenSimLibrary(aLibraryName.c_str(), false);
	if(!library) { cout<<"ERROR- library "<<aLibraryName<<" could not be loaded.\n\n"; }
}

//_____________________________________________________________________________
/**
 * A function for loading libraries specified in a command line.
 * LoadOpenSimLibrary() is used to load each library.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @see LoadOpenSimLibrary()
 */

OSIMCOMMON_API void 
OpenSim::LoadOpenSimLibraries(int argc,char **argv)
{
	int i;
	string option,value;
	OPENSIM_PORTABLE_HINSTANCE library;
	for(i=0;i<argc;i++) {
		if(argv[i][0]!='-') continue;
		option = argv[i];
		if((i+1)>=argc) break;  // no more arguments.
		if((option=="-Library")||(option=="-L")) {
			string libraryName = argv[i+1];
			library = LoadOpenSimLibrary(libraryName.c_str(), true);
			if(library==NULL) {
				cout<<"ERROR- library "<<value<<" could not be loaded.\n\n";
			} else {
				i++;
			}
		}
	}
}
