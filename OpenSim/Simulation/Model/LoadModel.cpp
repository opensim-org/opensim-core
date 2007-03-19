// LoadModel.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/IO.h>
#include "LoadModel.h"
#include "ActuatorSet.h"
#include "ContactForceSet.h"

#ifdef __linux__
// Current solution for linux compatibility is to remap LoadLibrary/GetProcAddress to dlopen/dlsym
// using macros.  Also use macros for portable handles.
#include <dlfcn.h>
#define PORTABLE_HMODULE void *
#define PORTABLE_HINSTANCE void *
#define WINAPI
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
#define PORTABLE_HMODULE HMODULE
#define PORTABLE_HINSTANCE HINSTANCE
#define LoadLibraryError()
#endif




using namespace OpenSim;
using namespace std;

extern "C" {

// Type definition for the CreateModel() function
// This is necessary to create a pointer that can properly point to the
// CreateModel() function.
typedef OpenSim::AbstractModel* (*CREATEMODEL)();
typedef OpenSim::AbstractModel* (*CREATEMODEL_FILE)(const string &);
typedef OpenSim::AbstractModel* (*CREATEMODEL_ActuatorsContacts)(OpenSim::ActuatorSet*,OpenSim::ContactForceSet*);
typedef OpenSim::AbstractModel* (*CREATEMODEL_ParamsActuatorsContacts)(const string&,OpenSim::ActuatorSet*,OpenSim::ContactForceSet*);

}


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

OSIMSIMULATION_API
PORTABLE_HMODULE
WINAPI
LoadOpenSimLibrary(const char *lpLibFileName)
{
	string libraryExtension;
#ifdef __linux__
	libraryExtension=".so";
#endif
	string fixedLibFileName = IO::FixSlashesInFilePath(lpLibFileName);
	string actualLibFileName = fixedLibFileName + libraryExtension;
	static const string debugSuffix="_d";
	bool hasDebugSuffix = (IO::GetSuffix(fixedLibFileName,debugSuffix.size())==debugSuffix);

	PORTABLE_HINSTANCE libraryHandle = NULL;

	// If we're in debug mode and a release library is specified, or we're in release
	// mode and a debug library is specified, we'll first try loading the debug library,
	// and then try loading the release library.
	bool tryDebugThenRelease = false;
#ifdef _DEBUG
	if(!hasDebugSuffix) {
		cout << "Will try loading debug library first" << endl;
		tryDebugThenRelease = true;
	}
#else
	if(hasDebugSuffix) {
		cout << "WARNING: Trying to load a debug library into release osimSimulation" << endl;
		tryDebugThenRelease = true;
	}
#endif

	if(tryDebugThenRelease) {
		if(hasDebugSuffix) IO::RemoveSuffix(fixedLibFileName,debugSuffix.size());
		string debugLibFileName = fixedLibFileName + debugSuffix + libraryExtension;
		string releaseLibFileName = fixedLibFileName + libraryExtension;
		if ((libraryHandle = LoadLibrary(debugLibFileName.c_str()))) {
			cout << "Loaded library " << debugLibFileName << endl;
		} else {
			LoadLibraryError();
			cout << "Loading of debug library " << debugLibFileName << " Failed. Trying " << releaseLibFileName << endl;
			if ((libraryHandle = LoadLibrary(releaseLibFileName.c_str()))) {
				cout << "Loaded library " << releaseLibFileName << endl;
			} else {
				LoadLibraryError();
				cout << "Failed to load either debug or release library " << releaseLibFileName << endl;
			}
		}
	} else {
		if ((libraryHandle = LoadLibrary(actualLibFileName.c_str()))) {
			cout << "Loaded library " << actualLibFileName << endl;
		} else {
			LoadLibraryError();
			cout << "Failed to load library " << actualLibFileName << endl;
		}
	}

	return libraryHandle;
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

OSIMSIMULATION_API void 
LoadOpenSimLibraries(int argc,char **argv)
{
	int i;
	string option,value;
	PORTABLE_HINSTANCE library;
	for(i=0;i<argc;i++) {
		if(argv[i][0]!='-') continue;
		option = argv[i];
		if((i+1)>=argc) break;  // no more arguments.
		if((option=="-Library")||(option=="-L")) {
			string libraryName = argv[i+1];
			library = LoadOpenSimLibrary(libraryName.c_str());
			if(library==NULL) {
				cout<<"ERROR- library "<<value<<" could not be loaded.\n\n";
			} else {
				i++;
			}
		}
	}
}




//_____________________________________________________________________________
/**
 * Load and create a model from a dynamically loaded library (DLL).
 *
 * @param aModelLibraryName Name of the model DLL (e.g., rdBlock_D).  Do not
 * include the library (.lib) or DLL (.dll) extension in the name of the
 * model library.
 * @param aModelFileName Name of the model xml file (optional).
 * @return Pointer to an intance of the model, NULL if no model was created.
 */
OSIMSIMULATION_API AbstractModel* LoadModel(const string &aModelLibraryName, const string &aModelFileName)
{
	// LOAD MODEL LIBRARY
	PORTABLE_HINSTANCE modelLibrary = LoadOpenSimLibrary(aModelLibraryName.c_str());
	if(modelLibrary==NULL) {
		cout<<"ERROR- library for model "<<aModelLibraryName<<" could not be loaded.\n\n";
		return(NULL);
	}

	AbstractModel *model=NULL;
	if(aModelFileName!="") {
		// GET POINTER TO THE CreateModel_File() FUNCTION
		CREATEMODEL_FILE createModelFunction = (CREATEMODEL_FILE)GetProcAddress(modelLibrary,"CreateModel_File");
		if(createModelFunction==NULL) {
			cout<<"ERROR- function CreateModel_File() was not found in library "<<aModelLibraryName<<".\n\n";
			return(NULL);
		}

		// CREATE THE MODEL
		model = createModelFunction(aModelFileName);
		if(model==NULL) {
			cout<<"ERROR- model "<<aModelLibraryName<<" was not created.\n\n";
			return(NULL);
		}
	} else {
		// GET POINTER TO THE CreateModel() FUNCTION
		CREATEMODEL createModelFunction = (CREATEMODEL)GetProcAddress(modelLibrary,"CreateModel");
		if(createModelFunction==NULL) {
			cout<<"ERROR- function CreateModel() was not found in library "<<aModelLibraryName<<".\n\n";
			return(NULL);
		}

		// CALL THE CreatModel() FUNCTION
		model = createModelFunction();
		if(model==NULL) {
			cout<<"ERROR- model "<<aModelLibraryName<<" was not created.\n\n";
			return(NULL);
		}
	}
	
	return(model);
}
