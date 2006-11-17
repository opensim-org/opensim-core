// LoadModel.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/IO.h>
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
#define LoadLibrary(name) dlopen(name, RTLD_LAZY)
#define GetProcAddress(handle, proc) dlsym(handle, proc)
#else
#define PORTABLE_HMODULE HMODULE
#define PORTABLE_HINSTANCE HINSTANCE
#endif




using namespace OpenSim;
using namespace std;

extern "C" {

// Type definition for the CreateModel() function
// This is necessary to create a pointer that can properly point to the
// CreateModel() function.
typedef OpenSim::Model* (*CREATEMODEL)();
typedef OpenSim::Model* (*CREATEMODEL_FILE)(const string &);
typedef OpenSim::Model* (*CREATEMODEL_ActuatorsContacts)(OpenSim::ActuatorSet*,OpenSim::ContactForceSet*);
typedef OpenSim::Model* (*CREATEMODEL_ParamsActuatorsContacts)(const string&,OpenSim::ActuatorSet*,OpenSim::ContactForceSet*);

}


static void PrintUsage(ostream &aOStream);


//_____________________________________________________________________________
/**
 * A wrapper around Window's LoadLibrary that implements library naming
 * convention and loading policy on windows which follows:
 * If you're loading rdSimulation_D and other libraries that do not have a
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

RDSIMULATION_API
PORTABLE_HMODULE
WINAPI
LoadOpenSimLibrary(const char *lpLibFileName)
{
	string libraryExtension;
#ifdef __linux__
	libraryExtension=".so";
#endif
	string fixedLibFileName = IO::FixSlashesInFilePath(lpLibFileName);
	string actualLibFileName(fixedLibFileName+libraryExtension);
	string debugSuffix="_d";
	char *locationOf_D=strstr(fixedLibFileName.c_str(), debugSuffix.c_str());
	bool hasDebugSuffix = (locationOf_D!= 0) && (strcmp(locationOf_D, debugSuffix.c_str())==0);

	PORTABLE_HINSTANCE libraryHandle = NULL;
#ifdef _DEBUG
	// NO _D SUFFIX
	// if library name has no trailing _D try to append it and load
	// find locaion of _D in lpLibFileName and make sure it's trailing
	if (!hasDebugSuffix) {
		// Append _D to lpLibFileName;
		cout << "WARNING: SUSPECT LOADING RELEASE LIB INTO DEBUG Simulation library." << endl;
		cout << "Trying to load a debug version ..." << endl;
		actualLibFileName = fixedLibFileName+debugSuffix+libraryExtension;
		// if that fails we'll try the one with no _D 
		if ((libraryHandle = LoadLibrary(actualLibFileName.c_str()))==0){
			cout << "Loading of Debug library " << actualLibFileName << " Failed. Trying " << fixedLibFileName << endl;
			// library with _D loading failed, try non _D version
			actualLibFileName = fixedLibFileName+libraryExtension;
			if ((libraryHandle = LoadLibrary(actualLibFileName.c_str()))!=0){
				cout << "Loaded library " << actualLibFileName << endl;
			}
			else
				cout << "Failed to load either debug or release library " << actualLibFileName << endl;
		}
		else
			cout << "Loaded library " << actualLibFileName << endl;

	// HAS _D SUFFIX
	} else {
		libraryHandle = LoadLibrary(actualLibFileName.c_str());
	}

#else
	// HAS _D SUFFIX
	// Here we're in release mode, highly unlikely to have a trailing _D intentionally!
	if (hasDebugSuffix){

		// try stripping the trailing _D first 
		cout << "WARNING: SUSPECT LOADING DEBUG LIB INTO RELEASE rdSimulation";
		cout << "Trying ";
		if ((libraryHandle = LoadLibrary(actualLibFileName.c_str()))==0){
			*locationOf_D = '\0';	// Strip trailing _D and retry (can we do that with const!)
			if ((libraryHandle = LoadLibrary(actualLibFileName.c_str()))!=0){
				cout << "Loaded library " << actualLibFileName << endl;
			}
			else
				cout << "Failed to load either debug or release library " << actualLibFileName << endl;

		}
		else
			cout << "Loaded library " << actualLibFileName << endl;

	// NO _D SUFFIX
	} else {
		libraryHandle = LoadLibrary(actualLibFileName.c_str());
	}
#endif
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

RDSIMULATION_API void 
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
 * @return Pointer to an intance of the model, NULL if no model was created.
 */
RDSIMULATION_API Model* LoadModel(const string &aModelLibraryName)
{
	// LOAD MODEL LIBRARY
	PORTABLE_HINSTANCE modelLibrary = LoadOpenSimLibrary(aModelLibraryName.c_str());
	if(modelLibrary==NULL) {
		cout<<"ERROR- library for model "<<aModelLibraryName<<" could not be loaded.\n\n";
		return(NULL);
	}

	// GET POINTER TO THE CreateModel() FUNCTION
	CREATEMODEL createModelFunction = (CREATEMODEL)GetProcAddress(modelLibrary,"CreateModel");
	if(createModelFunction==NULL) {
		cout<<"ERROR- function CreateModel() was not found in library "<<aModelLibraryName<<".\n\n";
		return(NULL);
	}

	// CALL THE CreatModel() FUNCTION
	Model *model = createModelFunction();
	if(model==NULL) {
		cout<<"ERROR- model "<<aModelLibraryName<<" was not created.\n\n";
		return(NULL);
	}
	
	return(model);
}


//_____________________________________________________________________________
/**
 * Load and create a model from a dynamically loaded library (DLL).
 *
 * @param argc Number of character strings in argv.
 * @param argv Array of character strings.
 * @return Pointer to an intance of the model, NULL if no model was created.
 */
RDSIMULATION_API Model* LoadModel(int argc,char **argv)
{
	// VARIABLES
	int i;
	string option,value;

	// PARSE THE COMMAND LINE FOR OTHER OPTIONS
	string actuators="",contacts="",params="";
	string modelFileName="",modelLibraryName="";
	for(i=1;i<argc;i++) {

		if(argv[i][0] != '-') continue;
		option = argv[i];

		if((argc<=1)||(option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")
			||(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
			PrintUsage(cout);
			return(NULL);

		} else if((option=="-Actuators")||(option=="-A")) {
			if((i+1)>=argc) continue;
			if(actuators=="") actuators = argv[i+1];

		} else if((option=="-Contacts")||(option=="-C")) {
			if((i+1)>=argc) continue;
			if(contacts=="") contacts = argv[i+1];

		} else if((option=="-Params")||(option=="-P")) {
			if((i+1)>=argc) continue;
			if(params=="") params = argv[i+1];

		} else if((option=="-ModelFile")||(option=="-MF")) {
			if((i+1)>=argc) continue;
			if(modelFileName=="") modelFileName = argv[i+1];

		} else if((option=="-ModelLibrary")||(option=="-ML")) {
			if((i+1)>=argc) continue;
			if(modelLibraryName=="") modelLibraryName = argv[i+1];

		} else if((option=="-Library")||(option=="-L")) {
			// DO NOTHING
			// -Library is a valid option.  Just not acting on it here.
		
		} else if((option=="-Setup")||(option=="-S")) {
			// DO NOTHING
			// -Setup is a valid option.  Just not acting on it here.
		
		} else {
			cout<<"WARN- "<<option<<" is an unknown option.\n";
		}
	}

	// CONSTRUCT ACTUATOR SET
	ActuatorSet *actuatorSet=0;
	if(actuators!="") {
		try {	// the constructor never returns NULL, either succeeds or throws an exception
			actuatorSet = new ActuatorSet(actuators.c_str());
		}
		catch (Exception &x)
		{
			actuatorSet=0;
		}

		if(actuatorSet==0) {
			cout<<"ERROR- actuator set "<<actuators<<" could not be constructed.\n\n";
		} else {
			cout<<"Constructed actuator set from file "<<actuators<<".\n";
		}
	}

	// CONSTRUCT CONTACT FORCE SET
	ContactForceSet *contactForceSet=NULL;
	if(contacts!="") {
		try {	// the constructor never returns NULL, either succeeds or throws an exception
			contactForceSet = new ContactForceSet(contacts.c_str());
		}
		catch (Exception &x)
		{
			contactForceSet=0;
		}
		if(contactForceSet==0) {
			cout<<"ERROR- contact force set "<<contacts<<" could not be constructed.\n\n";
		} else {
			cout<<"Constructed contact force set from file "<<contacts<<".\n";
		}
	}


	// LOAD MODEL LIBRARY
	PORTABLE_HINSTANCE modelLibrary = LoadOpenSimLibrary(modelLibraryName.c_str());
	if(modelLibrary==NULL) {
		cout<<"ERROR- library for model "<<modelLibraryName<<" could not be loaded.\n\n";
		return(NULL);
	}

	// CREATE MODEL
	Model *model = NULL;
	// Model file (deserialize)
	if(modelFileName!="") {

		// GET POINTER TO THE CreateModel() FUNCTION
		CREATEMODEL_FILE createModelFunction = (CREATEMODEL_FILE)GetProcAddress(modelLibrary,"CreateModel_File");
		if(createModelFunction==NULL) {
			cout<<"ERROR- function CreateModel_File() was not found in library "<<modelLibraryName<<".\n\n";
			return(NULL);
		}

		// CREATE THE MODEL
		model = createModelFunction(modelFileName);
		if(model==NULL) {
			cout<<"ERROR- model "<<modelLibraryName<<" was not created.\n\n";
			return(NULL);
		}

	// Params file and actuator and/or contact set
	} else if(params!="") {

		// GET POINTER TO THE CreateModel() FUNCTION
		CREATEMODEL_ParamsActuatorsContacts createModelFunction =
			(CREATEMODEL_ParamsActuatorsContacts)GetProcAddress(modelLibrary,
			"CreateModel_ParamsActuatorsContacts");
		if(createModelFunction==NULL) {
			cout<<"ERROR- function CreateModel_ParamsActuatorsContacts() was not found in library "<<modelLibraryName<<".\n\n";
			return(NULL);
		}

		// CREATE THE MODEL
		model = createModelFunction(params,actuatorSet,contactForceSet);
		if(model==NULL) {
			cout<<"ERROR- model "<<modelLibraryName<<" was not created.\n\n";
			return(NULL);
		}

	// Actuator and/or contact set
	} else if((actuatorSet!=NULL)||(contactForceSet!=NULL)) {

		// GET POINTER TO THE CreateModel() FUNCTION
		CREATEMODEL_ActuatorsContacts createModelFunction =
			(CREATEMODEL_ActuatorsContacts)GetProcAddress(modelLibrary,
			"CreateModel_ActuatorsContacts");
		if(createModelFunction==NULL) {
			cout<<"ERROR- function CreateModel_ActuatorsContacts() was not found in library "<<modelLibraryName<<".\n\n";
			return(NULL);
		}

		// CREATE THE MODEL
		model = createModelFunction(actuatorSet,contactForceSet);
		if(model==NULL) {
			cout<<"ERROR- model "<<modelLibraryName<<" was not created.\n\n";
			return(NULL);
		}

	// Default
	} else {

		// GET POINTER TO THE CreateModel() FUNCTION
		CREATEMODEL createModelFunction = (CREATEMODEL)GetProcAddress(modelLibrary,"CreateModel");
		if(createModelFunction==NULL) {
			cout<<"ERROR- function CreateModel() was not found in library "<<modelLibraryName<<".\n\n";
			return(NULL);
		}

		// CREATE THE MODEL
		model = createModelFunction();
		if(model==NULL) {
			cout<<"ERROR- model "<<modelLibraryName<<" was not created.\n\n";
			return(NULL);
		}
	}

	return(model);
}

//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(ostream &aOStream)
{
	aOStream<<"Options for LoadModel(int argc,char **argv):\n";
	aOStream<<"\t-Help or -H           Print the options for LoadModel(int argc,char **argv)\n";
	aOStream<<"\t-ModelLibrary or -ML  NameOfModelLibrary (do not include the library extension [e.g., .dll or .lib])\n";
	aOStream<<"\t-ModelFile or -MF     NameOfModelDeserializationFile (including path)\n";
	aOStream<<"\t-Actuators or -A      NameOfActuatorSet (including path)\n";
	aOStream<<"\t-Contacts or -C       NameofContactSet (including path)\n";
	aOStream<<"\t-Params or -P         NameOfPipelineParamsFile (including path)\n";
	aOStream<<"\t-Library or -L        NameOfAdditionalLibraryToLoad (to load more than one library,\n";
	aOStream<<"\t                      add as many -Library options as needed)\n";
}



