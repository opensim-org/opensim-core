// LoadModel.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include "LoadModel.h"
#include "Model.h"
#include "ActuatorSet.h"
#include "ContactForceSet.h"

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
OSIMSIMULATION_API Model* 
OpenSim::LoadModel(const string &aModelLibraryName, const string &aModelFileName)
{
	// LOAD MODEL LIBRARY
	OPENSIM_PORTABLE_HINSTANCE modelLibrary = LoadOpenSimLibrary(aModelLibraryName.c_str(), true);
	if(modelLibrary==NULL) {
		cout<<"ERROR- library for model "<<aModelLibraryName<<" could not be loaded.\n\n";
		return(NULL);
	}

	Model *model=NULL;
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
