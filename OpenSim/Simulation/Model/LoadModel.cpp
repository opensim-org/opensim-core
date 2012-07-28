// LoadModel.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

#include <iostream>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include "LoadModel.h"
#include "Model.h"

using namespace OpenSim;
using namespace std;

extern "C" {

// Type definition for the CreateModel() function
// This is necessary to create a pointer that can properly point to the
// CreateModel() function.
typedef OpenSim::Model* (*CREATEMODEL)();
typedef OpenSim::Model* (*CREATEMODEL_FILE)(const string &);
typedef OpenSim::Model* (*CREATEMODEL_Forces)(OpenSim::ForceSet*);
typedef OpenSim::Model* (*CREATEMODEL_ParamsForces)(const string&,OpenSim::ForceSet*);

}

//_____________________________________________________________________________
/**
 * Load and create a model from a dynamically loaded library (DLL).
 *
 * @param aModelLibraryName Name of the model DLL.  Do not
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
