// GenericModelMaker.cpp
// Author: Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include "GenericModelMaker.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Marker.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
GenericModelMaker::GenericModelMaker() :
   _fileName(_fileNameProp.getValueStr()),
	_markerSetFileName(_markerSetFileNameProp.getValueStr())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
GenericModelMaker::~GenericModelMaker()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGenericModelMaker GenericModelMaker to be copied.
 */
GenericModelMaker::GenericModelMaker(const GenericModelMaker &aGenericModelMaker) :
   Object(aGenericModelMaker),
   _fileName(_fileNameProp.getValueStr()),
	_markerSetFileName(_markerSetFileNameProp.getValueStr())
{
	setNull();
	setupProperties();
	copyData(aGenericModelMaker);
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one GenericModelMaker to another.
 *
 * @param aGenericModelMaker GenericModelMaker to be copied.
 */
void GenericModelMaker::copyData(const GenericModelMaker &aGenericModelMaker)
{
	_fileName = aGenericModelMaker._fileName;
	_markerSetFileName = aGenericModelMaker._markerSetFileName;
}

//_____________________________________________________________________________
/**
 * Set the data members of this GenericModelMaker to their null values.
 */
void GenericModelMaker::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void GenericModelMaker::setupProperties()
{
	_fileNameProp.setComment("Model file (.osim) for the unscaled model."); 
	_fileNameProp.setName("model_file");
	_propertySet.append(&_fileNameProp);

	_markerSetFileNameProp.setComment("Set of model markers used to scale the model. "
		"Scaling is done based on distances between model markers compared to "
		"the same distances between the corresponding experimental markers.");
	_markerSetFileNameProp.setName("marker_set_file");
	_propertySet.append(&_markerSetFileNameProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void GenericModelMaker::registerTypes()
{
	//Object::registerType(Marker());
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
GenericModelMaker& GenericModelMaker::operator=(const GenericModelMaker &aGenericModelMaker)
{
	// BASE CLASS
	Object::operator=(aGenericModelMaker);

	copyData(aGenericModelMaker);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * Execute the model making process, which involves reading
 * an XML model file and possible updating its marker set.
 *
 * @return Pointer to the Model that is constructed.
 */
Model* GenericModelMaker::processModel(const string& aPathToSubject)
{
	Model* model = NULL;

	cout << endl << "Step 1: Loading generic model" << endl;

	try
	{
		_fileName = aPathToSubject + _fileName;

		model = new Model(_fileName);
		model->initSystem();

		if (!_markerSetFileNameProp.getUseDefault()) {
			cout << "Loading marker set from '" << aPathToSubject+_markerSetFileName+"'" << endl;
			MarkerSet *markerSet = new MarkerSet(aPathToSubject + _markerSetFileName);
			model->updateMarkerSet(*markerSet);
		}
	}
	catch (Exception &x)
	{
		x.print(cout);
		return NULL;
	}

	return model;
}
