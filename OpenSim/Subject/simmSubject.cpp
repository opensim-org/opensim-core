// SimmSubject.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

//=============================================================================
// INCLUDES
//=============================================================================
#include "SimmSubject.h"
#include <OpenSim/Simulation/SIMM/simmIO.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>

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
SimmSubject::SimmSubject() :
	_mass(_massProp.getValueDbl()),
	_height(_heightProp.getValueDbl()),
	_age(_ageProp.getValueDbl()),
	_notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", SimmGenericModelMaker())),
	_genericModelMaker((SimmGenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", SimmModelScaler())),
	_modelScaler((SimmModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", SimmMarkerPlacer())),
	_markerPlacer((SimmMarkerPlacer&)_markerPlacerProp.getValueObj())
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor from an XML file
 */
SimmSubject::SimmSubject(const string &aFileName) :
   Object(aFileName),
	_mass(_massProp.getValueDbl()),
	_height(_heightProp.getValueDbl()),
	_age(_ageProp.getValueDbl()),
	_notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", SimmGenericModelMaker())),
	_genericModelMaker((SimmGenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", SimmModelScaler())),
	_modelScaler((SimmModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", SimmMarkerPlacer())),
	_markerPlacer((SimmMarkerPlacer&)_markerPlacerProp.getValueObj())
{
	setNull();
	setupProperties();

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd(0, 256);
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile);

	updateFromXMLNode();

	IO::chDir(saveWorkingDirectory);
	_pathToSubject = IO::getParentDirectory(aFileName);
}

//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmSubject::SimmSubject(DOMElement *aElement) :
   Object(aElement),
	_mass(_massProp.getValueDbl()),
	_height(_heightProp.getValueDbl()),
	_age(_ageProp.getValueDbl()),
	_notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", SimmGenericModelMaker())),
	_genericModelMaker((SimmGenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", SimmModelScaler())),
	_modelScaler((SimmModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", SimmMarkerPlacer())),
	_markerPlacer((SimmMarkerPlacer&)_markerPlacerProp.getValueObj())
{
	setNull();
	setupProperties();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmSubject::~SimmSubject()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aSubject SimmSubject to be copied.
 */
SimmSubject::SimmSubject(const SimmSubject &aSubject) :
   Object(aSubject),
	_mass(_massProp.getValueDbl()),
	_height(_heightProp.getValueDbl()),
	_age(_ageProp.getValueDbl()),
	_notes(_notesProp.getValueStr()),
   _genericModelMakerProp(PropertyObj("", SimmGenericModelMaker())),
	_genericModelMaker((SimmGenericModelMaker&)_genericModelMakerProp.getValueObj()),
   _modelScalerProp(PropertyObj("", SimmModelScaler())),
	_modelScaler((SimmModelScaler&)_modelScalerProp.getValueObj()),
   _markerPlacerProp(PropertyObj("", SimmMarkerPlacer())),
	_markerPlacer((SimmMarkerPlacer&)_markerPlacerProp.getValueObj())
{
	setNull();
	setupProperties();
	copyData(aSubject);
}

//_____________________________________________________________________________
/**
 * Copy this subject and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmSubject.
 */
Object* SimmSubject::copy() const
{
	SimmSubject *subject = new SimmSubject(*this);
	return(subject);
}

//_____________________________________________________________________________
/**
 * Copy this SimmSubject and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmSubject::SimmSubject(DOMElement*) in order to establish the
 * relationship of the SimmSubject object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmSubject object. Finally, the data members of the
 * copy are updated using SimmSubject::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmSubject.
 */
Object* SimmSubject::copy(DOMElement *aElement) const
{
	SimmSubject *subject = new SimmSubject(aElement);
	*subject = *this;
	subject->updateFromXMLNode();
	return(subject);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one SimmSubject to another.
 *
 * @param aSubject SimmSubject to be copied.
 */
void SimmSubject::copyData(const SimmSubject &aSubject)
{
	_mass = aSubject._mass;
	_height = aSubject._height;
	_age = aSubject._age;
	_notes = aSubject._notes;
	_genericModelMaker = aSubject._genericModelMaker;
	_modelScaler = aSubject._modelScaler;
	_markerPlacer = aSubject._markerPlacer;
}

//_____________________________________________________________________________
/**
 * Set the data members of this SimmSubject to their null values.
 */
void SimmSubject::setNull()
{
	setType("SimmSubject");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmSubject::setupProperties()
{
	_massProp.setName("mass");
	_massProp.setValue(-1.0);
	_propertySet.append(&_massProp);

	_heightProp.setName("height");
	_heightProp.setValue(-1.0);
	_propertySet.append(&_heightProp);

	_ageProp.setName("age");
	_ageProp.setValue(-1.0);
	_propertySet.append(&_ageProp);

	_notesProp.setName("notes");
	_propertySet.append(&_notesProp);

	_genericModelMakerProp.setName("");
	_genericModelMakerProp.setComment("File name for the nominal model.xml to use is specified here");
	_propertySet.append(&_genericModelMakerProp);

	_modelScalerProp.setName("");
	_modelScalerProp.setComment("Parameters to control nominal model scaling go here.");
	_propertySet.append(&_modelScalerProp);

	_markerPlacerProp.setName("");
	_markerPlacerProp.setComment("Marker set to override default markers is specified here");
	_propertySet.append(&_markerPlacerProp);
}

//_____________________________________________________________________________
/**
 * Register the types used by this class.
 */
void SimmSubject::registerTypes()
{
	Object::RegisterType(SimmGenericModelMaker());
	Object::RegisterType(SimmModelScaler());
	Object::RegisterType(SimmMarkerPlacer());
	SimmGenericModelMaker::registerTypes();
	SimmModelScaler::registerTypes();
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
SimmSubject& SimmSubject::operator=(const SimmSubject &aSubject)
{
	// BASE CLASS
	Object::operator=(aSubject);

	copyData(aSubject);

	return(*this);
}

//=============================================================================
// UTILITY
//=============================================================================
//_____________________________________________________________________________
/**
 * This method creates a subject-specific model from a generic model plus
 * marker cloud and coordinate data files, and then processes one or more
 * IK trials using the model. It uses:
 * SimmGenericModelMaker::processModel() to load the generic model,
 * SimmModelScaler::processModel() to scale the model to the subject, and
 * SimmMarkerPlacer::processModel() to place markers on the model.
 *
 * @return Whether or not there was a fatal error in any of the steps.
 */
bool SimmSubject::processModel()
{
	AbstractModel *model = NULL;

	cout << "Processing subject " << getName() << endl;

	/* Make the generic model. */
	if (!_genericModelMakerProp.getUseDefault())
	{
		model = _genericModelMaker.processModel(_pathToSubject);
		if (!model)
		{
			cout << "===ERROR===: Unable to load generic model." << endl;
			return false;
		}
	}

	/* Scale the generic model. */
	if (!_modelScalerProp.getUseDefault())
	{
		if (_genericModelMakerProp.getUseDefault())
		{
			cout << "===ERROR===: To use ModelScaler, you must specify GenericModelMaker." << endl;
			return false;
		}
		else
		{
			/* If a generic model was specified, but the model
			 * was not created successfully, you won't make it
			 * to this point. So here you know that model is
			 * a proper model.
			 */
			if (!_modelScaler.processModel(model, _pathToSubject))
			{
				cout << "===ERROR===: Unable to scale generic model." << endl;
				return false;
			}
		}
	}

	/* Place the markers on the scaled model. */
	if (!_markerPlacerProp.getUseDefault())
	{
		if (_genericModelMakerProp.getUseDefault())
		{
			cout << "===ERROR===: To use MarkerPlacer, you must specify GenericModelParameters." << endl;
			return false;
		}
		else
		{
			/* If a generic model was specified, but the model
			 * was not created successfully, you won't make it
			 * to this point. So here you know that model is
			 * a proper model.
			 */
			if (!_markerPlacer.processModel(model, _pathToSubject))
			{
				cout << "===ERROR===: Unable to place markers on model." << endl;
				return false;
			}
		}
	}

	/* Clean up. */
	delete model;

	return true;
}

//_____________________________________________________________________________
/**
 * Create a generic model, using SimmGenericModelMaker::processModel().
 *
 * @return Pointer to the AbstractModel that is created.
 */
AbstractModel* SimmSubject::createModel()
{
	cout << "Processing subject " << getName() << endl;

	/* Make the generic model. */
	if (!_genericModelMakerProp.getUseDefault())
	{
		AbstractModel *model = _genericModelMaker.processModel(_pathToSubject);
		if (model==0)
		{
			cout << "===ERROR===: Unable to load generic model." << endl;
			return 0;
		}
		else
			return model;
	}
	return 0;
}

/*
string SimmSubject::getParentDirectory(const string& fileName)
{
	string	result="";

	string::size_type dirSep = fileName.rfind('/'); // Unix/Mac dir separator
	
	if (dirSep == string::npos)
		dirSep = fileName.rfind('\\'); // DOS dir separator
	
	if (dirSep != string::npos) // if '_fileName' contains path information...
	{
		string dirPath(fileName, 0, dirSep+1);	// include trailing slashes
		result=dirPath;
	}

	return result;	// Eventually this will be moved to simmIO as a platform specific thing
}
*/

void SimmSubject::peteTest() const
{
	cout << "   Subject: " << getName() << endl;
	_genericModelMaker.peteTest();
	_modelScaler.peteTest();
	_markerPlacer.peteTest();
}
