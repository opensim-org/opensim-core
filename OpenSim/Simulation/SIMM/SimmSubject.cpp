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
#include "simmIO.h"
#include <OpenSim/Tools/IO.h>

using namespace OpenSim;
const double SimmSubject::DefaultMass=-1.0;
//=============================================================================
// STATICS
//=============================================================================
using namespace std;

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
   _genericModelParamsProp(PropertyObj("", SimmGenericModelParams())),
	_genericModelParams((SimmGenericModelParams&)_genericModelParamsProp.getValueObj()),
   _scalingParamsProp(PropertyObj("", SimmScalingParams())),
	_scalingParams((SimmScalingParams&)_scalingParamsProp.getValueObj()),
   _markerPlacementParamsProp(PropertyObj("", SimmMarkerPlacementParams())),
	_markerPlacementParams((SimmMarkerPlacementParams&)_markerPlacementParamsProp.getValueObj()),
   _IKParamsProp(PropertyObj("", SimmIKParams())),
	_IKParams((SimmIKParams&)_IKParamsProp.getValueObj())
{
	setNull();
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
   _genericModelParamsProp(PropertyObj("", SimmGenericModelParams())),
	_genericModelParams((SimmGenericModelParams&)_genericModelParamsProp.getValueObj()),
   _scalingParamsProp(PropertyObj("", SimmScalingParams())),
	_scalingParams((SimmScalingParams&)_scalingParamsProp.getValueObj()),
   _markerPlacementParamsProp(PropertyObj("", SimmMarkerPlacementParams())),
	_markerPlacementParams((SimmMarkerPlacementParams&)_markerPlacementParamsProp.getValueObj()),
   _IKParamsProp(PropertyObj("", SimmIKParams())),
	_IKParams((SimmIKParams&)_IKParamsProp.getValueObj())
{
	setNull();

	// Do the maneuver to change then restore working directory 
	// so that the parsing code behaves properly if called from a different directory.
	string saveWorkingDirectory = IO::getCwd(0, 256);
	string directoryOfSetupFile = IO::getParentDirectory(aFileName);
	IO::chDir(directoryOfSetupFile.c_str());

	updateFromXMLNode();

	IO::chDir(saveWorkingDirectory.c_str());
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
   _genericModelParamsProp(PropertyObj("", SimmGenericModelParams())),
	_genericModelParams((SimmGenericModelParams&)_genericModelParamsProp.getValueObj()),
   _scalingParamsProp(PropertyObj("", SimmScalingParams())),
	_scalingParams((SimmScalingParams&)_scalingParamsProp.getValueObj()),
   _markerPlacementParamsProp(PropertyObj("", SimmMarkerPlacementParams())),
	_markerPlacementParams((SimmMarkerPlacementParams&)_markerPlacementParamsProp.getValueObj()),
   _IKParamsProp(PropertyObj("", SimmIKParams())),
	_IKParams((SimmIKParams&)_IKParamsProp.getValueObj())
{
	setNull();

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
   _genericModelParamsProp(PropertyObj("", SimmGenericModelParams())),
	_genericModelParams((SimmGenericModelParams&)_genericModelParamsProp.getValueObj()),
   _scalingParamsProp(PropertyObj("", SimmScalingParams())),
	_scalingParams((SimmScalingParams&)_scalingParamsProp.getValueObj()),
   _markerPlacementParamsProp(PropertyObj("", SimmMarkerPlacementParams())),
	_markerPlacementParams((SimmMarkerPlacementParams&)_markerPlacementParamsProp.getValueObj()),
   _IKParamsProp(PropertyObj("", SimmIKParams())),
	_IKParams((SimmIKParams&)_IKParamsProp.getValueObj())
{
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

void SimmSubject::copyData(const SimmSubject &aSubject)
{
	_mass = aSubject._mass;
	_height = aSubject._height;
	_age = aSubject._age;
	_notes = aSubject._notes;
	_genericModelParams = aSubject._genericModelParams;
	_scalingParams = aSubject._scalingParams;
	_markerPlacementParams = aSubject._markerPlacementParams;
	_IKParams = aSubject._IKParams;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmSubject to their null values.
 */
void SimmSubject::setNull()
{
	setupProperties();
	setType("SimmSubject");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmSubject::setupProperties()
{
	_massProp.setName("mass");
	_massProp.setValue(SimmSubject::DefaultMass);
	_massProp.setComment("Mass of the subject in kgs.");
	_propertySet.append(&_massProp);

	_heightProp.setName("height");
	_heightProp.setValue(-1.0);
	_heightProp.setComment("Height of the subject in cms.");
	_propertySet.append(&_heightProp);

	_ageProp.setName("age");
	_ageProp.setValue(-1.0);
	_propertySet.append(&_ageProp);

	_notesProp.setName("notes");
	_notesProp.setComment("Save record-keeping info. here.");
	_propertySet.append(&_notesProp);

	_genericModelParamsProp.setName("");
	_genericModelParamsProp.setComment("File name for the (nominal model).sim to use is specified here.");
	_propertySet.append(&_genericModelParamsProp);

	_scalingParamsProp.setName("");
	_scalingParamsProp.setComment("Parameters to control nominal model scaling go here.");
	_propertySet.append(&_scalingParamsProp);

	_markerPlacementParamsProp.setName("");
	_markerPlacementParamsProp.setComment("Parameters needed to move model's marker locations to match recorded ones from a static trial.");
	_propertySet.append(&_markerPlacementParamsProp);

	_IKParamsProp.setName("");
	_IKParamsProp.setComment("Parameters pertinent to solving IK are specified here.");
	_propertySet.append(&_IKParamsProp);
}

void SimmSubject::registerTypes()
{
	Object::RegisterType(SimmGenericModelParams());
	Object::RegisterType(SimmScalingParams());
	Object::RegisterType(SimmMarkerPlacementParams());
	Object::RegisterType(SimmIKParams());
	SimmGenericModelParams::registerTypes();
	SimmScalingParams::registerTypes();
	SimmIKParams::registerTypes();
}

SimmSubject& SimmSubject::operator=(const SimmSubject &aSubject)
{
	// BASE CLASS
	Object::operator=(aSubject);

	copyData(aSubject);

	return(*this);
}

SimmModel* SimmSubject::createModel()
{
	cout << "Processing subject " << getName() << endl;

	/* Make the generic model. */
	if (!_genericModelParamsProp.getUseDefault())
	{
		
		SimmModel *model = _genericModelParams.processModel(_pathToSubject.c_str());
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

void SimmSubject::peteTest() const
{
	cout << "   Subject: " << getName() << endl;
	_genericModelParams.peteTest();
	_scalingParams.peteTest();
	_markerPlacementParams.peteTest();
	_IKParams.peteTest();
}

/**
 * Check if MarkerPlacementParams settings in file are different from defaults
 */
bool SimmSubject::isDefaultMarkerPlacementParams() const
{ 
	
	return (_markerPlacementParamsProp.getUseDefault()==true || _markerPlacementParams.isDefault()); 
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
