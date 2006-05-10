// SimmGenericModelParams.cpp
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
#include "SimmGenericModelParams.h"

//=============================================================================
// STATICS
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimmGenericModelParams::SimmGenericModelParams() :
   _fileName(_fileNameProp.getValueStr()),
   _markerSet((ArrayPtrs<SimmMarker>&)_markerSetProp.getValueObjArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmGenericModelParams::SimmGenericModelParams(DOMElement *aElement) :
   Object(aElement),
   _fileName(_fileNameProp.getValueStr()),
   _markerSet((ArrayPtrs<SimmMarker>&)_markerSetProp.getValueObjArray())
{
	setNull();

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmGenericModelParams::~SimmGenericModelParams()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aGenericModelParams SimmGenericModelParams to be copied.
 */
SimmGenericModelParams::SimmGenericModelParams(const SimmGenericModelParams &aGenericModelParams) :
   Object(aGenericModelParams),
   _fileName(_fileNameProp.getValueStr()),
   _markerSet((ArrayPtrs<SimmMarker>&)_markerSetProp.getValueObjArray())
{
	setupProperties();
	copyData(aGenericModelParams);
}
//_____________________________________________________________________________
/**
 * Copy this SimmGenericModelParams and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmGenericModelParams.
 */
Object* SimmGenericModelParams::copy() const
{
	SimmGenericModelParams *genericModelParams = new SimmGenericModelParams(*this);
	return(genericModelParams);
}
//_____________________________________________________________________________
/**
 * Copy this SimmGenericModelParams and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmGenericModelParams::SimmGenericModelParams(DOMElement*) in order to establish the
 * relationship of the SimmGenericModelParams object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmGenericModelParams object. Finally, the data members of the copy are
 * updated using SimmGenericModelParams::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmGenericModelParams.
 */
Object* SimmGenericModelParams::copy(DOMElement *aElement) const
{
	SimmGenericModelParams *genericModelParams = new SimmGenericModelParams(aElement);
	*genericModelParams = *this;
	genericModelParams->updateFromXMLNode();
	return(genericModelParams);
}

void SimmGenericModelParams::copyData(const SimmGenericModelParams &aGenericModelParams)
{
	_fileName = aGenericModelParams._fileName;
	_markerSet = aGenericModelParams._markerSet;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmGenericModelParams to their null values.
 */
void SimmGenericModelParams::setNull()
{
	setupProperties();
	setType("SimmGenericModelParams");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmGenericModelParams::setupProperties()
{
	_fileNameProp.setName("file_name");
	_fileNameProp.setComment("Name of xml file for nominal model (unscaled generic model)"); 
	_propertySet.append(&_fileNameProp);

	_markerSetProp.setName("MarkerSet");
	_markerSetProp.setComment("Name of xml file specifying Markers used by the gaitlab.");
	ArrayPtrs<Object> markers;
	_markerSetProp.setValue(markers);
	_propertySet.append(&_markerSetProp);
}

void SimmGenericModelParams::registerTypes()
{
	Object::RegisterType(SimmMarker());
}

SimmGenericModelParams& SimmGenericModelParams::operator=(const SimmGenericModelParams &aGenericModelParams)
{
	// BASE CLASS
	Object::operator=(aGenericModelParams);

	copyData(aGenericModelParams);

	return(*this);
}

SimmModel* SimmGenericModelParams::processModel()
{
	SimmModel* model = NULL;

	cout << endl << "Step 1: Loading generic model" << endl;

	try
	{
		model = new SimmModel(_fileName);
		model->setup();

		if (model->builtOK())
			model->updateMarkers(_markerSet);
	}
	catch (Exception &x)
	{
		x.print(cout);
		cout << "Press Return to continue. " << endl;
		cout.flush();
		int c = getc( stdin );
		return NULL;
	}

	return model;
}

void SimmGenericModelParams::peteTest() const
{
	cout << "   GenericModel: " << getName() << endl;
	cout << "      fileName: " << _fileName << endl;
	cout << "      markers:" << endl;

	for (int i = 0; i < _markerSet.getSize(); i++)
		_markerSet[i]->peteTest();
}

