// SimmIKParams.cpp
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
#include "SimmIKParams.h"

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
SimmIKParams::SimmIKParams() :
   _modelFileName(_modelFileNameProp.getValueStr()),
	_markerSet((ArrayPtrs<SimmMarker>&)_markerSetProp.getValueObjArray()),
	_coordinateSet((ArrayPtrs<SimmCoordinate>&)_coordinateSetProp.getValueObjArray()),
	_IKTrialParamsSet((ArrayPtrs<SimmIKTrialParams>&)_IKTrialParamsSetProp.getValueObjArray())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmIKParams::SimmIKParams(DOMElement *aElement) :
   Object(aElement),
   _modelFileName(_modelFileNameProp.getValueStr()),
	_markerSet((ArrayPtrs<SimmMarker>&)_markerSetProp.getValueObjArray()),
	_coordinateSet((ArrayPtrs<SimmCoordinate>&)_coordinateSetProp.getValueObjArray()),
	_IKTrialParamsSet((ArrayPtrs<SimmIKTrialParams>&)_IKTrialParamsSetProp.getValueObjArray())
{
	setNull();
	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmIKParams::~SimmIKParams()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMarkerPlacementParams SimmIKParams to be copied.
 */
SimmIKParams::SimmIKParams(const SimmIKParams &aIKParams) :
   Object(aIKParams),
   _modelFileName(_modelFileNameProp.getValueStr()),
	_markerSet((ArrayPtrs<SimmMarker>&)_markerSetProp.getValueObjArray()),
	_coordinateSet((ArrayPtrs<SimmCoordinate>&)_coordinateSetProp.getValueObjArray()),
	_IKTrialParamsSet((ArrayPtrs<SimmIKTrialParams>&)_IKTrialParamsSetProp.getValueObjArray())
{
	setupProperties();
	copyData(aIKParams);
}
//_____________________________________________________________________________
/**
 * Copy this IK params and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmIKParams.
 */
Object* SimmIKParams::copy() const
{
	SimmIKParams *IKParams = new SimmIKParams(*this);
	return(IKParams);
}
//_____________________________________________________________________________
/**
 * Copy this SimmIKParams and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmIKParams::SimmIKParams(DOMElement*) in order to establish the
 * relationship of the SimmIKParams object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmIKParams object. Finally, the data members of the
 * copy are updated using SimmIKParams::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmIKParams.
 */
Object* SimmIKParams::copy(DOMElement *aElement) const
{
	SimmIKParams *IKParams = new SimmIKParams(aElement);
	*IKParams = *this;
	IKParams->updateFromXMLNode();
	return(IKParams);
}

void SimmIKParams::copyData(const SimmIKParams &aIKParams)
{
	_modelFileName = aIKParams._modelFileName;
	_markerSet = aIKParams._markerSet;
	_coordinateSet = aIKParams._coordinateSet;
	_IKTrialParamsSet = aIKParams._IKTrialParamsSet;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmIKParams to their null values.
 */
void SimmIKParams::setNull()
{
	setupProperties();
	setType("SimmIKParams");
	setName("");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmIKParams::setupProperties()
{
	_modelFileNameProp.setName("model_file");
	_modelFileNameProp.setComment("Name of model file. This is assumed to be scaled already with marker placement done.");
	_propertySet.append(&_modelFileNameProp);

	_markerSetProp.setName("MarkerSet");
	_markerSetProp.setComment("Markers to be used by all IK trials");
	ArrayPtrs<Object> ms;
	_markerSetProp.setValue(ms);
	_propertySet.append(&_markerSetProp);

	_coordinateSetProp.setName("CoordinateSet");
	_coordinateSetProp.setComment("Specify how to initialize coodinates for IK. Use value 'fromFile' to force IK to use a file to set the initial values. Filename is specified in the appropriate trial block.");
	ArrayPtrs<Object> cs;
	_coordinateSetProp.setValue(cs);
	_propertySet.append(&_coordinateSetProp);

	_IKTrialParamsSetProp.setName("IKTrialSet");
	_IKTrialParamsSetProp.setComment("Trial paramaters, one block per trial");
	ArrayPtrs<Object> iktps;
	_IKTrialParamsSetProp.setValue(iktps);
	_propertySet.append(&_IKTrialParamsSetProp);
}

void SimmIKParams::registerTypes()
{
	Object::RegisterType(SimmIKTrialParams());
}

SimmIKParams& SimmIKParams::operator=(const SimmIKParams &aIKParams)
{
	// BASE CLASS
	Object::operator=(aIKParams);

	copyData(aIKParams);

	return(*this);
}

void SimmIKParams::peteTest() const
{
	int i;

	cout << "   IKParameters: " << getName() << endl;
	cout << "      modelFileName: " << _modelFileName << endl;
	for (i = 0; i < _markerSet.getSize(); i++)
		_markerSet[i]->peteTest();
	for (i = 0; i < _coordinateSet.getSize(); i++)
		_coordinateSet[i]->peteTest();
	for (i = 0; i < _IKTrialParamsSet.getSize(); i++)
		_IKTrialParamsSet[i]->peteTest();
}
