// SimmBody.cpp
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
#include "SimmBody.h"
#include "SimmKinematicsEngine.h"
#include "simmMacros.h"
#include <OpenSim/Tools/VisibleObject.h>


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
SimmBody::SimmBody() :
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj())
{
	setNull();
}
//_____________________________________________________________________________
/**
 * Constructor from an XML node
 */
SimmBody::SimmBody(DOMElement *aElement) :
   Object(aElement),
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj())
{
	setNull();

	updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
SimmBody::~SimmBody()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aBody SimmBody to be copied.
 */
SimmBody::SimmBody(const SimmBody &aBody) :
   Object(aBody),
   _mass(_massProp.getValueDbl()),
   _massCenter(_massCenterProp.getValueDblArray()),
   _inertia(_inertiaProp.getValueDblArray()),
	_displayerProp(PropertyObj("", VisibleObject())),
   _displayer((VisibleObject&)_displayerProp.getValueObj()),
	_markerSetProp(PropertyObj("", SimmMarkerSet())),
	_markerSet((SimmMarkerSet&)_markerSetProp.getValueObj())
{
	setupProperties();
	copyData(aBody);
}
//_____________________________________________________________________________
/**
 * Copy this body and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this SimmBody.
 */
Object* SimmBody::copy() const
{
	SimmBody *body = new SimmBody(*this);
	return(body);
}
//_____________________________________________________________________________
/**
 * Copy this SimmBody and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using
 * SimmBody::SimmBody(DOMElement*) in order to establish the
 * relationship of the SimmBody object with the XML node. Then, the
 * assignment operator is used to set all data members of the copy to the
 * values of this SimmBody object. Finally, the data members of the copy are
 * updated using SimmBody::updateFromXMLNode().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this SimmBody.
 */
Object* SimmBody::copy(DOMElement *aElement) const
{
	SimmBody *body = new SimmBody(aElement);
	*body = *this;
	body->updateFromXMLNode();
	return(body);
}

void SimmBody::copyData(const SimmBody &aBody)
{
	_mass = aBody._mass;
	_massCenter = aBody._massCenter;
	_inertia = aBody._inertia;
	_displayer = aBody._displayer; //? Do we need a dep copy here? when is this invoked?
	_markerSet = aBody._markerSet;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this SimmBody to their null values.
 */
void SimmBody::setNull()
{
	setupProperties();
	setType("SimmBody");
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void SimmBody::setupProperties()
{
	_massProp.setName("mass");
	_massProp.setValue(5.5);
	_propertySet.append(&_massProp);

	const double defaultMC[] = {1.1, 2.2, 3.3};
	_massCenterProp.setName("mass_center");
	_massCenterProp.setValue(3, defaultMC);
	_propertySet.append(&_massCenterProp);

	const double defaultInertia[] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
	_inertiaProp.setName("inertia");
	_inertiaProp.setValue(9, defaultInertia);
	_propertySet.append(&_inertiaProp);

	_displayerProp.setName("Displayer");
	_propertySet.append(&_displayerProp);

	_markerSetProp.setName("SimmMarkerSet");
	_propertySet.append(&_markerSetProp);
}

SimmBody& SimmBody::operator=(const SimmBody &aBody)
{
	// BASE CLASS
	Object::operator=(aBody);

	copyData(aBody);

	return(*this);
}

/* Perform some set up functions that happen after the
 * object has been deserialized or copied.
 */
void SimmBody::setup(SimmKinematicsEngine* aEngine)
{
	//SimmBone* sb;

	/* The _bones array can contain any VisibleObject.
	 * Check for ones of type SimmBone and call the
	 * setup function for them (to read in the VTK
	 * files, etc.).
	 *
	int i;
	for (i = 0; i < _bones.getSize(); i++)
	{
		if (sb = dynamic_cast<SimmBone*>(_bones[i]))
			sb->setup(aEngine);
	}
	*/
	int i;
	for (i = 0; i < _displayer.getNumGeometryFiles(); i++)
	{
		_displayer.addGeometry(new PolyhedralGeometry("bones/"+_displayer.getGeometryFileName(i)));
	}

	for (i = 0; i < _markerSet.getSize(); i++)
	{
		_markerSet.get(i)->setup(aEngine);
		// This should happen inside setup
		_displayer.addDependent(_markerSet.get(i)->getDisplayer());
		Transform position;
		position.translate(_markerSet.get(i)->getOffset());
		_markerSet.get(i)->getDisplayer()->setTransform(position);

	}

	_displayer.setOwner(this);
	/*
	for (i = 0; i < 3; i++)
		_scaleFactor[i] = 1.0;
		*/
}

void SimmBody::addMarker(SimmMarker* aMarker)
{
	// newMarker will be deleted when the _markers array
	// is deleted because _memoryOwner is set to true.
	SimmMarker* newMarker = new SimmMarker(*aMarker);
	_markerSet.append(newMarker);
}

SimmMarker* SimmBody::getMarker(int index) const
{
	if (index >= 0 && index < _markerSet.getSize())
		return _markerSet.get(index);

	return NULL;
}

int SimmBody::deleteAllMarkers()
{
	int numDeleted = _markerSet.getSize();

	_markerSet.setSize(0);

	return numDeleted;
}

void SimmBody::deleteMarker(const SimmMarker* aMarker)
{
	_markerSet.remove(aMarker);
}

/* Remove all markers from the body that are not in the passed-in list. */
int SimmBody::deleteUnusedMarkers(const Array<string>& aMarkerNames)
{
	int j=0;
	int numDeleted = 0;

	for (int i = 0; i < _markerSet.getSize(); i++)
	{
		for (j = 0; j < aMarkerNames.getSize(); j++)
		{
			if (aMarkerNames[j] == _markerSet.get(i)->getName())
				break;
		}
		if (j == aMarkerNames.getSize())
		{
			_markerSet.remove(i);
			numDeleted++;
			i--; // decrement i so the next marker will not be skipped
		}
	}

	return numDeleted;
}

void SimmBody::scale(Array<double>& aScaleFactors, bool aPreserveMassDist)
{
	int i;

	double oldScaleFactors[3];
	getDisplayer()->getScaleFactors(oldScaleFactors);

	for (i = 0; i < 3; i++)
	{
		_massCenter[i] *= aScaleFactors[i];
		oldScaleFactors[i] *= aScaleFactors[i];
	}
	// Update scale factors for displayer
	getDisplayer()->setScaleFactors(aScaleFactors.get());

	if (!aPreserveMassDist)
		scaleInertialProperties(aScaleFactors);

	/*
	SimmBone* sb;
	for (i = 0; i < _bones.getSize(); i++)
	{
		if (sb = dynamic_cast<SimmBone*>(_bones[i]))
			sb->scale(aScaleFactors);
	}
	*/
	for (i = 0; i < _markerSet.getSize(); i++)
		_markerSet.get(i)->scale(aScaleFactors);
}

void SimmBody::scaleInertialProperties(Array<double>& aScaleFactors)
{
	int i;

	/* If the mass is zero, then make the inertia tensor zero as well.
	 * If the X, Y, Z scale factors are equal, then you can scale the
	 * inertia tensor exactly by the square of the scale factor, since
	 * each element in the tensor is proportional to the square of one
	 * or more dimensional measurements. For determining if the scale
	 * factors are equal, ignore reflections-- look only at the
	 * absolute value of the factors.
	 */
	if (_mass <= ROUNDOFF_ERROR)
	{
		for (i = 0; i < 9; i++)
			_inertia[i] = 0.0;
	}
	else if (EQUAL_WITHIN_ERROR(DABS(aScaleFactors[0]), DABS(aScaleFactors[1])) &&
		      EQUAL_WITHIN_ERROR(DABS(aScaleFactors[1]), DABS(aScaleFactors[2])))
	{
		_mass *= DABS((aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]));

		for (i = 0; i < 9; i++)
			_inertia[i] *= (aScaleFactors[0] * aScaleFactors[0]);
	}
	else
	{
		/* If the scale factors are not equal, then assume that the segment
		 * is a cylinder and the inertia is calculated about one end of it.
		 */
		int axis;

		/* 1. Find the smallest diagonal component. This dimension is the axis
		 *    of the cylinder.
		 */
		if (_inertia[0] <= _inertia[4])
		{
			if (_inertia[0] <= _inertia[8])
				axis = 0;
			else
				axis = 2;
		}
		else if (_inertia[4] <= _inertia[8])
		{
			axis = 1;
		}
		else
		{
			axis = 2;
		}

		/* 2. The smallest inertial component is equal to 0.5 * mass * radius * radius,
		 *    so you can rearrange and solve for the radius.
		 */
		int oa;
		double radius, rad_sqr, length;
		double term = 2.0 * _inertia[axis * 3 + axis] / _mass;
		if (term < 0.0)
			radius = 0.0;
		else
			radius = sqrt(term);

		/* 3. Choose either of the other diagonal components and use it to solve for the
		*    length of the cylinder. This component is equal to:
		*    0.333 * mass * length * length  +  0.25 * mass * radius * radius
		*/
		if (axis == 0)
			oa = 1;
		else
			oa = 0;
		term = 3.0 * (_inertia[oa * 3 + oa] - 0.25 * _mass * radius * radius) / _mass;
		if (term < 0.0)
			length = 0.0;
		else
			length = sqrt(term);

		/* 4. Scale the mass, radius, and length, and recalculate the diagonal inertial terms. */
		_mass *= DABS((aScaleFactors[0] * aScaleFactors[1] * aScaleFactors[2]));
		length *= DABS(aScaleFactors[axis]);

		if (axis == 0)
		{
			rad_sqr = radius * DABS(aScaleFactors[1]) * radius * DABS(aScaleFactors[2]);
			_inertia[0] = 0.5 * _mass * rad_sqr;
			_inertia[4] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[8] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
		}
		else if (axis == 1)
		{
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[2]);
			_inertia[0] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[4] = 0.5 * _mass * rad_sqr;
			_inertia[8] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
		}
		else
		{
			rad_sqr = radius * DABS(aScaleFactors[0]) * radius * DABS(aScaleFactors[1]);
			_inertia[0] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[4] = _mass * ((length * length / 3.0) + 0.25 * rad_sqr);
			_inertia[8] = 0.5 * _mass * rad_sqr;
		}

		/* 5. Scale the cross terms, in case some are non-zero. */
		_inertia[1] *= DABS((aScaleFactors[0] * aScaleFactors[1]));
		_inertia[2] *= DABS((aScaleFactors[0] * aScaleFactors[2]));
		_inertia[3] *= DABS((aScaleFactors[1] * aScaleFactors[0]));
		_inertia[5] *= DABS((aScaleFactors[1] * aScaleFactors[2]));
		_inertia[6] *= DABS((aScaleFactors[2] * aScaleFactors[0]));
		_inertia[7] *= DABS((aScaleFactors[2] * aScaleFactors[1]));
	}
}

void SimmBody::writeSIMM(ofstream& out) const
{
	int i;
	//SimmBone* sb;

	out << "beginsegment " << getName() << endl;
	out << "mass " << _mass << endl;
	out << "masscenter " << _massCenter[0] << " " << _massCenter[1] << " " << _massCenter[2] << endl;
	out << "inertia " << _inertia[0] << " " << _inertia[1] << " " << _inertia[2] << endl;
	out << "        " << _inertia[3] << " " << _inertia[4] << " " << _inertia[5] << endl;
	out << "        " << _inertia[6] << " " << _inertia[7] << " " << _inertia[8] << endl;
	
	string fileName;
	for (i = 0; i < _displayer.getNumGeometryFiles()  ; i++)
	{
		fileName = _displayer.getGeometryFileName(i);
		int dot = fileName.find_last_of(".");
		if (dot > 0)
			fileName.erase(dot, 4);
		fileName += ".asc";
		out << "bone " << fileName << endl;
	}
	
	for (i = 0; i < _markerSet.getSize(); i++)
		_markerSet.get(i)->writeSIMM(out);

	double scaleFactors[3];
	_displayer.getScaleFactors(scaleFactors);

	out << "scale " << scaleFactors[0] << " " << scaleFactors[1] << " " << scaleFactors[2] << endl;
	out << "endsegment" << endl << endl;
}

void SimmBody::writeMarkers(ofstream& out) const
{
	/* The code to write a marker to a file could be
	 * turned into a SimmMarker method, but what gets
	 * written to the file is not a standard marker
	 * node, but rather one that can be pasted into
	 * a MarkerSet node in a SimmSubject definition.
	 * The bodyName field is needed for this, and
	 * bodyName is not used by every marker in the model.
	 */
	for (int i = 0; i < _markerSet.getSize(); i++)
	{
		const double* pos = _markerSet.get(i)->getOffset();

		out << "   <SimmMarker name=\"" << _markerSet.get(i)->getName() << "\">" << endl;
		out << "      <body>" << getName() << "</body>" << endl;
		out << "      <location>" << pos[0] << " " << pos[1] << " " << pos[2] << "</location>" << endl;
		out << "      <weight>" << _markerSet.get(i)->getWeight() << "</weight>" << endl;
		out << "      <fixed>" << ((_markerSet.get(i)->getFixed()) ? ("true") : ("false")) << "</fixed>" << endl;
		out << "   </SimmMarker>" << endl;
	}
}

void SimmBody::peteTest() const
{
	int i;
	//SimmBone* sb;

	cout << "Body: " << getName() << endl;
	cout << "   mass: " << _mass << endl;
	cout << "   massCenter: " << _massCenter << endl;
	cout << "   inertia: " << _inertia << endl;
	/*
	for (i = 0; i < _bones.getSize(); i++)
	{
		if (sb = dynamic_cast<SimmBone*>(_bones[i]))
			sb->peteTest();
	}
	*/
	for (i = 0; i < _markerSet.getSize(); i++)
		_markerSet.get(i)->peteTest();
}

void SimmBody::getScaleFactors(Array<double>& scales) const
{

	double scaleFactors[3];
	_displayer.getScaleFactors(scaleFactors);

	for (int i=0; i<3; i++)
		scales[i] = scaleFactors[i];

}
