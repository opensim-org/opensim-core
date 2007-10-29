// VisibleObject.cpp
// Author: Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author:  
 */


//============================================================================
// INCLUDES
//============================================================================
#include "VisibleObject.h"
#include "Exception.h"
#include "PropertyInt.h"
#include "PropertyStr.h"
#include "PropertyObj.h"
#include "PropertyStrArray.h"
#include "PropertyDblArray.h"
#include "Geometry.h"


using namespace OpenSim;
using namespace std;


//============================================================================
// CONSTANTS
//============================================================================


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
VisibleObject::~VisibleObject()
{
	freeGeometry();
	_owner = NULL;
	_dependents.setSize(0);
}

//_____________________________________________________________________________
/**
 * Default constructor.
 */
VisibleObject::VisibleObject():
_geometryFileNames(_propGeometryFileNames.getValueStrArray()),
_allGeometry(0),
_propVisibleProp(PropertyObj("", VisibleProperties())),
_visibleProp((VisibleProperties&)_propVisibleProp.getValueObj()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
_dependents(0)
{
	// NULL STATES
	setNull();

	// MEMBER VARIABLES
	setType("VisibleObject");
	setName("");
}

//_____________________________________________________________________________
/**
 * Construct an object from file.
 *
 * The object is constructed from the root element of the XML document.
 * The type of object is the tag name of the XML root element.
 *
 * @param aFileName File name of the document.
 */
VisibleObject::VisibleObject(const string &aFileName):
Object(aFileName, false),
_geometryFileNames(_propGeometryFileNames.getValueStrArray()),
_allGeometry(0),
_propVisibleProp(PropertyObj("", VisibleProperties())),
_visibleProp((VisibleProperties&)_propVisibleProp.getValueObj()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
_dependents(0)
{
	// NULL STATES
	setNull();

	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all VisibleObject's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an VisibleObject:
 *
 * 1) Construction based on XML file (@see VisibleObject(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by VisibleObject(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * VisibleObject member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated VisibleObject member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the VisibleObject from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aObject Object to be copied.
 * @see VisibleObject(const XMLDocument *aDocument)
 * @see VisibleObject(const char *aFileName)
 * @see generateXMLDocument()
 */
VisibleObject::VisibleObject(const VisibleObject &aObject):
Object(aObject),
_geometryFileNames(_propGeometryFileNames.getValueStrArray()),
_allGeometry(0),
_propVisibleProp(PropertyObj("", VisibleProperties())),
_visibleProp((VisibleProperties&)_propVisibleProp.getValueObj()),
_scaleFactors(_propScaleFactors.getValueDblArray()),
_dependents(0)
{
	// NULL MEMBER VARIABLES
	setNull();

	// COPY TYPE AND NAME
	*this = aObject;
}


//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void VisibleObject::
setNull()
{
	setupProperties();

	Array<double> unit3(1.0, 3);
	_scaleFactors = unit3;

	_owner = 0;
	_dependents.setMemoryOwner(false);

}
/**
 * virtual copy constructor
 */
Object* VisibleObject::
copy() const
{

	VisibleObject *object = new VisibleObject(*this);
	return(object);
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void VisibleObject::setupProperties()
{

	_propGeometryFileNames.setName("geometry_files");
	Array<string> filenames("");
	_propGeometryFileNames.setValue(filenames);
	_propertySet.append(&_propGeometryFileNames);

	_propVisibleProp.setName("visible_properties");
	_propertySet.append(&_propVisibleProp);

	_propScaleFactors.setName("scale_factors");
	_propScaleFactors.setAllowableArraySize(3);
	_propertySet.append(&_propScaleFactors);
}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.
 *
 * @return Reference to this object.
 */
VisibleObject& VisibleObject::
operator=(const VisibleObject &aObject)
{
	// BASE CLASS
	Object::operator=(aObject);

	setNumGeometryFiles(aObject.getNumGeometryFiles());
	for(int i=0; i < aObject.getNumGeometryFiles(); i++)
		setGeometryFileName(i, aObject.getGeometryFileName(i));

	_transform = aObject._transform;
	_visibleProp = aObject._visibleProp;

	_scaleFactors = aObject._scaleFactors;
	return(*this);
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if two objects are equal.
 *
 * @return True if the two objects are equal, false otherwise.
 */
bool VisibleObject::
operator==(const VisibleObject &aObject)
{
	return(Object::operator==(aObject));
}


//=============================================================================
// GET AND SET
//=============================================================================
//------------- Geometry files mgmt. -------------------------------------------
/**
 * set the number of geometry files associated with visible object
 */
void VisibleObject::
setNumGeometryFiles(int n)
{
	_geometryFileNames.setSize(n);
}
/**
 * Get the number of geometry files associated with visible object
 */
const int VisibleObject::
getNumGeometryFiles() const
{
	return (_geometryFileNames.getSize());
}
/**
 * set the name of ith geometry files associated with visible object
 */
void VisibleObject::
setGeometryFileName(int i, const string &aGeometryFileName)
{
	_geometryFileNames.set(i,aGeometryFileName);
}
/**
 * Get the name of ith geometry files associated with visible object
 */

const std::string& VisibleObject::
getGeometryFileName(int i) const
{
	return _geometryFileNames[i];
}

/**
 * set the visible properties of a visible object
 */
//------------- Properties: Shading, Color, etc. -------------------------------
void VisibleObject::
setVisibleProperties(const VisibleProperties &aVisibleProperties)
{
	_visibleProp = aVisibleProperties;
}
/**
 * Retrieve the visible properties of a visible object
 */
VisibleProperties& VisibleObject::
getVisibleProperties()
{
	return (_visibleProp);
}
//_____________________________________________________________________________
/**
 * Set Scale factors for geometry.
 *
 */
void VisibleObject::
setScaleFactors(const double aScaleFactors[3])
{
	_propScaleFactors.setUseDefault(false);
	for(int i=0; i < 3; i++)
		_scaleFactors[i]=(aScaleFactors[i]);
}
//_____________________________________________________________________________
/**
 * Get Scale factors for geometry.
 *
 */
void VisibleObject::
getScaleFactors(double aScaleFactors[3]) const
{
	for(int i=0; i < 3; i++)
		aScaleFactors[i] = _scaleFactors[i];
}
//------------- Transform -----------------------------------------------------
/**
 * Set the transform of a visible object
 */
void VisibleObject::
setTransform(const Transform &aTransform)
{
	_transform = aTransform;
	setChanged();
}
/**
 * Retrieve the transform of a visible object
 */

Transform& VisibleObject::
getTransform() 
{
	return _transform;
}
//------------- Transform -----------------------------------------------------
// More interfaces for transform as other interfaces (Angles based) are needed
// Rotations in Radians
// three rotation angles, assumed order XYZ
/**
 * Rotate a visible object by 3 angles in radians around XYZ in order
 */
void VisibleObject::
rotateRadians(const double rR[3])
{
	getTransform().rotate(rR, Transform::Radians, Transform::XYZ);
}

/**
 * Rotate a visible object by 3 angles in radians with user specified order
 */
void VisibleObject::
rotateRadians(const double rR[3], const Transform::RotationOrder order)
{
	getTransform().rotate(rR, Transform::Radians, order);
}
/**
 * Rotate a visible object by an angle in radians around X axis
 */
void VisibleObject::
rotateRadiansX(const double rR)
{
	getTransform().rotateX(rR, Transform::Radians);
}
/**
 * Rotate a visible object by an angle in radians around Y axis
 */
void VisibleObject::
rotateRadiansY(const double rR)
{
	getTransform().rotateY(rR, Transform::Radians);
}
/**
 * Rotate a visible object by an angle in radians around Z axis
 */
void VisibleObject::
rotateRadiansZ(const double rR)
{
	getTransform().rotateZ(rR, Transform::Radians);
}
/**
 * Rotate a visible object by an angle in radians around an axis
 */
void VisibleObject::
rotateRadiansAxis(const double rR, const double axis[3])
{
	getTransform().rotateAxis(rR, Transform::Radians, axis);
}

/**
 * Rotate a visible object by three euler angle in degrees around in XYZ order
 */

void VisibleObject::
rotateDegrees(const double rD[3])
{
	getTransform().rotate(rD, Transform::Degrees, Transform::XYZ);
}
/**
 * Rotate a visible object by three euler angle in degrees around in specified order
 */
void VisibleObject::
rotateDegrees(const double rD[3], const Transform::RotationOrder order)
{
	getTransform().rotate(rD, Transform::Degrees, order);
}
/**
 * Rotate a visible object by an angle in degrees around X axis
 */
void VisibleObject::
rotateDegreesX(const double rD)
{
	getTransform().rotateX(rD, Transform::Degrees);
}
/**
 * Rotate a visible object by an angle in degrees around Y axis
 */
void VisibleObject::
rotateDegreesY(const double rD)
{
	getTransform().rotateY(rD, Transform::Degrees);
}
/**
 * Rotate a visible object by an angle in degrees around Z axis
 */
void VisibleObject::
rotateDegreesZ(const double rD)
{
	getTransform().rotateZ(rD, Transform::Degrees);
}
/**
 * Rotate a visible object by an angle in degrees around an axis
 */
void VisibleObject::
rotateDegreesAxis(const double rD, const double axis[3])
{
	getTransform().rotateAxis(rD, Transform::Degrees, axis);
}
/**
 * Translate a visible object by a translation vector
 */
void VisibleObject::
translate(const double t[3])
{
	getTransform().translate(t);
}

