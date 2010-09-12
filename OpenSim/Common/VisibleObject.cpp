// VisibleObject.cpp
// Author: Ayman Habib
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

/*  
 * Author:  
 */


//============================================================================
// INCLUDES
//============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include "VisibleObject.h"
#include "Exception.h"
#include "PropertyInt.h"
#include "PropertyStr.h"
#include "PropertyObj.h"
#include "PropertyStrArray.h"
#include "PropertyDblArray.h"
#include "PropertyTransform.h"
#include "Geometry.h"
#include "GeometrySet.h"


using namespace OpenSim;
using namespace std;
using SimTK::Vec3;
using OpenSim::GeometrySet;

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
};

//_____________________________________________________________________________
/**
 * Default constructor.
 */
VisibleObject::VisibleObject():
Object(),
_propGeometrySet(PropertyObj("", GeometrySet())),
_geometrySet((GeometrySet&)_propGeometrySet.getValueObj()),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_transformProp(PropertyTransform("transform", SimTK::Transform())),
_transform(_transformProp.getValueTransform()),
_showAxes(_propShowAxes.getValueBool()),
_displayPreference((DisplayGeometry::DisplayPreference&)_propDisplayPreference.getValueInt()),
_allGeometry(0),
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
_propGeometrySet(PropertyObj("", GeometrySet())),
_geometrySet((GeometrySet&)_propGeometrySet.getValueObj()),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_transformProp(PropertyTransform("transform", SimTK::Transform())),
_transform(_transformProp.getValueTransform()),
_showAxes(_propShowAxes.getValueBool()),
_displayPreference((DisplayGeometry::DisplayPreference&)_propDisplayPreference.getValueInt()),
_allGeometry(0),
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
_propGeometrySet(PropertyObj("", GeometrySet())),
_geometrySet((GeometrySet&)_propGeometrySet.getValueObj()),
_scaleFactors(_propScaleFactors.getValueDblVec()),
_transformProp(PropertyTransform("transform", SimTK::Transform())),
_transform(_transformProp.getValueTransform()),
_showAxes(_propShowAxes.getValueBool()),
_displayPreference((DisplayGeometry::DisplayPreference&)_propDisplayPreference.getValueInt()),
_allGeometry(0),
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
void VisibleObject::setNull()
{
	setupProperties();

	_scaleFactors = 1.0;
	_transform = SimTK::Transform();
	_owner = 0;
	_dependents.setMemoryOwner(false);
	_showAxes=false;
	_displayPreference=DisplayGeometry::GouraudShaded;


}
/**
 * virtual copy constructor
 */
Object* VisibleObject::copy() const
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

	_propGeometrySet.setName("GeometrySet");
	_propGeometrySet.setComment("Set of geometry files and associated attributes, allow .vtp, .stl, .obj");

	_propertySet.append(&_propGeometrySet);

	_propScaleFactors.setName("scale_factors");
	_propScaleFactors.setComment("Three scale factors for display purposes: scaleX scaleY scaleZ");
	_propertySet.append(&_propScaleFactors);

	_transformProp.setName("transform");
	_transformProp.setComment("transform relative to owner specified as 3 rotations (rad) followed by 3 translations rX rY rZ tx ty tz");
	_propertySet.append(&_transformProp);

	_propShowAxes.setName("show_axes");
	_propShowAxes.setComment("Whether to show a coordinate frame");
	_propertySet.append(&_propShowAxes);

	_propDisplayPreference.setName("display_preference");
	_propDisplayPreference.setComment("Display Pref. 0:Hide 1:Wire 3:Flat 4:Shaded Can be overriden for individual geometries");
	_propertySet.append(&_propDisplayPreference);

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
VisibleObject& VisibleObject::operator=(const VisibleObject &aObject)
{
	// BASE CLASS
	Object::operator=(aObject);

	_geometrySet = aObject._geometrySet;
	_scaleFactors = aObject._scaleFactors;
	_transform = aObject._transform;
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
bool VisibleObject::operator==(const VisibleObject &aObject)
{
	return(Object::operator==(aObject));
}


//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Set Scale factors for geometry.
 *
 */
void VisibleObject::setScaleFactors(const SimTK::Vec3& aScaleFactors)
{
	_propScaleFactors.setUseDefault(false);
	_scaleFactors=aScaleFactors;
}
//_____________________________________________________________________________
/**
 * Get Scale factors for geometry.
 *
 */
void VisibleObject::getScaleFactors(SimTK::Vec3& aScaleFactors) const
{
	aScaleFactors = _scaleFactors;
}

void VisibleObject::getRotationsAndTranslationsAsArray6(double aArray[]) const
{
	_transformProp.getRotationsAndTranslationsAsArray6(aArray);
}


void VisibleObject::setGeometryFileName(int i, const std::string &aGeometryFileName)
{
	_geometrySet.append(new DisplayGeometry(aGeometryFileName));
}

const int VisibleObject::getNumGeometryFiles() const
{
	return _geometrySet.getSize();
}
void VisibleObject::setNumGeometryFiles(int n)
{
	_geometrySet.setSize(n);
}
const std::string& VisibleObject::getGeometryFileName(int idx) const 
{
	if (idx > _geometrySet.getSize()-1)
		throw ( Exception("getGeometryFileName: no geometry corresponding to index") );
	return _geometrySet[idx].getGeometryFile();
}

// DisplayPreference
DisplayGeometry::DisplayPreference VisibleObject::getDisplayPreference() const
{
	return _displayPreference;
}
void VisibleObject::setDisplayPreference(const DisplayGeometry::DisplayPreference& aPreference)
{
	_displayPreference = aPreference;
	// Push preference down to pieces
	for(int i=0; i<_geometrySet.getSize(); i++)
		_geometrySet[i].setDisplayPreference(aPreference);
}
// Handle conversion from older format
void VisibleObject::updateFromXMLNode()
{
	int documentVersion = getDocument()->getDocumentVersion();
	if ( documentVersion < XMLDocument::getLatestVersion()){
		// Now check if we need to create a correction controller to replace springs
		if (_node!=NULL && documentVersion<20101){
			// Get geometry files and Preferences if any and set them into 
			DOMElement*visiblePropertiesNode = XMLNode::GetFirstChildElementByTagName(_node, "VisibleProperties");
			if (visiblePropertiesNode!=NULL){
				// Move display_prference, and show_axes nodes up to VisibleObject
				DOMElement*dNode = XMLNode::GetFirstChildElementByTagName(visiblePropertiesNode, "display_prference");
				if (dNode){
					((DOMNode *)visiblePropertiesNode)->removeChild(dNode);
					_node->appendChild(dNode);
				}
				dNode = XMLNode::GetFirstChildElementByTagName(visiblePropertiesNode, "show_axes");
				if (dNode){
					((DOMNode *)visiblePropertiesNode)->removeChild(dNode);
					_node->appendChild(dNode);
				}
			}
			DOMElement*geometryNode = XMLNode::GetFirstChildElementByTagName(_node, "geometry_files");
			string propValue="";
			bool hasPieces=false;
			if (geometryNode!= NULL){
				DOMText* txtNode=NULL;
				if(txtNode=XMLNode::GetTextNode(geometryNode)) {
					// Could still be empty or whiteSpace
					string transcoded = XMLNode::TranscodeAndTrim(txtNode->getNodeValue());
					if (transcoded.length()>0){
						propValue = XMLNode::GetValue<std::string>(txtNode);
						Object::updateFromXMLNode();
						stringstream ss(propValue);
						string nextFile;
						while (ss>>nextFile){
							setGeometryFileName(0, nextFile);	// This actually appends
							hasPieces=true;
						}
					}
				}
			}
		}
	}
	else
		Object::updateFromXMLNode();
}
