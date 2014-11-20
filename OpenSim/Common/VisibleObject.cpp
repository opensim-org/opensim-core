/* -------------------------------------------------------------------------- *
 *                        OpenSim:  VisibleObject.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/*  
 * Author:  
 */


//============================================================================
// INCLUDES
//============================================================================
#include <OpenSim/Common/XMLDocument.h>
#include "VisibleObject.h"
#include "Exception.h"
#include "PropertyInt.h"
#include "PropertyStr.h"
#include "PropertyObj.h"
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

    SimTK::Xml::Element e = updDocument()->getRootDataElement(); 
    updateFromXMLNode(e);
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
    _showAxes = aObject._showAxes;
    _displayPreference = aObject._displayPreference;
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
bool VisibleObject::operator==(const VisibleObject &aObject) const
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
    _propScaleFactors.setValueIsDefault(false);
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
    _propGeometrySet.setValueIsDefault(false);
    _geometrySet.adoptAndAppend(new DisplayGeometry(aGeometryFileName));
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
void VisibleObject::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{ 
    SimTK::Array_<SimTK::String> oldGeometryFiles;
    if ( versionNumber < XMLDocument::getLatestVersion()){
        if (versionNumber<20101){
            SimTK::Xml::element_iterator visPropIter = aNode.element_begin("VisibleProperties");
            // Get geometry files and Preferences if any and set them into 
            if (visPropIter!=aNode.element_end()){
                // Move display_prference, and show_axes nodes up to VisibleObject
                SimTK::Xml::element_iterator  prefIter = visPropIter->element_begin("display_preference");
                if (prefIter!= visPropIter->element_end()){
                    SimTK::Xml::Node moveNode = visPropIter->removeNode(prefIter);
                    aNode.insertNodeAfter(aNode.element_end(), moveNode);
                }
                SimTK::Xml::element_iterator  showAxesIter = visPropIter->element_begin("show_axes");
                if (showAxesIter!=aNode.element_end()){
                    SimTK::Xml::Node moveNode = visPropIter->removeNode(showAxesIter);
                    aNode.insertNodeAfter(aNode.element_end(), moveNode);
                }
            }
            SimTK::Xml::element_iterator geometryIter = aNode.element_begin("geometry_files");
            string propValue="";
            bool hasPieces=false;
            if (geometryIter!= aNode.element_end()){
                geometryIter->getValueAs(oldGeometryFiles);
            }
        }
    }
    Object::updateFromXMLNode(aNode, versionNumber);
    if (oldGeometryFiles.size()>0){
        for(unsigned i=0; i< oldGeometryFiles.size(); i++) 
            setGeometryFileName(i, oldGeometryFiles[i]);
    }
}
