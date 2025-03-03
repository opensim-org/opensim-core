/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Object.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDES
//============================================================================

#include "Object.h"

#include "Assertion.h"
#include "Exception.h"
#include "IO.h"
#include "Logger.h"
#include "PropertyTransform.h"
#include "Property_Deprecated.h"
#include "XMLDocument.h"
#include <fstream>

using namespace OpenSim;
using namespace std;
using SimTK::Vec3;
using SimTK::Transform;

//=============================================================================
// STATICS
//=============================================================================
ArrayPtrs<Object>           Object::_registeredTypes;
std::map<string,Object*>    Object::_mapTypesToDefaultObjects;
std::map<string,string>     Object::_renamedTypesMap;

bool                        Object::_serializeAllDefaults=false;
const string                Object::DEFAULT_NAME(ObjectDEFAULT_NAME);

//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Object::~Object() = default;

//_____________________________________________________________________________
/**
 * Default constructor.
 */
Object::Object()
{
    setNull();
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
Object::Object(const string &aFileName, bool aUpdateFromXMLNode)
{
    // INITIALIZATION
    setNull();

    // CREATE DOCUMENT
    // Check file exists before trying to parse it. Is there a faster way to do this?
    // This maybe slower than we like but definitely faster than 
    // going all the way down to the parser to throw an exception for null document!
    // -Ayman 8/06
    OPENSIM_THROW_IF(aFileName.empty(), Exception,
        getClassName() + 
        ": Cannot construct from empty filename. No filename specified.");

    OPENSIM_THROW_IF(!ifstream(aFileName.c_str(), ios_base::in).good(),
        Exception,
        getClassName() + ": Cannot open file " + aFileName +
        ". It may not exist or you do not have permission to read it.");

    _document = std::make_shared<XMLDocument>(aFileName);

    // GET DOCUMENT ELEMENT
    SimTK::Xml::Element myNode =  _document->getRootDataElement(); //either actual root or node after OpenSimDocument

    // UPDATE OBJECT
    // Set current working directory to directory in which we found
    // the XML document so that contained file names will be interpreted
    // relative to that directory. Make sure we switch back properly in case
    // of an exception.
    if (aUpdateFromXMLNode) {
        IO::CwdChanger cwd = IO::CwdChanger::changeToParentOf(aFileName);
        updateFromXMLNode(myNode, _document->getDocumentVersion());
    }
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Object's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Object:
 *
 * 1) Construction based on XML file (@see Object(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Object(const XMLDocument *aDocument).
 * This constructor explicitly requests construction based on an
 * XML document.  In this way the proper connection between an object's node
 * and the corresponding node within the XML document is established.
 * This constructor is a copy constructor of sorts because all essential
 * Object member variables should be held within the XML document.
 * The advantage of this style of construction is that nodes
 * within the XML document, such as comments that may not have any
 * associated Object member variable, are preserved.
 *
 * 3) A call to generateXMLDocument().
 * This method generates an XML document for the Object from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aObject Object to be copied.
 * @see Object(const XMLDocument *aDocument)
 * @see Object(const char *aFileName)
 * @see generateXMLDocument()
 */
Object::Object(const Object &aObject)
{
    setNull();

    // Use copy assignment operator to copy simple data members and the
    // property table; XML document is not copied and the new object is
    // marked "inlined", meaning it is not associated with an XML document.
    *this = aObject;
}

Object::Object(SimTK::Xml::Element& aNode)
{
    setNull();
    updateFromXMLNode(aNode, -1);
}

//-----------------------------------------------------------------------------
// COPY ASSIGNMENT
//-----------------------------------------------------------------------------
/**
 * Assign this object to the values of another.  The XML-associated variable
 * members are not copied-- the XML nodes and/or document must be generated
 * anew for a copied object.
 *
 * @return Reference to this object.
 * @see updateXMLNode()
 */
Object& Object::operator=(const Object& source)
{
    if (&source != this) {
        _name           = source._name;
        _description    = source._description;
        _authors        = source._authors;
        _references     = source._references;
        _propertyTable  = source._propertyTable;
        _document.reset();
        _inlined = true; // meaning: not associated to an XML document
    }
    return *this;
}


 //=============================================================================
// CONSTRUCTION METHODS
//==============================================================================
//_____________________________________________________________________________
/**
 * Set all non-static member variables to their null or default values.
 */
void Object::setNull()
{
    _propertySet.clear();
    _propertyTable.clear();
    _objectIsUpToDate = false;

    _name = "";
    _description = "";
    _authors = "";
    _references = "";

    _document.reset();
    _inlined = true;
}

//-----------------------------------------------------------------------------
// EQUALITY
//-----------------------------------------------------------------------------
// Compare the base class mundane data members, and the properties. Concrete
// Objects should override this but they must make sure to invoke the base
// operator.
bool Object::operator==(const Object& other) const
{
    auto printDiff = [](const std::string& name,
                            const std::string& thisValue,
                            const std::string& otherValue) {
        log_debug("In Object::operator==(), differing {}:\nleft: {}\nright: {}",
                name, thisValue, otherValue);
    };
    if (getConcreteClassName()  != other.getConcreteClassName()) {
        printDiff("ConcreteClassName", getConcreteClassName(),
                  other.getConcreteClassName());
        return false;
    }
    if (getName()               != other.getName()) {
        printDiff("name", getName(), other.getName());
        return false;
    }
    if (getDescription()        != other.getDescription()) {
        printDiff("description", getDescription(), other.getDescription());
        return false;
    }
    if (getAuthors()            != other.getAuthors()) {
        printDiff("authors", getAuthors(), other.getAuthors());
        return false;
    }
    if (getReferences()         != other.getReferences()) {
        printDiff("references", getReferences(), other.getReferences());
        return false;
    }

    // Must have the same number of properties, in the same order.
    const int numProps = getNumProperties();
    if (other.getNumProperties() != numProps) {
        printDiff("number of properties", std::to_string(numProps),
                  std::to_string(other.getNumProperties()));
        return false;
    }

    for (int px = 0; px < numProps; ++px) {
        const AbstractProperty& myProp    = getPropertyByIndex(px);
        const AbstractProperty& otherProp = other.getPropertyByIndex(px);

        if (!myProp.equals(otherProp)) {
            printDiff("property '" + myProp.getName() + "'",
                      myProp.toString(), otherProp.toString());
            return false;
        }
    }

    return true;
}

//-----------------------------------------------------------------------------
// LESS THAN
//-----------------------------------------------------------------------------
// This Object is less than another if the name of this string is less
// than the name of the other Object. TODO: is that a unique ordering?
bool Object::
operator<(const Object& other) const
{
    return getName() < other.getName();
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// NAME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the name of this object.
 */
void Object::
setName(const string &aName)
{
    _name = aName;
}
//_____________________________________________________________________________
/**
 * Get the name of this object.
 */
const string& Object::
getName() const
{
    return(_name);
}

//_____________________________________________________________________________
/**
 * Wrapper to be used on Java side to display objects in tree.
 */
const string& Object::
toString() const
{
    return(getName());
}

//-----------------------------------------------------------------------------
// DESCRIPTION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the description of this object.
 */
void Object::
setDescription(const string &aDescrip)
{
    _description = aDescrip;
}
//_____________________________________________________________________________
/**
 * Get the description of this object.
 */
const string& Object::
getDescription() const
{
    return(_description);
}

//=============================================================================
// PUBLIC PROPERTY ACCESS
//=============================================================================
// TODO: (sherm 20120315) These currently provide support for the deprecated
// PropertySet method of handling properties, not yet fully replaced by the
// PropertyTable approach. The interface here hides the fact that there are
// to different sets of properties -- instead, it will appear that there is
// a single set which will actually be all those from the PropertyTable 
// followed by all those from the PropertySet, so that the property index of
// the first PropertySet property is one larger than that of the last 
// PropertyTable property. Names will be looked up first in the PropertyTable
// and then in the PropertySet.

int Object::
getNumProperties() const {
    return   _propertyTable.getNumProperties() 
           + _propertySet.getSize(); // TODO: remove
}

const AbstractProperty& Object::
getPropertyByIndex(int propertyIndex) const {
    if (!(0 <= propertyIndex && propertyIndex < getNumProperties()))
        throw Exception("Property index " + SimTK::String(propertyIndex)
                        + " out of range 0 <= index < "
                        + SimTK::String(getNumProperties())
                        + " for Object " + getName());

    // TODO: remove deprecated code from here ...
    if (propertyIndex >= _propertyTable.getNumProperties()) {
        const int setIndex = propertyIndex-_propertyTable.getNumProperties();
        return *_propertySet.get(setIndex);
    }
    // ... to here.

    return _propertyTable.getAbstractPropertyByIndex(propertyIndex);
}

AbstractProperty& Object::
updPropertyByIndex(int propertyIndex) {
    if (!(0 <= propertyIndex && propertyIndex < getNumProperties()))
        throw Exception("Property index " + SimTK::String(propertyIndex)
                        + " out of range 0 <= index < "
                        + SimTK::String(getNumProperties())
                        + " for Object " + getName());

    // A property is being modified.
    _objectIsUpToDate = false;

    // TODO: remove deprecated code from here ...
    if (propertyIndex >= _propertyTable.getNumProperties()) {
        const int setIndex = propertyIndex-_propertyTable.getNumProperties();
        return *_propertySet.get(setIndex);
    }
    // ... to here.

    return _propertyTable.updAbstractPropertyByIndex(propertyIndex);
}

bool Object::
hasProperty(const std::string& name) const {
    if (name.empty())
        throw OpenSim::Exception
           ("Object::hasProperty(name): name cannot be empty. For looking up a "
            "one-object, nameless property by object class name, use the other "
            " signature hasProperty<T>() with T the expected object type.");

    if (_propertyTable.hasProperty(name))
        return true;

    // TODO: remove deprecated code from here ...
    if (_propertySet.contains(name))
        return true;
    // ... to here.

    return false;
}

const AbstractProperty& Object::
getPropertyByName(const std::string& name) const {
    const AbstractProperty* p = _propertyTable.getPropertyPtr(name);
    if (p) return *p;

    // TODO: remove deprecated code from here ...
    p = _propertySet.contains(name);
    if (p) return *p;
    // ... to here.

    throw Exception("Property '" + name + "' not present in Object "
                    + getName());
    return *p; //NOT REACHED
}

AbstractProperty& Object::
updPropertyByName(const std::string& name) {
    // A property is being modified.
    _objectIsUpToDate = false;

    AbstractProperty* p = _propertyTable.updPropertyPtr(name);
    if (p) return *p;

    // TODO: remove deprecated code from here ...
    p = _propertySet.contains(name);
    if (p) return *p;
    // ... to here.

    throw Exception("Property '" + name + "' not present in Object "
                    + getName());
    return *p; //NOT REACHED
}

//=============================================================================
// REGISTRATION
//=============================================================================
//-----------------------------------------------------------------------------
// REGISTER TYPE
//-----------------------------------------------------------------------------

//_____________________________________________________________________________
/*
 * Register a supported object type.  A global list of all supported objects
 * (i.e., objects derived from Object) is kept mainly for two purposes:
 *
 * ---- Object Deserialization ----
 * Once a type T is registered, that type can be read from XML files
 * assuming that the type has implemented the following methods:
 *  1)  copy constructor
 *  2)  virtual T* clone() const
 *  3)  static const char* getClassName()
 *  4)  T& operator=() 
 *
 * ---- Initialization by Default Object ----
 * When objects are deserialized, they are constructed based on the registered
 * type and receive all of the registered type's property values.  These
 * values are over-ridden only if there is an element within an XML file that
 * overrides a default element.
 *
 * Because this method is static, registration of object types needs to be
 * done only once per process and an object does not need to be
 * instantiated to do so.
 *
 * This method makes a copy of the specified object.
 *
 * @param aObject Object of the type to be registered.  If the type is
 * already registered, the current object is replaced by a copy of
 * the specified object.
 * @see isValidDefault()
 */
/*static*/ void Object::
registerType(const Object& aObject)
{
    // GET TYPE
    const string& type = aObject.getConcreteClassName();
    if(type.empty()) {
        log_error("Object.registerType: no type name has been set.");
        return;
    }
    log_debug("Object.registerType: {}.", type);

    // REPLACE IF A MATCHING TYPE IS ALREADY REGISTERED
    for(int i=0; i <_registeredTypes.size(); ++i) {
        Object *object = _registeredTypes.get(i);
        if(object->getConcreteClassName() == type) {
            log_debug("Object.registerType: replacing registered object of "
                      "type {} with a new default object of the same type.",
                      type);
            Object* defaultObj = aObject.clone();
            defaultObj->setName(DEFAULT_NAME);
            _registeredTypes.set(i,defaultObj);
            _mapTypesToDefaultObjects[type]= defaultObj;
            return;
        } 
    }

    // REGISTERING FOR THE FIRST TIME -- APPEND
    Object* defaultObj = aObject.clone();
    defaultObj->setName(DEFAULT_NAME);
    _registeredTypes.append(defaultObj);
    _mapTypesToDefaultObjects[type]= defaultObj;
}

/*static*/ void Object::
renameType(const std::string& oldTypeName, const std::string& newTypeName)
{
    if(oldTypeName == newTypeName)
        return; 

    std::map<std::string,Object*>::const_iterator p = 
        _mapTypesToDefaultObjects.find(newTypeName);

    if (p == _mapTypesToDefaultObjects.end())
        throw OpenSim::Exception(
            "Object::renameType(): illegal attempt to rename object type "
            + oldTypeName + " to " + newTypeName + " which is unregistered.",
            __FILE__, __LINE__);

    _renamedTypesMap[oldTypeName] = newTypeName;
}

/*static*/ const Object* Object::
getDefaultInstanceOfType(const std::string& objectTypeTag) {
    std::string actualName = objectTypeTag;
    bool wasRenamed = false; // for a better error message

    // First apply renames if any.

    // Avoid an infinite loop if there is a cycle in the rename table.
    const int MaxRenames = (int)_renamedTypesMap.size();
    int renameCount = 0;
    while(true) {
        std::map<std::string,std::string>::const_iterator newNamep =
            _renamedTypesMap.find(actualName);
        if (newNamep == _renamedTypesMap.end())
            break; // actualName has not been renamed

        if (++renameCount > MaxRenames) {
            throw OpenSim::Exception(
                "Object::getDefaultInstanceOfType(): cycle in rename table "
                "found when looking for '" + objectTypeTag + "'.");
        }

        actualName = newNamep->second;
        wasRenamed = true;
    }

    // Look up the "actualName" default object and return it.
    std::map<std::string,Object*>::const_iterator p = 
        _mapTypesToDefaultObjects.find(actualName);
    if (p != _mapTypesToDefaultObjects.end())
        return p->second;

    // The requested object was not registered. That's OK normally but is
    // a bug if we went through the rename table since you are only allowed
    // to rename things to registered objects.
    if (wasRenamed) {
        throw OpenSim::Exception(
            "Object::getDefaultInstanceOfType(): '" + objectTypeTag
            + "' was renamed to '" + actualName 
            + "' which is not the name of a registered object.");
    }

    return NULL;
}

/*
 * Create a new instance of the type indicated by objectTypeTag.
 * The instance is initialized to the default Object of corresponding type.
 * Note that renaming of old types may occur; the returned object will have
 * the current type tag.
 */
/*static*/ Object* Object::
newInstanceOfType(const std::string& objectTypeTag)
{
    const Object* defaultObj = getDefaultInstanceOfType(objectTypeTag);
    if (defaultObj)
        return defaultObj->clone();
    log_error("Object::newInstanceOfType(): object type '{}' is not a registered "
            "Object! It will be ignored.",
            objectTypeTag);
    throw Exception("Object::newInstanceOfType(): object type '{" +
                    objectTypeTag
                    +"}' is not a registered Object! It will be ignored.");

    return NULL;
}

/*
 * getRegisteredTypenames() is a utility to retrieve all the typenames 
 * registered so far. This is done by traversing the registered objects map, 
 * so only concrete classes are dealt with. The result returned in rTypeNames 
 * should not be cached while more dlls are loaded as they get stale
 * instead the list should be constructed whenever in doubt.
 */
/*static*/ void Object::
getRegisteredTypenames(Array<std::string>& rTypeNames)
{
    std::map<string,Object*>::const_iterator p = 
        _mapTypesToDefaultObjects.begin();
    for (; p != _mapTypesToDefaultObjects.end(); ++p)
        rTypeNames.append(p->first);
    // Renamed type names don't appear in the registeredTypes map, unless
    // they were separately registered.
}

//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// LOCAL STATIC UTILITY FUNCTIONS
//-----------------------------------------------------------------------------
template<class T> static void 
UpdateFromXMLNodeSimpleProperty(Property_Deprecated* aProperty, 
                                SimTK::Xml::Element& aNode, 
                                const string&        aName)
{
    aProperty->setValueIsDefault(true);

    SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
    if (iter == aNode.element_end()) return;    // Not found

    T value;
    iter->getValueAs(value); // fails for Nan, infinity, -infinity, true/false
    aProperty->setValue(value);
    aProperty->setValueIsDefault(false);
}

template<class T> static void 
UpdateFromXMLNodeArrayProperty(Property_Deprecated* aProperty, 
                               SimTK::Xml::Element& aNode, 
                               const string&        aName)
{
    aProperty->setValueIsDefault(true);

    SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
    if (iter == aNode.element_end()) return;    // Not found

    SimTK::Array_<T> value;
    iter->getValueAs(value);

    OpenSim::Array<T> osimValue;
    osimValue.setSize(value.size());
    for(unsigned i=0; i< value.size(); i++) osimValue[i]=value[i];
    aProperty->setValue(osimValue);
    aProperty->setValueIsDefault(false);
}

//------------------------------------------------------------------------------
// OBJECT XML METHODS
//------------------------------------------------------------------------------
// Populate this Object from XML element corresponding to an Object Property.
// We check for a file="xxx" attribute and read the object from that file
// if it is present. Otherwise we punt to updateFromXMLNode() and read the
// object directly from the supplied element.
void Object::readObjectFromXMLNodeOrFile
   (SimTK::Xml::Element& objectElement, 
    int                  versionNumber)
{
    // If object is from non-inlined, detect it and set attributes
    // However we need to do that on the finalized object as copying
    // does not keep track of XML related issues
    const std::string file = 
        objectElement.getOptionalAttributeValueAs<std::string>("file", "");

    // otherwise object is described in file and it has root element
    const bool inlinedObject = (file == ""); 

    if (inlinedObject) {
        // This object comes from the main XML document.
        updateFromXMLNode(objectElement, versionNumber);
        return;
    }

    // This object specifies an external file from which it should be read.

    // When including contents from another file it's assumed file path is 
    // relative to the current working directory, which is usually set to be
    // the directory that contained the top-level XML file.
    try {
        log_info("Reading object from file [{}] cwd ={}.",
            file, IO::getCwd());
        _document = std::make_shared<XMLDocument>(file);
    } catch(const std::exception& ex){
        log_error("Failure reading object from file [{}] cwd ={} Error:{}",
                file, IO::getCwd(), ex.what());
        return;
    }
    _inlined=false;
    SimTK::Xml::Element e = _document->getRootDataElement();
    updateFromXMLNode(e, _document->getDocumentVersion());
}

template<class T> static void 
UpdateXMLNodeSimpleProperty(const Property_Deprecated*  aProperty, 
                            SimTK::Xml::Element&        dParentNode, 
                            const string&               aName)
{
    const T &value = aProperty->getValue<T>();
    if(!aProperty->getValueIsDefault()||Object::getSerializeAllDefaults()) {
        SimTK::Xml::Element elt(aProperty->getName(), value);
        dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
    } 
}

template<class T> static void 
UpdateXMLNodeArrayProperty(const Property_Deprecated*   aProperty,  
                           SimTK::Xml::Element&         dParentNode, 
                           const string&                aName)
{

    const Array<T> &value = aProperty->getValueArray<T>();
    
    if(!aProperty->getValueIsDefault()||Object::getSerializeAllDefaults()) {
        SimTK::Xml::Element elt(aProperty->getName(), value);
        dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
    } 
}

static void 
UpdateXMLNodeVec(const Property_Deprecated*     aProperty, 
                 SimTK::Xml::Element&           dParentNode, 
                 const string&                  aName)
{
    const Array<double> &value = aProperty->getValueArray<double>();
    
    if(!aProperty->getValueIsDefault()||Object::getSerializeAllDefaults()) {
        SimTK::Xml::Element elt(aProperty->getName(), value);
        dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
    } 

}

static void 
UpdateXMLNodeTransform(const Property_Deprecated*   aProperty, 
                       SimTK::Xml::Element&         dParentNode, 
                       const string&                aName)
{

    // Get 6 raw numbers into an array and then use those to update the node
    OpenSim::Array<double> arr(0, 6);
    ((PropertyTransform *)aProperty)->getRotationsAndTranslationsAsArray6(&arr[0]);
    if(!aProperty->getValueIsDefault()||Object::getSerializeAllDefaults()) {
        SimTK::Xml::Element elt(aProperty->getName(), arr);
        dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
    } 
}


//-----------------------------------------------------------------------------
// UPDATE OBJECT
//-----------------------------------------------------------------------------
void Object::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
try {
    // NAME
    const string dName = 
        aNode.getOptionalAttributeValueAs<std::string>("name", "");

    // Set the name of this object.
    setName(dName);

    // UPDATE DEFAULT OBJECTS
    updateDefaultObjectsFromXMLNode(); // May need to pass in aNode

    // LOOP THROUGH PROPERTIES
    for(int i=0; i < _propertyTable.getNumProperties(); ++i) {
        AbstractProperty& prop = _propertyTable.updAbstractPropertyByIndex(i);
        prop.readFromXMLParentElement(aNode, versionNumber);
    }

    // LOOP THROUGH DEPRECATED PROPERTIES
    // TODO: get rid of this
    for(int i=0;i<_propertySet.getSize();i++) {

        Property_Deprecated* property = _propertySet.get(i);

        // TYPE
        Property_Deprecated::PropertyType type = property->getType();   

        // NAME
        string name = property->getName();
        SimTK::String valueString;
        SimTK::String lowerCaseValueString;
        SimTK::Xml::element_iterator iter;
        SimTK::Array_<SimTK::String> value;
        OpenSim::Array<bool> osimValue;
        // VALUE
        switch(type) {

        // Bool
        case(Property_Deprecated::Bool) : 
            property->setValueIsDefault(true);
            iter= aNode.element_begin(name);
            if (iter == aNode.element_end()) break; // Not found
            iter->getValueAs(valueString); // true/false
            lowerCaseValueString = valueString.toLower();
            property->setValue(lowerCaseValueString=="true"?true:false);
            //UpdateFromXMLNodeSimpleProperty<bool>(property, aNode, name);
            property->setValueIsDefault(false);
            break;
        // Int
        case(Property_Deprecated::Int) :
            UpdateFromXMLNodeSimpleProperty<int>(property, aNode, name);
            break;
        // Double
        case(Property_Deprecated::Dbl) :
            property->setValueIsDefault(true);
            iter= aNode.element_begin(name);
            if (iter == aNode.element_end()) continue;  // Not found
            iter->getValueAs(valueString); // special values
            lowerCaseValueString = valueString.toLower();
            if (lowerCaseValueString=="infinity" || lowerCaseValueString=="inf")
                property->setValue(SimTK::Infinity);
            else if (lowerCaseValueString=="-infinity" || lowerCaseValueString=="-inf")
                property->setValue(-SimTK::Infinity);
            else if (lowerCaseValueString=="nan")
                property->setValue(SimTK::NaN);
            else
                UpdateFromXMLNodeSimpleProperty<double>(property, aNode, name);
            property->setValueIsDefault(false);
            break;
        // Str
        case(Property_Deprecated::Str) : 
            UpdateFromXMLNodeSimpleProperty<string>(property, aNode, name);
            break;
        // BoolArray
        case(Property_Deprecated::BoolArray) : 
            // Parse as a String array then map true/false to boolean values
            property->setValueIsDefault(true);
            iter = aNode.element_begin(name);
            if (iter == aNode.element_end()) continue;  // Not found
            iter->getValueAs(value);
            //cout << value << endl;
            osimValue.setSize(value.size());
            for(unsigned i=0; i< value.size(); i++) osimValue[i]=(value[i]=="true");
            property->setValue(osimValue);
            property->setValueIsDefault(false);
            break;
        // IntArray
        case(Property_Deprecated::IntArray) :
            UpdateFromXMLNodeArrayProperty<int>(property,aNode,name);
            break;
        // DblArray
        case(Property_Deprecated::DblArray) :
        case(Property_Deprecated::DblVec) :
        case(Property_Deprecated::Transform) :
            UpdateFromXMLNodeArrayProperty<double>(property,aNode,name);
            break;
        // StrArray
        case(Property_Deprecated::StrArray) :
            UpdateFromXMLNodeArrayProperty<string>(property,aNode,name);
            break;

        // Obj
        case(Property_Deprecated::Obj) : {
            property->setValueIsDefault(true);
            Object &object = property->getValueObj();
            SimTK::Xml::element_iterator iter = 
                aNode.element_begin(object.getConcreteClassName());
            if (iter == aNode.element_end()) 
                continue;   // No element of this object type found.

            // If matchName is set, search through elements of this type to find
            // one that has a "name" attribute that matches the name of this
            // object.
            if (property->getMatchName()) {
                while(object.getName() != 
                        iter->getOptionalAttributeValueAs<std::string>("name", dName) 
                      && iter != aNode.element_end()) 
                {
                    ++iter;
                }
                if (iter != aNode.element_end()) {
                    object.readObjectFromXMLNodeOrFile(*iter, versionNumber);
                    property->setValueIsDefault(false);
                }
            } else {
                object.readObjectFromXMLNodeOrFile(*iter, versionNumber);
                property->setValueIsDefault(false);
            }
            break; 
        }

        // ObjArray AND ObjPtr (handled very similarly)
        case(Property_Deprecated::ObjArray) : 
        case(Property_Deprecated::ObjPtr) : {
            property->setValueIsDefault(true);

            // FIND THE PROPERTY ELEMENT (in aNode)
            const SimTK::Xml::element_iterator propElementIter = aNode.element_begin(name);
            if (propElementIter==aNode.element_end()) 
                break;

            if(type==Property_Deprecated::ObjArray) {
                // CLEAR EXISTING OBJECT ARRAY
                // Eran: Moved after elmt check above so that values set by constructor are kept if
                // property is not specified in the xml file
                property->clearObjArray();
            }

            property->setValueIsDefault(false);

            // LOOP THROUGH PROPERTY ELEMENT'S CHILD ELEMENTS
            // Each element is expected to be an Object of some type given
            // by the element's tag.
            Object *object =NULL;
            int objectsFound = 0;
            SimTK::Xml::element_iterator iter = propElementIter->element_begin();
            while(iter != propElementIter->element_end()){
                // Create an Object of the element tag's type.
                object = newInstanceOfType(iter->getElementTag());
                objectsFound++;

                if(type==Property_Deprecated::ObjPtr) {
                    if(objectsFound > 1){
                        //throw XMLParsingException("Found multiple objects under "+name+" tag, but expected only one.",objElmt,__FILE__,__LINE__);
                    }
                    else{
                        property->setValue(object);
                    }
                } else {
                    property->appendValue(object);
                }
                object->updateFromXMLNode(*iter, versionNumber);
                iter++;
            }
                
            break; }

        // NOT RECOGNIZED
        default :
            log_warn("Object.UpdateObject: WARN- unrecognized property type.");
            break;
        }
    }


    } catch (const Exception &ex) {
        // Important to catch exceptions here so we can restore current working directory...
        // And then we can re-throw the exception
        throw(ex);
    }

}

//-----------------------------------------------------------------------------
// UPDATE DEFAULT OBJECTS FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update the registered default objects based on an object's XML node.
 *
 * This method looks for an element with a tag name "defaults" and reads
 * the objects in that element and registers them using the method
 * Object::registerType().
 */
void Object::
updateDefaultObjectsFromXMLNode()
{
    
    // MUST BE ROOT ELEMENT
    if(!_document) return;

    // GET DEFAULTS ELEMENT
    SimTK::Xml::element_iterator iterDefault =
        _document->getRootDataElement().element_begin("defaults");
    if (iterDefault==_document->getRootDataElement().element_end() || 
        !iterDefault->isValid()) return;    // No defaults, skip over

    if (_document->hasDefaultObjects()) return; // Could be processed by base class, if so skip.

    SimTK::Array_<SimTK::Xml::Element> elts = iterDefault->getAllElements();
    for(unsigned it = 0; it < elts.size(); it++) {
        SimTK::String stg = elts[it].getElementTag();

        // GET DEFAULT OBJECT
        const Object *defaultObject = getDefaultInstanceOfType(stg);
        if(defaultObject==NULL) continue;

        // GET ELEMENT
        const string& type = defaultObject->getConcreteClassName();
        SimTK::Xml::element_iterator iterDefaultType=
            iterDefault->element_begin(type);
        if(iterDefaultType==iterDefault->element_end()) continue;

        // CONSTRUCT AND REGISTER DEFAULT OBJECT
        // Used to call a special copy method that took DOMElement* but 
        // that ended up causing XML to be parsed twice.  Got rid of that
        // copy method! - Eran, Feb/07
        Object *object = defaultObject->clone();
        object->updateFromXMLNode(*iterDefaultType, 
                                  _document->getDocumentVersion());
        object->setName(DEFAULT_NAME);
        registerType(*object);
        _document->addDefaultObject(object); // object will be owned by _document
    } 
}

//-----------------------------------------------------------------------------
// UPDATE XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________

void Object::updateXMLNode(SimTK::Xml::Element& aParent,
                           const AbstractProperty* prop) const
{
    // Handle non-inlined object
    if(!getInlined()) {
        // If object is not inlined we don't want to generate node in original document
        // Handle not-inlined objects first.
        if (!aParent.isValid()) {
            log_warn("Root node of an OpenSim::Object must be inlined, skipping XML reading.");
        }
        else {
        // Can we make this more efficient than recreating the node again?
        // We can possibly check when setInlined() is invoked if we need to do it or not
        // Create a new document and write object to it
        string offlineFileName = getDocumentFileName();
        if(IO::GetPrintOfflineDocuments()) {
            // The problem is that generateChildXMLDocument makes a root which allows print
            // to do its job but root is duplicated. If we don't create the node then generateXMLDocument
            // is invoked which messes up the whole _childDocument mechanism as _document is overwritten.
            _inlined=true;
            print(offlineFileName);
            _inlined=false;
            SimTK::Xml::Element myObjectElement(getConcreteClassName());
            myObjectElement.setAttributeValue("file", offlineFileName);
            aParent.insertNodeAfter(aParent.node_end(), myObjectElement);
        }
        /*
        if (!_refNode) _refNode = XMLNode::AppendNewElementWithComment(aParent,getType(),getName());
        XMLNode::SetAttribute(_refNode,"file",offlineFileName);
        XMLNode::RemoveAttribute(_refNode,"name"); // Shouldn't have a name attribute in the reference document
        XMLNode::RemoveChildren(_refNode); // Shouldn't have any children in the reference document*/
        }
        return;
    }
    
    // GENERATE XML NODE for object
    SimTK::Xml::Element myObjectElement(getConcreteClassName());
    
    // if property is provided and it is not of unnamed type, use the property name
    if(prop && prop->isOneObjectProperty() && !prop->isUnnamedProperty()) {
        myObjectElement.setAttributeValue("name", prop->getName());
    } // otherwise if object has a name use it as the name value
    else if (!getName().empty()) { 
        myObjectElement.setAttributeValue("name", getName());
    }

    aParent.insertNodeAfter(aParent.node_end(), myObjectElement);

    // DEFAULT OBJECTS
    //updateDefaultObjectsXMLNode(aParent);
    if (_document) _document->writeDefaultObjects(myObjectElement);


    // LOOP THROUGH PROPERTIES
    bool wroteAnyProperties = false;
    for(int i=0; i < _propertyTable.getNumProperties(); ++i) {
        const AbstractProperty& prop = _propertyTable.getAbstractPropertyByIndex(i);

        // Don't write out if this is just a default value.
        if (!prop.getValueIsDefault() || Object::getSerializeAllDefaults()) {
            prop.writeToXMLParentElement(myObjectElement);
            wroteAnyProperties = true;
        }
    }

    // LOOP THROUGH DEPRECATED PROPERTIES
    // TODO: get rid of this
    for(int i=0;i<_propertySet.getSize();i++) {

        const Property_Deprecated *prop = _propertySet.get(i);
        if (prop->getValueIsDefault() && !Object::getSerializeAllDefaults())
            continue;

        wroteAnyProperties = true;

        // Add comment if any
        if (!prop->getComment().empty())
            myObjectElement.insertNodeAfter(myObjectElement.node_end(), 
                SimTK::Xml::Comment(prop->getComment()));

        // TYPE
        Property_Deprecated::PropertyType type = prop->getType();

        // NAME
        string name = prop->getName();

        string stringValue="";
        // VALUE
        switch(type) {

        // Bool
        case(Property_Deprecated::Bool) :
            UpdateXMLNodeSimpleProperty<bool>(prop, myObjectElement, name);
            break;
        // Int
        case(Property_Deprecated::Int) :
            UpdateXMLNodeSimpleProperty<int>(prop, myObjectElement, name);
            break;
        // Dbl
        case(Property_Deprecated::Dbl) :
            if (SimTK::isFinite(prop->getValueDbl()))
                UpdateXMLNodeSimpleProperty<double>(prop, myObjectElement, name);
            else {
                if (prop->getValueDbl() == SimTK::Infinity)
                    stringValue="Inf";
                else if (prop->getValueDbl() == -SimTK::Infinity)
                    stringValue="-Inf";
                else if (SimTK::isNaN(prop->getValueDbl()))
                    stringValue="NaN";
                if(!prop->getValueIsDefault()) {
                    SimTK::Xml::Element elt(prop->getName(), stringValue);
                    myObjectElement.insertNodeAfter(myObjectElement.node_end(), elt);
                }
            } 
            break;
        // Str
        case(Property_Deprecated::Str) :
            UpdateXMLNodeSimpleProperty<string>(prop, myObjectElement, name);
            break;
        // BoolArray
        case(Property_Deprecated::BoolArray) :
            // print array as String and add it as such to element
            //UpdateXMLNodeArrayProperty<bool>(prop,myObjectElement,name); BoolArray Handling on Write
            stringValue = "";
            {
                //int n = prop->getArraySize();
                const Array<bool> &valueBs = prop->getValueArray<bool>();
                for (int i=0; i<valueBs.size(); ++i) 
                    stringValue += (valueBs[i]?"true ":"false ");

                SimTK::Xml::Element elt(prop->getName(), stringValue);
                myObjectElement.insertNodeAfter(myObjectElement.node_end(), elt);
            }
            break;
        // IntArray
        case(Property_Deprecated::IntArray) :
            UpdateXMLNodeArrayProperty<int>(prop,myObjectElement,name);
            break;
        // DblArray
        case(Property_Deprecated::DblArray) :
            UpdateXMLNodeArrayProperty<double>(prop,myObjectElement,name);
            break;
        // DblVec3
        case(Property_Deprecated::DblVec) :
            UpdateXMLNodeVec(prop,myObjectElement,name);
            break;
        // Transform
        case(Property_Deprecated::Transform) :
            UpdateXMLNodeTransform(prop,myObjectElement,name);
            break;
        // StrArray
        case(Property_Deprecated::StrArray) :
            UpdateXMLNodeArrayProperty<string>(prop,myObjectElement,name);
            break;

        // Obj
        case(Property_Deprecated::Obj) : {
            //PropertyObj *propObj = (PropertyObj*)prop;
            const Object &object = prop->getValueObj();
            object.updateXMLNode(myObjectElement);
            break; }

        // ObjArray AND ObjPtr (handled very similarly)
        case(Property_Deprecated::ObjArray) :
        case(Property_Deprecated::ObjPtr) : {
                if(type==Property_Deprecated::ObjArray) {
                        // Set all the XML nodes to NULL, and then update them all
                        // in order, with index=0 so each new one is added to the end
                        // of the list (more efficient than inserting each one into
                        // the proper slot).
                    SimTK::Xml::Element objectArrayElement(prop->getName());
                    myObjectElement.insertNodeAfter(myObjectElement.node_end(), objectArrayElement);
                       for(int j=0;j<prop->getArraySize();j++)
                        prop->getValueObjPtr(j)->updateXMLNode(objectArrayElement);
                } else { // ObjPtr
                    const Object *object = prop->getValueObjPtr();
                    SimTK::Xml::Element objectBaseElement(prop->getName());
                    myObjectElement.insertNodeAfter(myObjectElement.node_end(), objectBaseElement);
                    if(object) { // Add node for base classHEREHEREHERE
                        object->updateXMLNode(objectBaseElement);
                    }
                }
            } 
            break; 

        // NOT RECOGNIZED
        default:
            log_warn("Object.UpdateObject: WARN- unrecognized property type.");
            break;
        }
    }

    if (!wroteAnyProperties) {
        myObjectElement.insertNodeAfter(myObjectElement.node_end(), 
            SimTK::Xml::Comment
               ("All properties of this object have their default values."));
    }
}

//_____________________________________________________________________________
/**
 * Update the XML node for defaults object.
 */
void Object::
updateDefaultObjectsXMLNode(SimTK::Xml::Element& aParent)
{
    if (!_document || !_document->hasDefaultObjects())
        return;
    string defaultsTag = "defaults";
    SimTK::Xml::element_iterator elmt = aParent.element_begin(defaultsTag);
    // Not root element- remove defaults
    //if(elmt==aParent.element_end());
    // Root element- write valid defaults
        
        
}
//-----------------------------------------------------------------------------
// NODE
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// DOCUMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
// getDocument(), updDocument() are inline.

//_____________________________________________________________________________
/**
 * Get the document's filename
 *
 * @return Document's filename for this object.
 */
string Object::getDocumentFileName() const
{
    return _document ? _document->getFileName() : "";
}


int Object::getDocumentFileVersion() const
{ 
    return _document ? _document->getDocumentVersion() : -1;
}


//-----------------------------------------------------------------------------
// GENERATE XML DOCUMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Generate a new XML document with this object as the root node.
 */
void Object::
generateXMLDocument()
{
    // CREATE NEW DOCUMENT
    if (!_document)
        _document = std::make_shared<XMLDocument>();
}

//=============================================================================
// XML support for inlining/offlining objects
//=============================================================================
/**
 * Get the value of the inlined flag
 */
bool Object::
getInlined() const
{
    return _inlined;
}

void Object::
setInlined(bool aInlined, const std::string &aFileName)
{
    // Wipe out the previously associated document if we weren't inline.
    if (!_inlined && _document) {
        _document.reset();
    }

    _inlined = aInlined; // set new inline status

    if(!_inlined) {
        _document = std::make_shared<XMLDocument>();
        _document->setFileName(aFileName);
    }
}

//-----------------------------------------------------------------------------
// setAllPropertiesUseDefault
//-----------------------------------------------------------------------------
void Object::
setAllPropertiesUseDefault(bool aUseDefault)
{
    // LOOP THROUGH PROPERTIES
    const int numProps = getNumProperties();
    for (int px = 0; px < numProps; ++px) {
        AbstractProperty& myProp = updPropertyByIndex(px);
        myProp.setAllPropertiesUseDefault(aUseDefault);
    }
}

//=============================================================================
// IO
//=============================================================================
//-----------------------------------------------------------------------------
// PRINT OBJECT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Print the object.
 *
 * @param aFileName File name.  If the file name is NULL, which is the
 * default, the object is printed to standard out.  
 */
bool Object::
print(const string &aFileName) const
{
    // Default to strict exception to avoid creating bad files
    // but for debugging allow users to be more lenient.
    if (Logger::shouldLog(Logger::Level::Debug)) {
        try {
            warnBeforePrint();
        } catch (...) {}
    } else {
        warnBeforePrint();
    }

    {
        // Temporarily change current directory so that inlined files
        // are written to correct relative directory
        IO::CwdChanger cwd = IO::CwdChanger::changeToParentOf(aFileName);
        auto newDoc = std::make_shared<XMLDocument>();
        if (_document) {
            newDoc->copyDefaultObjects(*_document);
        }
        _document = std::move(newDoc);
        SimTK::Xml::Element e = _document->getRootElement();
        updateXMLNode(e);
    }

    _document->print(aFileName);
    return true;
}

//-----------------------------------------------------------------------------
// PRINT PROPERTY INFORMATION
//-----------------------------------------------------------------------------
// Print property information for registered classes. This is used by OpenSim 
// tools to provide a nice "help" capability for objects.

// This signature accepts "className.propertyName", splits out the individual
// segments and calls the other signature.
bool Object::
PrintPropertyInfo(ostream &aOStream,
                  const string &aClassNameDotPropertyName,
                  bool printFlagInfo)
{
    // PARSE NAMES
    string compoundName = aClassNameDotPropertyName;

    string::size_type delimPos = compoundName.find(".");
    string className = compoundName.substr(0,delimPos);
    string propertyName = "";
    if(delimPos!=string::npos) {
        propertyName = compoundName.substr(delimPos+1);
    }

    return PrintPropertyInfo(aOStream, className, propertyName, printFlagInfo);
}

// This is the real method.
bool Object::
PrintPropertyInfo(ostream &aOStream,
                  const string &aClassName, const string &aPropertyName,
                  bool printFlagInfo)
{
    std::stringstream ss;

    if(aClassName=="") {
        // NO CLASS
        int size = _registeredTypes.getSize();
        ss<<"REGISTERED CLASSES ("<<size<<")\n";
        Object *obj;
        for(int i=0;i<size;i++) {
            obj = _registeredTypes.get(i);
            if(obj==NULL) continue;
            ss<<obj->getConcreteClassName()<<endl;
        }
        if (printFlagInfo) {
            ss<<"\n\nUse '-PropertyInfo ClassName' to list the properties of a particular class.\n\n";
        }

        if (aOStream.rdbuf() == std::cout.rdbuf()) {
            log_cout(ss.str());
        } else {
            aOStream << ss.str() << std::endl;
        }

        return true;
    }

    // FIND CLASS
    const Object* object = getDefaultInstanceOfType(aClassName);
    if(object==NULL) {
        if (printFlagInfo) {
            ss<<"\nA class with the name '"<<aClassName<<"' was not found.\n";
            ss<<"\nUse '-PropertyInfo' without specifying a class name to print a listing of all registered classes.\n";
        }

        if (aOStream.rdbuf() == std::cout.rdbuf()) {
            log_cout(ss.str());
        } else {
            aOStream << ss.str() << std::endl;
        }

        return false;
    }

    PropertySet propertySet = object->getPropertySet();
    const Property_Deprecated* prop;
    const AbstractProperty* abstractProperty;
    if((aPropertyName=="")||(aPropertyName=="*")) {
        // NO PROPERTY
        int propertySetSize = propertySet.getSize();
        int propertyTableSize = object->_propertyTable.getNumProperties();
        int size = propertySetSize + propertyTableSize;
        ss<<"\nPROPERTIES FOR "<<aClassName<<" ("<<size<<")\n";
        string comment;
        int i;
        for(i=0;i<propertyTableSize;i++) {
            abstractProperty = 
                &object->_propertyTable.getAbstractPropertyByIndex(i);
            if(abstractProperty==NULL) continue;
            if(aPropertyName=="") {
                ss<<i+1<<". "<<abstractProperty->getName()<<endl;
            } else {
                ss<<"\n"<<i+1<<". "<<abstractProperty->getName()<<"\n";
                comment = abstractProperty->getComment();
                if(!comment.empty()) {
                    string formattedComment = IO::formatText(comment,"\t",80);
                    ss<<"\t"<<formattedComment<<"\n";
                }
            }
        }

        for(;i<size;i++) {
            prop = object->_propertySet.get(i-propertyTableSize);
            if(prop==NULL) continue;
            if(aPropertyName=="") {
                ss<<i+1<<". "<<prop->getName()<<endl;
            } else {
                ss<<"\n"<<i+1<<". "<<prop->getName()<<"\n";
                comment = prop->getComment();
                if(!comment.empty()) {
                    string formattedComment = IO::formatText(comment,"\t",80);
                    ss<<"\t"<<formattedComment<<"\n";
                }
            }
        }

        if (printFlagInfo) {
            ss << "\n\nUse '-PropertyInfo ClassName.PropertyName' to print "
                "info for a particular property.\n";
            if(aPropertyName!="*") {
                ss << "Use '-PropertyInfo ClassName.*' to print info for all "
                    "properties in a class.\n";
            }
        }

        if (aOStream.rdbuf() == std::cout.rdbuf()) {
            log_cout(ss.str());
        } else {
            aOStream << ss.str() << std::endl;
        }

        return true;
    }

    // FIND PROPERTY
    try {
        prop = propertySet.get(aPropertyName);
        // OUTPUT
        ss << "\n" << aClassName << "." << aPropertyName << "\n"
                 << prop->getComment() << "\n";

        if (aOStream.rdbuf() == std::cout.rdbuf()) {
            log_cout(ss.str());
        } else {
            aOStream << ss.str() << std::endl;
        }

        return true;
    } catch(...) {
        try {
            abstractProperty = object->_propertyTable.getPropertyPtr(aPropertyName);
            if (abstractProperty == nullptr) {
                throw Exception("No property '" + aPropertyName +
                        "' class '" + aClassName + "'.");
            }
            // OUTPUT
            ss << "\n" <<aClassName << "." << aPropertyName <<"\n"
                     << abstractProperty->getComment()<<"\n";

            if (aOStream.rdbuf() == std::cout.rdbuf()) {
                log_cout(ss.str());
            } else {
                aOStream << ss.str() << std::endl;
            }

            return true;
        } catch (...) {
            if (printFlagInfo) {
                ss << "\nPrintPropertyInfo: no property with the name "
                    << aPropertyName;
                ss << " was found in class " << aClassName << ".\n";
                ss << "Omit the property name to get a listing of all "
                    "properties in a class.\n";
            }

            if (aOStream.rdbuf() == std::cout.rdbuf()) {
                log_cout(ss.str());
            } else {
                aOStream << ss.str() << std::endl;
            }

            return false;
        }
    }
}


//=============================================================================
// Utilities, factory methods
//=============================================================================
/**
 * makeObjectFromFile creates an OpenSim object based on the type of the object at the root
 * node of the XML file passed in. This is useful since the constructor of Object doesn't have the
 * proper type info. This works by using the defaults table so that "Object" does not need to know about 
 * derived classes, however it uses the defaults table to get an instance, so only "Registered" types will 
 * be considered.
 *
 * Note: The object created is "New" so whoever makes the call also takes ownership of the object
 */
Object* Object::
makeObjectFromFile(const std::string &aFileName)
{
    /**
     * Open the file and get the type of the root element
     */
    try {
        auto doc = std::make_shared<XMLDocument>(aFileName);
        // Here we know the fie exists and is good, chdir to where the file lives
        string rootName = doc->getRootTag();

        bool newFormat = false;
        if (rootName == "OpenSimDocument") { // New format, get child node instead
            rootName = doc->getRootElement().element_begin()->getElementTag();
            newFormat = true;
        }

        Object* newObject = newInstanceOfType(rootName);
        if(newObject == nullptr) {
            throw Exception("Unrecognized XML element '"+rootName+"' and root of file '"+aFileName+"'",__FILE__,__LINE__);
        }

        // Here file is deemed legit, chdir to where the file lives
        // here and restore at the end so offline objects are handled
        // properly
        IO::CwdChanger cwd = IO::CwdChanger::changeToParentOf(aFileName);
        newObject->_document = std::move(doc);
        if (newFormat) {
            newObject->updateFromXMLNode(*newObject->_document->getRootElement().element_begin(), newObject->_document->getDocumentVersion());
        } else {
            SimTK::Xml::Element e = doc->getRootElement();
            newObject->updateFromXMLNode(e, 10500);
        }

        return newObject;
    } catch(const std::exception& x) {
        log_error(x.what());
        return nullptr;
    } catch(...) {
        // Document couldn't be opened, or something went really bad
        return nullptr;
    }
}

void Object::makeObjectNamesConsistentWithProperties()
{
    bool prevDirtyFlag = isObjectUpToDateWithProperties();
    bool modificationMade = false;

    // Cycle through this object's Object properties and make sure those
    // that are objects have names that are consistent with object property. 
    for (int i = 0; i < getNumProperties(); ++i) {
        auto& prop = updPropertyByIndex(i);  // CARE: sets object as not up to date
        // check if property is of type Object
        if (prop.isObjectProperty()) {
            // a property is a list so cycle through its contents
            for (int j = 0; j < prop.size(); ++j) {
                Object& obj = prop.updValueAsObject(j);
                // If a single object property, set the object's name to the
                // property's name, otherwise it will be inconsistent with
                // what is serialized (property name).
                if (!prop.isUnnamedProperty() && prop.isOneObjectProperty() && obj.getName() != prop.getName()) {
                    obj.setName(prop.getName());
                    modificationMade = true;
                }
                // In any case, any objects that are properties of this object
                // also need to be processed
                obj.makeObjectNamesConsistentWithProperties();
            }
        }
    }

    if (!modificationMade) {
        if (prevDirtyFlag) {
            setObjectIsUpToDateWithProperties();
        } else {
            clearObjectIsUpToDateWithProperties();
        }
    }
}

void Object::setObjectIsUpToDateWithProperties()
{
    _objectIsUpToDate = true;
}

void Object::updateFromXMLDocument()
{
    OPENSIM_ASSERT_FRMOBJ(_document != nullptr);

    SimTK::Xml::Element e = _document->getRootDataElement();
    IO::CwdChanger cwd = IO::CwdChanger::changeToParentOf(_document->getFileName());
    updateFromXMLNode(e, _document->getDocumentVersion());
}

void Object::setDocument(XMLDocument* doc)
{
    _document.reset(doc);
}

std::string Object::dump() const {
    SimTK::String outString;
    XMLDocument doc;
    Object::setSerializeAllDefaults(true);
    SimTK::Xml::Element elem = doc.getRootElement();
    updateXMLNode(elem);
    Object::setSerializeAllDefaults(false);
    doc.getRootElement().node_begin()->writeToString(outString);
    return std::move(outString);
}


void Object::setDebugLevel(int newLevel) {
    switch (newLevel) {
    case -4: Logger::setLevel(Logger::Level::Off);
        break;
    case -3: Logger::setLevel(Logger::Level::Critical);
        break;
    case -2: Logger::setLevel(Logger::Level::Error);
        break;
    case -1: Logger::setLevel(Logger::Level::Warn);
        break;
    case 0: Logger::setLevel(Logger::Level::Info);
        break;
    case 1: Logger::setLevel(Logger::Level::Debug);
        break;
    case 2: Logger::setLevel(Logger::Level::Trace);
        break;
    case 3:
        // Backwards compatibility.
        Logger::setLevel(Logger::Level::Trace);
        break;
    default:
        OPENSIM_THROW(
                Exception, "Expected newLevel to be -4, -3, "
                           "-2, -1, 0, 1, 2, or 3; but got {}.",
                newLevel);
    }
}

int Object::getDebugLevel() {
    const auto level = Logger::getLevel();
    switch (level) {
    case Logger::Level::Off: return -4;
    case Logger::Level::Critical: return -3;
    case Logger::Level::Error: return -2;
    case Logger::Level::Warn: return -1;
    case Logger::Level::Info: return 0;
    case Logger::Level::Debug: return 1;
    case Logger::Level::Trace: return 2;
    default:
        OPENSIM_THROW(Exception, "Internal error.");
    }
}

/**
    * The following code accounts for an object made up to call 
    * RegisterTypes_osimCommon function on entry to the DLL in a cross platform manner 
    * 
    */ 
// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond  
class osimCommonInstantiator 
{ 
public: 
        osimCommonInstantiator(); 
private: 
        void registerDllClasses(); 
};
    
osimCommonInstantiator::osimCommonInstantiator() 
{ 
        registerDllClasses(); 
} 
    
extern "C" OSIMCOMMON_API void RegisterTypes_osimCommon(); 
void osimCommonInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimCommon(); 
}
#ifndef OPENSIM_DISABLE_STATIC_TYPE_REGISTRATION
    static osimCommonInstantiator instantiator;
#endif
/// @endcond
