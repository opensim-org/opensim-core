// Object.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)   2005, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDES
//============================================================================
#include <fstream>
#include "Object.h"
#include "XMLDocument.h"
#include "Exception.h"
#include "Property.h"
#include "PropertyObj.h"
#include "PropertyDblVec.h"
#include "PropertyTransform.h"
#include "IO.h"
#include "OldVersionException.h"

using namespace OpenSim;
using namespace std;
using SimTK::Xml;
using SimTK::Vec3;
using SimTK::Transform;

//=============================================================================
// STATICS
//=============================================================================
ArrayPtrs<Object> Object::_Types;

stringsToObjects Object::_mapTypesToDefaultObjects;
bool Object::_serializeAllDefaults=false;

#include <vector>
#include <algorithm>  // Include algorithms

//============================================================================
// CONSTANTS
//============================================================================
const string Object::DEFAULT_NAME(ObjectDEFAULT_NAME);
int Object::_debugLevel = 0;
Array<std::string> Object::_deprecatedTypes;
//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Object::~Object()
{
	//delete _document;
	if (_debugLevel==4)
		std::cout << "deleting object of type:" << getType() << " named:" << getName() << endl;
}

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
	{
		// Check file exists before trying to parse it. Is there a faster way to do this?
		// This maybe slower than we like but definitely faster than 
		// going all the way down to the parser to throw an exception for null document!
		// -Ayman 8/06
		if(aFileName.empty()) {
			string msg =
				"Object: ERR- Empty filename encountered.";
			throw Exception(msg,__FILE__,__LINE__);
		} else 
			if(!ifstream(aFileName.c_str(), ios_base::in).good()) {
			string msg =
				"Object: ERR- Could not open file " + aFileName+ ". It may not exist or you don't have permission to read it.";
			throw Exception(msg,__FILE__,__LINE__);
		}	
	}

	_document = new XMLDocument(aFileName);

	// GET DOCUMENT ELEMENT
	SimTK::Xml::Element myNode =  _document->getRootDataElement(); //either actual root or node after OpenSimDocument

	// UPDATE OBJECT
	if (aUpdateFromXMLNode) updateFromXMLNode(myNode, _document->getDocumentVersion());

}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * Copy constructors for all Object's only copy the non-XML variable
 * members of the object; that is, the object's DOMnode and XMLDocument
 * are not copied but set to NULL.  The reason for this is that for the
 * object and all its derived classes to establish the correct connection
 * to the XML document nodes, the the object would need to reconstruct based
 * on the XML document not the values of the object's member variables.
 *
 * There are three proper ways to generate an XML document for an Object:
 *
 * 1) Construction based on XML file (@see Object(const char *aFileName)).
 * In this case, the XML document is created by parsing the XML file.
 *
 * 2) Construction by Object(const XMLDocument *aDocument).
 * This constructor explictly requests construction based on an
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

	// COPY TYPE AND NAME
	*this = aObject;
}

Object::Object(SimTK::Xml::Element& aNode)
{
	setNull();
	updateFromXMLNode(aNode, -1);
}

//_____________________________________________________________________________
/**
 * Construct and return a copy of this object.
 *
 * The object is allocated using the new operator, so the caller is
 * responsible for deleting the returned object.
 *
 * @return Copy of this object.
 */
Object* Object::
copy() const
{
	Object *object = new Object(*this);
	return(object);
}

 //=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to their null or default values.
 */
void Object::
setNull()
{
	setType("Object");
	setName("");

	setupProperties();

	_document = NULL;
	_inlined = true;
	_propertySet.clear();
	_description = "";
	_authors = "";
	_references = "";

}
//_____________________________________________________________________________
/**
 * Set up the serialized member variables.  This involves both generating
 * the properties and connecting them to the local pointers used to access
 * the serialized member variables.
 */
void Object::
setupProperties()
{

	// CURRENTLY THERE ARE NO SERIALIZED MEMBERS IN Object

}

//_____________________________________________________________________________
/**
 * Perform any initializations that should occur upon instantiation.
 */
void Object::
init()
{

	// CURRENTLY THERE ARE NO INITIALIZATIONS NEEDED.

}

//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assign this object to the values of another.  The XML-associated variable
 * members are not copied-- the XML nodes and/or document must be generated
 * anew for a copied object.
 *
 * @return Reference to this object.
 * @see updateXMLNode()
 * @see generateXMLNode()
 */
Object& Object::
operator=(const Object &aObject)
{
	setType(aObject.getType());
	setName(aObject.getName());
	_authors=aObject.getAuthors();
	_references=aObject.getReferences();
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
bool Object::
operator==(const Object &aObject) const
{
	if(getType() != aObject.getType()) return(false);
	if(getName() != aObject.getName()) return(false);
	if (_authors!= aObject.getAuthors()) return(false);
	if (_references!= aObject.getReferences()) return(false);
	bool equal = true;
	for (int i=0; i< _propertySet.getSize() && equal ; i++){
		const Property& myProperty = *(_propertySet.get(i));
		const Property& theirProperty = *(aObject.getPropertySet().get(i));
		switch(myProperty.getType()){
			case (Property::Bool): 
				if (myProperty.getValueBool()!=theirProperty.getValueBool()) return false;
				continue;
			case Property::Int:
				if (myProperty.getValueInt()!=theirProperty.getValueInt()) return false;
				continue;
			case Property::Dbl:
				if (fabs(myProperty.getValueDbl()-theirProperty.getValueDbl())>1e-7) return false;
				continue;
			case Property::Str:
				if (myProperty.getValueStr()!=theirProperty.getValueStr()) return false;
				continue;
			case Property::Obj:
				if (!(myProperty.getValueObj()==theirProperty.getValueObj())) return false;
				continue;
			case Property::ObjPtr:
				equal = (myProperty==theirProperty);
				if (!equal) return false;
				continue;
			case Property::BoolArray:
				for(int j=0; j < myProperty.getValueBoolArray().getSize() && equal; j++)
					equal= (myProperty.getValueBoolArray().get(j)==
								theirProperty.getValueBoolArray().get(j));
				if (!equal) return false;
				continue;
			case Property::IntArray:
				for(int j=0; j < myProperty.getValueIntArray().getSize() && equal; j++)
					equal= (myProperty.getValueIntArray().get(j) ==
								theirProperty.getValueIntArray().get(j));
				if (!equal) return false;
				continue;
			case Property::DblArray:
				for(int j=0; j < myProperty.getValueDblArray().getSize() && equal; j++)
					equal= (fabs(myProperty.getValueDblArray().get(j)-
								theirProperty.getValueDblArray().get(j))<1e-8);
				if (!equal) return false;
				continue;
			case Property::StrArray:
				for(int j=0; j < myProperty.getValueStrArray().getSize() && equal; j++)
					equal= (myProperty.getValueStrArray().get(j)==
								theirProperty.getValueStrArray().get(j));
				if (!equal) return false;
				continue;
	
			case Property::ObjArray:
				equal = (myProperty==theirProperty);
				if (!equal) return false;
				continue;
			case Property::DblVec:
				{
				int M = myProperty.getArraySize();
				equal = (((const PropertyDblVec_<1>&)myProperty).getValueDblVec() - 
					((const PropertyDblVec_<1>&)theirProperty).getValueDblVec()).norm() < 1e-8;
				if (!equal) return false;
				continue;
				}
			case Property::Transform:
				const SimTK::Transform& t1 = ((const PropertyTransform&)myProperty).getValueTransform();
				const SimTK::Transform& t2 = ((const PropertyTransform&)theirProperty).getValueTransform();
				SimTK::Transform tComposed =  t1.compose(t2.invert());
				equal = (tComposed.p().norm() < 1e-8 &&
					fabs(tComposed.R().trace()-3) < 1e-8);
				if (!equal) return false;
				continue;

		}
	}

	if (equal)
		equal = _propertyTable == aObject._propertyTable;

	return(equal);
}

//-----------------------------------------------------------------------------
// LESS THAN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine if this property is less than another.
 *
 * This property is less than another if the name of this string is less
 * than the name of the other property.
 *
 * @param aProperty Property for which to make the less than test.
 * @return True if this object's name is less than the other, false otherwise.
 * used to put Objects in Arrays/maps
 */
bool Object::
operator<(const Object &aObject) const
{
	return(_name < aObject._name);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// TYPE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the type of this object.
 *
 * @param aType Type of this object represented as a string.  In most all
 * cases, the type should be the name of the class.
 */
void Object::
setType(const string &aType)
{
	_type = aType;
}
//_____________________________________________________________________________
/**
 * Get the type of this object.
 *
 * @return The type of the object.  In most all cases, the type should be the
 * the name of the class.
 */
const string& Object::
getType() const
{
	return(_type);
}

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
// REGISTRATION
//=============================================================================
//-----------------------------------------------------------------------------
// REGISTER TYPE
//-----------------------------------------------------------------------------
void Object::
RenameType(const std::string& oldTypeName, const Object& newTypeObject)
{
	Object* objectCopy = newTypeObject.copy();
	if (objectCopy != NULL){
		objectCopy->setType(oldTypeName);
		RegisterType(*objectCopy);
		if (_deprecatedTypes.findIndex(oldTypeName)==-1)
			_deprecatedTypes.append(oldTypeName);
	}
}

//_____________________________________________________________________________
/**
 * Register a supported object type.  A global list of all supported objects
 * (i.e., objects derived from Object) is kept mainly for two purposes:
 *
 * ---- Object Deserialization ----
 * Once a type is registered, that type can be read from XML files
 * assuming that the type has implemented the following methods:
 *	1)	copy constructor
 *	2)	virtual Object* copy() const,
 *	3)	<class>& operator=() (where the class name is substituted for <class>),
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
void Object::
RegisterType(const Object &aObject)
{
	// GET TYPE
	const string &type = aObject.getType();
	if(type.empty()) {
		printf("Object.RegisterType: ERR- no type name has been set.\n");
		return;
	}
	if (_debugLevel>=2) {
		cout << "Object.RegisterType: " << type << " .\n";
	}

	// REPLACE IF A MATCHING TYPE IS ALREADY REGISTERED
	int i;
	for(i=0;i<_Types.getSize();i++) {
		Object *object = _Types.get(i);
		if(object->getType() == type) {
			if(_debugLevel>=2) {
				cout<<"Object.RegisterType: replacing registered object of type ";
				cout<<type;
				cout<<"\n\twith a new default object of the same type."<<endl;
			}
			_Types.set(i,aObject.copy());
			_Types.get(i)->setName(DEFAULT_NAME);
			_mapTypesToDefaultObjects[aObject.getType()]= _Types.get(i);
			return;
		} 
	}

	// APPEND
	Object *defaultObj = aObject.copy();
	defaultObj->setType(aObject.getType());	// Since the copy overwrites type
	_Types.append(defaultObj);
	// Object is registered for first time
	//if (defaultObj->getAuthors()!="")
	//	cout << "This software include component "<<aObject.getType()<< " developed by "<< defaultObj->getAuthors() << endl;

	_mapTypesToDefaultObjects[aObject.getType()]= defaultObj;
	_Types.getLast()->setName(DEFAULT_NAME);
}


//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// UTILITY FUNCTIONS
//-----------------------------------------------------------------------------
template<class T> void UpdateFromXMLNodeSimpleProperty(Property *aProperty, SimTK::Xml::Element& aNode, const string &aName)
{
	aProperty->setUseDefault(true);
	SimTK::String string;
	aNode.writeToString(string);
	SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
	if (iter == aNode.element_end()) return;	// Not found

	T value;
	iter->getValueAs(value); // fails for Nan, infinity, -infinity, true/false
			aProperty->setValue(value);
			aProperty->setUseDefault(false);
}

template<class T> void UpdateFromXMLNodeSimpleProperty2(AbstractProperty *aAbstractProperty, SimTK::Xml::Element& aNode, const string &aName)
{
	aAbstractProperty->setUseDefault(true);
	SimTK::String string;
	aNode.writeToString(string);
	SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
	if (iter == aNode.element_end()) return;	// Not found

	Property2<T> *aProperty = dynamic_cast<Property2<T> *>(aAbstractProperty);
	T value;
	iter->getValueAs(value); // fails for Nan, infinity, -infinity, true/false
			aProperty->setValue(value);
			aProperty->setUseDefault(false);
}

template<class T> void UpdateFromXMLNodeArrayProperty(Property *aProperty, SimTK::Xml::Element& aNode, const string &aName)
{
	aProperty->setUseDefault(true);
	//SimTK::String string;
	//aNode.writeToString(string);
	SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
	if (iter == aNode.element_end()) return;	// Not found

	SimTK::Array_<T> value;
	iter->getValueAs(value);
	//cout << value << endl;
	OpenSim::Array<T> osimValue;
	osimValue.setSize(value.size());
	for(unsigned i=0; i< value.size(); i++) osimValue[i]=value[i];
	aProperty->setValue(osimValue);
		aProperty->setUseDefault(false);
}

template<class T> void UpdateFromXMLNodeArrayProperty2(AbstractProperty *aAbstractProperty, SimTK::Xml::Element& aNode, const string &aName)
{
	aAbstractProperty->setUseDefault(true);
	//SimTK::String string;
	//aNode.writeToString(string);
	SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
	if (iter == aNode.element_end()) return;	// Not found

	Property2< OpenSim::Array<T> > *aProperty = dynamic_cast<Property2< OpenSim::Array<T> > *>(aAbstractProperty);
	SimTK::Array_<T> value;
	iter->getValueAs(value);
	//cout << value << endl;
	OpenSim::Array<T> osimValue;
	osimValue.setSize(value.size());
	for(unsigned i=0; i< value.size(); i++) osimValue[i]=value[i];
	aProperty->setValue(osimValue);
		aProperty->setUseDefault(false);
}

void UpdateFromXMLNodeVec3Property2(AbstractProperty *aAbstractProperty, SimTK::Xml::Element& aNode, const string &aName)
{
	aAbstractProperty->setUseDefault(true);
	//SimTK::String string;
	//aNode.writeToString(string);
	SimTK::Xml::element_iterator iter = aNode.element_begin(aName);
	if (iter == aNode.element_end()) return;	// Not found

	Property2<Vec3> *aProperty = dynamic_cast<Property2<Vec3> *>(aAbstractProperty);
	SimTK::Array_<double> value;
	iter->getValueAs(value);
	//cout << value << endl;
	Vec3 &propertyValues = aProperty->updValue();
	propertyValues[0]=value[0];
	propertyValues[1]=value[1];
	propertyValues[2]=value[2];
	aProperty->setUseDefault(false);
}

void Object::	// Populate Object from XML node corresponding to Obj property
InitializeObjectFromXMLNode(Property *aProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber)
{
	SimTK::String toString;
	rObjectElement->writeToString(toString);
	// If object is from non-inlined, detect it and set attributes
	// However we need to do that on the finalized object as copying
	// does not keep track of XML related issues
	//DOMElement *refNode;
	//XMLDocument *childDocument;
	std::string file = "";
	file = rObjectElement->getOptionalAttributeValueAs<std::string>("file", file);

	bool inlinedObject = (file == ""); // otherwise object is described in file and it has root element

	aProperty->setUseDefault(false);

	// CONSTRUCT THE OBJECT BASED ON THE ELEMENT
	// Used to call a special copy method that took DOMElement* but 
	// that ended up causing XML to be parsed twice.  Got rid of that
	// copy method! - Eran, Feb/07
	
	// Set inlining attributes on final object
	if (!inlinedObject){
		XMLDocument* newDoc = new XMLDocument(file);
		aObject->_inlined=false;
		SimTK::Xml::Element e = newDoc->getRootDataElement();
		aObject->updateFromXMLNode(e, newDoc->getDocumentVersion());
	}
	else
		aObject->updateFromXMLNode(*rObjectElement, versionNumber);
	//aObject->updateFromXMLNode();
}

void Object::	// Populate Object from XML node corresponding to Obj property
InitializeObjectFromXMLNode2(AbstractProperty *aAbstractProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber)
{
	SimTK::String toString;
	rObjectElement->writeToString(toString);
	// If object is from non-inlined, detect it and set attributes
	// However we need to do that on the finalized object as copying
	// does not keep track of XML related issues
	//DOMElement *refNode;
	//XMLDocument *childDocument;
	std::string file = "";
	file = rObjectElement->getOptionalAttributeValueAs<std::string>("file", file);

	bool inlinedObject = (file == ""); // otherwise object is described in file and it has root element

	aAbstractProperty->setUseDefault(false);

	// CONSTRUCT THE OBJECT BASED ON THE ELEMENT
	// Used to call a special copy method that took DOMElement* but 
	// that ended up causing XML to be parsed twice.  Got rid of that
	// copy method! - Eran, Feb/07
	
	// Set inlining attributes on final object
	if (!inlinedObject){
		XMLDocument* newDoc = new XMLDocument(file);
		aObject->_inlined=false;
		SimTK::Xml::Element e = newDoc->getRootDataElement();
		aObject->updateFromXMLNode(e, newDoc->getDocumentVersion());
	}
	else
		aObject->updateFromXMLNode(*rObjectElement, versionNumber);
	//aObject->updateFromXMLNode();
}

template<class T> void UpdateXMLNodeSimpleProperty(const Property *aProperty, SimTK::Xml::Element& dParentNode, const string &aName)
{
	const T &value = aProperty->getValue<T>();
	if(!aProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aProperty->getName(), value);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	} 
}

template<class T> void UpdateXMLNodeSimpleProperty2(const AbstractProperty *aAbstractProperty, SimTK::Xml::Element& dParentNode, const string &aName)
{
	const Property2<T> *aProperty = dynamic_cast<const Property2<T> *>(aAbstractProperty);
	const T &value = aProperty->getValue();
	if(!aProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aProperty->getName(), value);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	} 
}

template<class T> void UpdateXMLNodeArrayProperty(const Property *aProperty,  SimTK::Xml::Element& dParentNode, const string &aName)
{

	const Array<T> &value = aProperty->getValueArray<T>();
	
	if(!aProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aProperty->getName(), value);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	} 
}

template<class T> void UpdateXMLNodeArrayProperty2(const AbstractProperty *aAbstractProperty,  SimTK::Xml::Element& dParentNode, const string &aName)
{
	const Property2< Array<T> > *aProperty = dynamic_cast<const Property2< Array<T> > *>(aAbstractProperty);
	const Array<T> &value = aProperty->getValue();
	
	if(!aProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aProperty->getName(), value);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	} 
}

void UpdateXMLNodeVec(const Property *aProperty, SimTK::Xml::Element& dParentNode, const string &aName)
{
	const Array<double> &value = aProperty->getValueArray<double>();
	
	if(!aProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aProperty->getName(), value);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	} 

}

void UpdateXMLNodeVec3(const AbstractProperty *aAbstractProperty, SimTK::Xml::Element& dParentNode, const string &aName)
{
	const Property2<Vec3> *aProperty = dynamic_cast<const Property2<Vec3> *>(aAbstractProperty);
	const Vec3 &vector = aProperty->getValue();
	const Array<double> values(0.0, 3);
	values[0] = vector[0];
	values[1] = vector[1];
	values[2] = vector[2];
	
	if(!aAbstractProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aAbstractProperty->getName(), values);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	} 

}

void UpdateXMLNodeTransform(const Property *aProperty, SimTK::Xml::Element& dParentNode, const string &aName)
{

	// Get 6 raw numbers into an array and then use those to update the node
	OpenSim::Array<double> arr(0, 6);
	((PropertyTransform *)aProperty)->getRotationsAndTranslationsAsArray6(&arr[0]);
	//if(!aProperty->getUseDefault()||Object::getSerializeAllDefaults()) {
		SimTK::Xml::Element elt(aProperty->getName(), arr);
		dParentNode.insertNodeAfter(dParentNode.node_end(), elt);
	//} 
}
//-----------------------------------------------------------------------------
// UPDATE OBJECT
//-----------------------------------------------------------------------------
//__________
void Object::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
try {
	// NAME
	string dName="";
	dName = aNode.getOptionalAttributeValueAs<std::string>("name", dName);
	setName(dName);

	// UPDATE DEFAULT OBJECTS
	updateDefaultObjectsFromXMLNode(); // May need to pass in aNode

	// LOOP THROUGH PROPERTIES
	for(int i=0;i<_propertySet.getSize();i++) {

		Property *property = _propertySet.get(i);

		// TYPE
		Property::PropertyType type = property->getType();	

		// NAME
		string name = property->getName();
		if(_debugLevel>=4) {
			cout << "Object.updateFromXMLNode: ("<<getType()<<":"<<getName()<<") updating property " << name << endl;
		}

		SimTK::String valueString;
		SimTK::String lowerCaseValueString;
		SimTK::Xml::element_iterator iter;
		SimTK::Array_<SimTK::String> value;
		OpenSim::Array<bool> osimValue;
		// VALUE
		switch(type) {

		// Bool
		case(Property::Bool) : 
			iter= aNode.element_begin(name);
			if (iter == aNode.element_end()) break;	// Not found
			iter->getValueAs(valueString); // true/false
			lowerCaseValueString = valueString.toLower();
			property->setValue(lowerCaseValueString=="true"?true:false);
			//UpdateFromXMLNodeSimpleProperty<bool>(property, aNode, name);
			break;
		// Int
		case(Property::Int) :
			UpdateFromXMLNodeSimpleProperty<int>(property, aNode, name);
			break;
		// Double
		case(Property::Dbl) :
			iter= aNode.element_begin(name);
			if (iter == aNode.element_end()) continue;	// Not found
			iter->getValueAs(valueString); // special values
			lowerCaseValueString = valueString.toLower();
			if (lowerCaseValueString=="infinity")
				property->setValue(SimTK::Infinity);
			else if (lowerCaseValueString=="-infinity")
				property->setValue(-SimTK::Infinity);
			else if (lowerCaseValueString=="nan")
				property->setValue(SimTK::NaN);
			else
				UpdateFromXMLNodeSimpleProperty<double>(property, aNode, name);
			break;
		// Str
		case(Property::Str) : 
			UpdateFromXMLNodeSimpleProperty<string>(property, aNode, name);
			break;
		// BoolArray
		case(Property::BoolArray) : 
			// Parse as a String array then map true/false to boolean values
			property->setUseDefault(true);
			iter = aNode.element_begin(name);
			if (iter == aNode.element_end()) continue;	// Not found
			iter->getValueAs(value);
			//cout << value << endl;
			osimValue.setSize(value.size());
			for(unsigned i=0; i< value.size(); i++) osimValue[i]=(value[i]=="true");
			property->setValue(osimValue);
			property->setUseDefault(false);
			break;
		// IntArray
		case(Property::IntArray) :
			UpdateFromXMLNodeArrayProperty<int>(property,aNode,name);
			break;
		// DblArray
		case(Property::DblArray) :
		case(Property::DblVec) :
		case(Property::Transform) :
			UpdateFromXMLNodeArrayProperty<double>(property,aNode,name);
			break;
		// StrArray
		case(Property::StrArray) :
			UpdateFromXMLNodeArrayProperty<string>(property,aNode,name);
			break;

		// Obj
		case(Property::Obj) : {
			property->setUseDefault(true);
			Object &object = property->getValueObj();
			SimTK::Xml::element_iterator iter = aNode.element_begin(object.getType());
			if (iter == aNode.element_end()) continue;	// Not found
			if (((PropertyObj*)property)->getMatchName()){
					while(object.getName() != iter->getOptionalAttributeValueAs<std::string>("name", dName) &&
						 iter != aNode.element_end()){
							iter++;
				}
					if (iter != aNode.element_end())
						InitializeObjectFromXMLNode(property, iter, &object, versionNumber);
					}
			else
				InitializeObjectFromXMLNode(property, iter, &object, versionNumber);
			break; }

		// ObjArray AND ObjPtr (handled very similarly)
		case(Property::ObjArray) : 
		case(Property::ObjPtr) : {
			property->setUseDefault(true);

			// GET ENCLOSING ELEMENT
			//DOMElement *elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			//if(elmt==NULL) {
			//	if (_debugLevel>=4) {
			//		cout<<"Object.updateFromXMLNode: ERR- failed to find element ";
			//		cout<<name<<endl;
			//	}
			//	break;
			//}

			if(type==Property::ObjArray) {
				// CLEAR EXISTING OBJECT ARRAY
				// Eran: Moved after elmt check above so that values set by constructor are kept if
				// property is not specified in the xml file
				property->clearObjArray();
			}

			// Call parseFileAttribute to take care of the case where a file attribute points to
			// an external XML file.  Essentially this will make elmt point to the top level
			// element of the other file
			//{
			//	DOMElement *refNode;
			//	XMLDocument *childDocument;
			//	parseFileAttribute(elmt, refNode, childDocument, elmt);
			//}

			// LOOP THROUGH CHILD NODES
			const SimTK::Xml::element_iterator& propElementIter =  aNode.element_begin(name);
			if (propElementIter==aNode.element_end()) break;
			Object *object =NULL;
			int objectsFound = 0;
			SimTK::Xml::element_iterator iter = propElementIter->element_begin();
			while(iter != propElementIter->element_end()){
				// getChildNodes() returns all types of DOMNodes including comments, text, etc., but we only want
				// to process element nodes
				//std::cout << "Create Object of type " << iter->getElementTag() << std::endl;
				object = newInstanceOfType(iter->getElementTag());
				if(!object) { std::cerr << "Object type " << iter->getElementTag() << " not recognized" << std::endl; iter++; continue; }
				//if(!property->isValidObject(object)) throw XMLParsingException("Unexpected object of type "+objectType+" found under "+name+" tag.",objElmt,__FILE__,__LINE__);
				objectsFound++;

				if(type==Property::ObjPtr) {
					if(objectsFound > 1){
						//throw XMLParsingException("Found multiple objects under "+name+" tag, but expected only one.",objElmt,__FILE__,__LINE__);
					}
					else{
						property->setValue(object);
					}
				} else {
					property->appendValue(object);
				}
				//object->_document = _document;	// Propagate _document ptr.
				object->updateFromXMLNode(*iter, versionNumber);
				iter++;
			}
				
			break; }

		// NOT RECOGNIZED
		default :
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;
			break;
		}
	}

	Array<AbstractProperty *> propertyArray = getPropertyArray();
	for(int i=0;i<_propertyTable.getSize();i++) {

		AbstractProperty *abstractProperty = propertyArray[i];

		// TYPE
		AbstractProperty::PropertyType type = abstractProperty->getPropertyType();	

		// NAME
		string name = abstractProperty->getName();
		if(_debugLevel>=4) {
			cout << "Object.updateFromXMLNode: ("<<getType()<<":"<<getName()<<") updating property " << name << endl;
		}

		SimTK::String valueString;
		SimTK::String lowerCaseValueString;
		SimTK::Xml::element_iterator iter;
		SimTK::Array_<SimTK::String> value;
		OpenSim::Array<bool> osimValue;

		// VALUE
		switch(type) {

		// Bool
		case(AbstractProperty::Bool) : {
			Property2<bool> *propertyBool = dynamic_cast<Property2<bool> *>(abstractProperty);
			iter= aNode.element_begin(name);
			if (iter == aNode.element_end()) break;	// Not found
			iter->getValueAs(valueString); // true/false
			lowerCaseValueString = valueString.toLower();
			propertyBool->setValue(lowerCaseValueString=="true"?true:false);
			//UpdateFromXMLNodeSimpleProperty<bool>(property, aNode, name);
			break; }
		// Int
		case(AbstractProperty::Int) :
			UpdateFromXMLNodeSimpleProperty2<int>(abstractProperty, aNode, name);
			break;
		// Double
		case(AbstractProperty::Dbl) : {
			Property2<double> *propertyDbl = dynamic_cast<Property2<double> *>(abstractProperty);
			iter= aNode.element_begin(name);
			if (iter == aNode.element_end()) continue;	// Not found
			iter->getValueAs(valueString); // special values
			lowerCaseValueString = valueString.toLower();
			if (lowerCaseValueString=="infinity")
				propertyDbl->setValue(SimTK::Infinity);
			else if (lowerCaseValueString=="-infinity")
				propertyDbl->setValue(-SimTK::Infinity);
			else if (lowerCaseValueString=="nan")
				propertyDbl->setValue(SimTK::NaN);
			else
				UpdateFromXMLNodeSimpleProperty2<double>(propertyDbl, aNode, name);
			break; }
		// Str
		case(AbstractProperty::Str) :
			UpdateFromXMLNodeSimpleProperty2<string>(abstractProperty, aNode, name);
			break;
		// BoolArray
		case(AbstractProperty::BoolArray) : {
			Property2< Array<bool> > *propertyBoolArray = dynamic_cast<Property2< Array<bool> > *>(abstractProperty);
			// Parse as a String array then map true/false to boolean values
			propertyBoolArray->setUseDefault(true);
			iter = aNode.element_begin(name);
			if (iter == aNode.element_end()) continue;	// Not found
			iter->getValueAs(value);
			//cout << value << endl;
			osimValue.setSize(value.size());
			for(unsigned i=0; i< value.size(); i++) osimValue[i]=(value[i]=="true");
			propertyBoolArray->setValue(osimValue);
			propertyBoolArray->setUseDefault(false);
			break; }
		// IntArray
		case(AbstractProperty::IntArray) :
			UpdateFromXMLNodeArrayProperty2<int>(abstractProperty,aNode,name);
			break;
		// DblArray
		case(AbstractProperty::DblArray) :
			UpdateFromXMLNodeArrayProperty2<double>(abstractProperty,aNode,name);
			break;
		case(AbstractProperty::DblVec3) :
			UpdateFromXMLNodeVec3Property2(abstractProperty,aNode,name);
			break;/*
		case(AbstractProperty::Transform) :
			UpdateFromXMLNodeArrayProperty2<double>(abstractProperty,aNode,name);
			break;*/
		// StrArray
		case(AbstractProperty::StrArray) :
			UpdateFromXMLNodeArrayProperty2<string>(abstractProperty,aNode,name);
			break;

		// Obj
		case(AbstractProperty::Obj) : {
			Property2<Object> *propertyObj = static_cast<Property2<Object> *>(abstractProperty);
			propertyObj->setUseDefault(true);
			Object &object = propertyObj->updValue();
			SimTK::Xml::element_iterator iter = aNode.element_begin(object.getType());
			if (iter == aNode.element_end()) continue;	// Not found
			if (propertyObj->getMatchName()){
					while(object.getName() != iter->getOptionalAttributeValueAs<std::string>("name", dName) &&
						 iter != aNode.element_end()){
							iter++;
				}
					if (iter != aNode.element_end())
						InitializeObjectFromXMLNode2(propertyObj, iter, &object, versionNumber);
					}
			else
				InitializeObjectFromXMLNode2(propertyObj, iter, &object, versionNumber);
			break; }

		// ObjArray AND ObjPtr (handled very similarly)
		case(AbstractProperty::ObjArray) : {
			Property2< ArrayPtrs<Object> > *propertyObjArray = dynamic_cast<Property2< ArrayPtrs<Object> > *>(abstractProperty);
			propertyObjArray->setUseDefault(true);
			ArrayPtrs<Object> &objArray = propertyObjArray->updValue();
			objArray.setSize(0);

			// GET ENCLOSING ELEMENT
			//DOMElement *elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			//if(elmt==NULL) {
			//	if (_debugLevel>=4) {
			//		cout<<"Object.updateFromXMLNode: ERR- failed to find element ";
			//		cout<<name<<endl;
			//	}
			//	break;
			//}

			// Call parseFileAttribute to take care of the case where a file attribute points to
			// an external XML file.  Essentially this will make elmt point to the top level
			// element of the other file
			//{
			//	DOMElement *refNode;
			//	XMLDocument *childDocument;
			//	parseFileAttribute(elmt, refNode, childDocument, elmt);
			//}

			// LOOP THROUGH CHILD NODES
			const SimTK::Xml::element_iterator& propElementIter =  aNode.element_begin(name);
			if (propElementIter==aNode.element_end()) break;
			Object *object = NULL;
			int objectsFound = 0;
			SimTK::Xml::element_iterator iter = propElementIter->element_begin();
			while(iter != propElementIter->element_end()){
				// getChildNodes() returns all types of DOMNodes including comments, text, etc., but we only want
				// to process element nodes
				//std::cout << "Create Object of type " << iter->getElementTag() << std::endl;
				object = newInstanceOfType(iter->getElementTag());
				if(!object) { std::cerr << "Object type " << iter->getElementTag() << " not recognized" << std::endl; iter++; continue; }
				//if(!property->isValidObject(object)) throw XMLParsingException("Unexpected object of type "+objectType+" found under "+name+" tag.",objElmt,__FILE__,__LINE__);
				objectsFound++;

				// add code to check if object is valid
				objArray.append(object);
				
				//object->_document = _document;	// Propagate _document ptr.
				object->updateFromXMLNode(*iter, versionNumber);
				iter++;
			}
			break; }
		case(AbstractProperty::ObjPtr) : {
			Property2<Object *> *propertyObjPtr = static_cast<Property2<Object *> *>(abstractProperty);
			propertyObjPtr->setUseDefault(true);

			// GET ENCLOSING ELEMENT
			//DOMElement *elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			//if(elmt==NULL) {
			//	if (_debugLevel>=4) {
			//		cout<<"Object.updateFromXMLNode: ERR- failed to find element ";
			//		cout<<name<<endl;
			//	}
			//	break;
			//}

			// Call parseFileAttribute to take care of the case where a file attribute points to
			// an external XML file.  Essentially this will make elmt point to the top level
			// element of the other file
			//{
			//	DOMElement *refNode;
			//	XMLDocument *childDocument;
			//	parseFileAttribute(elmt, refNode, childDocument, elmt);
			//}

			// LOOP THROUGH CHILD NODES
			const SimTK::Xml::element_iterator& propElementIter =  aNode.element_begin(name);
			if (propElementIter==aNode.element_end()) break;
			Object *object = NULL;
			int objectsFound = 0;
			SimTK::Xml::element_iterator iter = propElementIter->element_begin();
			while(iter != propElementIter->element_end()){
				// getChildNodes() returns all types of DOMNodes including comments, text, etc., but we only want
				// to process element nodes
				//std::cout << "Create Object of type " << iter->getElementTag() << std::endl;
				object = newInstanceOfType(iter->getElementTag());
				if(!object) { std::cerr << "Object type " << iter->getElementTag() << " not recognized" << std::endl; iter++; continue; }
				//if(!property->isValidObject(object)) throw XMLParsingException("Unexpected object of type "+objectType+" found under "+name+" tag.",objElmt,__FILE__,__LINE__);
				objectsFound++;

				if(objectsFound > 1){
					//throw XMLParsingException("Found multiple objects under "+name+" tag, but expected only one.",objElmt,__FILE__,__LINE__);
				}
				else{
					propertyObjPtr->setValue(object);
				}
				
				//object->_document = _document;	// Propagate _document ptr.
				object->updateFromXMLNode(*iter, versionNumber);
				iter++;
			}
				
			break; }

		// NOT RECOGNIZED
		default :
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;
			break;
		}
	}

	} catch (const Exception &ex) {
		// Important to catch exceptions here so we can restore current working directory...
		// And then we can rethrow the exception
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
 * Object::RegisterType().
 */
void Object::
updateDefaultObjectsFromXMLNode()
{
	
	// MUST BE ROOT ELEMENT
	if(_document==NULL) return;

	// GET DEFAULTS ELEMENT
	SimTK::Xml::element_iterator iterDefault = _document->getRootDataElement().element_begin("defaults");
	if (iterDefault==_document->getRootDataElement().element_end() || 
		!iterDefault->isValid()) return;	// No defaults, skip over

	SimTK::Array_<SimTK::Xml::Element> elts = iterDefault->getAllElements();
	for(unsigned it = 0; it < elts.size(); it++) {
		SimTK::String stg = elts[it].getElementTag();

		// GET DEFAULT OBJECT
		Object *defaultObject = _mapTypesToDefaultObjects[stg];
		if(defaultObject==NULL) continue;

		// GET ELEMENT
		const string &type = defaultObject->getType();
		SimTK::Xml::element_iterator iterDefaultType =iterDefault->element_begin(type);
		//DOMElement *elmt = XMLNode::GetFirstChildElementByTagName(defaultsElmt,type);
		if(iterDefaultType==iterDefault->element_end()) continue;

		// CONSTRUCT AND REGISTER DEFAULT OBJECT
		// Used to call a special copy method that took DOMElement* but 
		// that ended up causing XML to be parsed twice.  Got rid of that
		// copy method! - Eran, Feb/07
		Object *object = defaultObject->copy();
		object->updateFromXMLNode(*iterDefaultType, _document->getDocumentVersion());
		object->setName(DEFAULT_NAME);
		RegisterType(*object);
		_document->addDefaultObject(object);	// object will be owned by the _document
		//delete object;
	} 
}

//-----------------------------------------------------------------------------
// UPDATE XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update the XML node that represents this object.
 *
 * @param aParent Parent XML node of this object.  Sending in a parent node
 * allows an XML node to be generated for this object if it doesn't already
 * have one.  If the parent node is sent in as NULL and this object doesn't
 * already have an XML node, this object will become the root node for a
 * new XML document.  If this object already has an XML node associated with
 * it, no new nodes are ever generated and the parent node is not used.
 *
 * @param aParent Parent XML element.
 */
void Object::
updateXMLNode(SimTK::Xml::Element& aParent)
{
	// Handle non-inlined object
	if(!getInlined()) {
		// If object is not inlined we don't want to generate node in original document
		// Handle not-inlined objects first.
		if (!aParent.isValid()) {
			cout<<"Root node must be inlined"<<*this<<endl;
		} else {
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
	SimTK::Xml::Element myObjectElement(getType());
	myObjectElement.setAttributeValue("name", getName());
	aParent.insertNodeAfter(aParent.node_end(), myObjectElement);

	//SimTK::String elemAsString;
	//aParent.writeToString(elemAsString);

	// DEFAULT OBJECTS
	//updateDefaultObjectsXMLNode(aParent);
	if (_document) _document->writeDefaultObjects(myObjectElement);
	// LOOP THROUGH PROPERTIES
	for(int i=0;i<_propertySet.getSize();i++) {

		//_document->writeToString(elemAsString);

		Property *property = _propertySet.get(i);

		// Add comment if any
		if (!property->getComment().empty()) {
			myObjectElement.insertNodeAfter(myObjectElement.node_end(), SimTK::Xml::Comment(property->getComment()));
		}
		// TYPE
		Property::PropertyType type = property->getType();

		// NAME
		string name = property->getName();

		string stringValue="";
		// VALUE
		switch(type) {

		// Bool
		case(Property::Bool) :
			UpdateXMLNodeSimpleProperty<bool>(property, myObjectElement, name);
			break;
		// Int
		case(Property::Int) :
			UpdateXMLNodeSimpleProperty<int>(property, myObjectElement, name);
			break;
		// Dbl
		case(Property::Dbl) :
			if (SimTK::isFinite(property->getValueDbl()))
				UpdateXMLNodeSimpleProperty<double>(property, myObjectElement, name);
			else {
				if (property->getValueDbl() == SimTK::Infinity)
					stringValue="infinity";
				else if (property->getValueDbl() == -SimTK::Infinity)
					stringValue="-infinity";
				else if (SimTK::isNaN(property->getValueDbl()))
					stringValue="NaN";
				if(!property->getUseDefault()) {
					SimTK::Xml::Element elt(property->getName(), stringValue);
					myObjectElement.insertNodeAfter(myObjectElement.node_end(), elt);
				}
			} 
			break;
		// Str
		case(Property::Str) :
			UpdateXMLNodeSimpleProperty<string>(property, myObjectElement, name);
			break;
		// BoolArray
		case(Property::BoolArray) :
			UpdateXMLNodeArrayProperty<bool>(property,myObjectElement,name);
			break;
		// IntArray
		case(Property::IntArray) :
			UpdateXMLNodeArrayProperty<int>(property,myObjectElement,name);
			break;
		// DblArray
		case(Property::DblArray) :
			UpdateXMLNodeArrayProperty<double>(property,myObjectElement,name);
			break;
		// DblVec3
		case(Property::DblVec) :
			UpdateXMLNodeVec(property,myObjectElement,name);
			break;
		// Transform
		case(Property::Transform) :
			UpdateXMLNodeTransform(property,myObjectElement,name);
			break;
		// StrArray
		case(Property::StrArray) :
			UpdateXMLNodeArrayProperty<string>(property,myObjectElement,name);
			break;

		// Obj
		case(Property::Obj) : {
			PropertyObj *propObj = (PropertyObj*)property;
			Object &object = property->getValueObj();
			object.updateXMLNode(myObjectElement);
			/*
			if(propObj->getMatchName()) {
				
				// Find the first element with correct tag & name attribute
				string objName = object.getName();
				elmt = XMLNode::GetFirstChildElementByTagName(myObjectElement, object.getType(), &objName);
			} else {
				// Find the first element with correct tag (name not important)
				elmt = XMLNode::GetFirstChildElementByTagName(myObjectElement, object.getType());
			}

			if(!elmt && !property->getUseDefault()) {
				elmt = XMLNode::InsertNewElementWithComment(myObjectElement, object.getType(), object.getName(), property->getComment(), aNodeIndex);
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}

			if(elmt) {
				// If it's not inlined, hopefully calling updateXMLNode will be enough...
				// (it probably won't touch the referring element, only the offline document)
				if(object.getInlined()) object.setXMLNode(elmt);
				else object._refNode = elmt;
				object.updateXMLNode(myObjectElement);
			}*/
			break; }

		// ObjArray AND ObjPtr (handled very similarly)
		case(Property::ObjArray) :
		case(Property::ObjPtr) : {
				if(type==Property::ObjArray) {
						// Set all the XML nodes to NULL, and then update them all
						// in order, with index=0 so each new one is added to the end
						// of the list (more efficient than inserting each one into
						// the proper slot).
					SimTK::Xml::Element objectArrayElement(property->getName());
					myObjectElement.insertNodeAfter(myObjectElement.node_end(), objectArrayElement);
					   for(int j=0;j<property->getArraySize();j++)
						property->getValueObjPtr(j)->updateXMLNode(objectArrayElement);
				} else { // ObjPtr
					Object *object = property->getValueObjPtr();
					SimTK::Xml::Element objectBaseElement(property->getName());
					myObjectElement.insertNodeAfter(myObjectElement.node_end(), objectBaseElement);
					if(object) { // Add node for base classHEREHEREHERE
						object->updateXMLNode(objectBaseElement);
					}
				}
			} 
			break; 

		// NOT RECOGNIZED
		default :
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;
			break;
		}
	}

	Array<AbstractProperty *> propertyArray = getPropertyArray();
	for(int i=0;i<_propertyTable.getSize();i++) {

		//_document->writeToString(elemAsString);

		AbstractProperty *abstractProperty = propertyArray[i];

		// Add comment if any
		if (!abstractProperty->getComment().empty()) {
			myObjectElement.insertNodeAfter(myObjectElement.node_end(), SimTK::Xml::Comment(abstractProperty->getComment()));
		}
		// TYPE
		AbstractProperty::PropertyType type = abstractProperty->getPropertyType();

		// NAME
		string name = abstractProperty->getName();

		string stringValue="";
		
		// VALUE
		switch(type) {

		// Bool
		case(AbstractProperty::Bool) :
			UpdateXMLNodeSimpleProperty2<bool>(abstractProperty, myObjectElement, name);
			break;
		// Int
		case(AbstractProperty::Int) :
			UpdateXMLNodeSimpleProperty2<int>(abstractProperty, myObjectElement, name);
			break;
		// Dbl
		case(AbstractProperty::Dbl) : {
			Property2<double> *propertyDbl = dynamic_cast<Property2<double> *>(abstractProperty);
			if (SimTK::isFinite(propertyDbl->getValue()))
				UpdateXMLNodeSimpleProperty2<double>(propertyDbl, myObjectElement, name);
			else {
				if (propertyDbl->getValue() == SimTK::Infinity)
					stringValue="infinity";
				else if (propertyDbl->getValue() == -SimTK::Infinity)
					stringValue="-infinity";
				else if (SimTK::isNaN(propertyDbl->getValue()))
					stringValue="NaN";
				if(!propertyDbl->getUseDefault()) {
					SimTK::Xml::Element elt(propertyDbl->getName(), stringValue);
					myObjectElement.insertNodeAfter(myObjectElement.node_end(), elt);
				}
			} 
			break; }
		// Str
		case(AbstractProperty::Str) :
			UpdateXMLNodeSimpleProperty2<string>(abstractProperty, myObjectElement, name);
			break;
		// BoolArray
		case(AbstractProperty::BoolArray) :
			UpdateXMLNodeArrayProperty2<bool>(abstractProperty,myObjectElement,name);
			break;
		// IntArray
		case(AbstractProperty::IntArray) :
			UpdateXMLNodeArrayProperty2<int>(abstractProperty,myObjectElement,name);
			break;
		// DblArray
		case(AbstractProperty::DblArray) :
			UpdateXMLNodeArrayProperty2<double>(abstractProperty,myObjectElement,name);
			break;
		// DblVec3
		case(AbstractProperty::DblVec3) :
			UpdateXMLNodeVec3(abstractProperty,myObjectElement,name);
			break;/*
		// Transform
		case(Property::Transform) :
			UpdateXMLNodeTransform(property,myObjectElement,name);
			break;*/
		// StrArray
		case(AbstractProperty::StrArray) :
			UpdateXMLNodeArrayProperty2<string>(abstractProperty,myObjectElement,name);
			break;

		// Obj
		case(AbstractProperty::Obj) : {
			Property2<Object> *propertyObj = static_cast<Property2<Object> *>(abstractProperty);
			Object &object = propertyObj->updValue();
			object.updateXMLNode(myObjectElement);
			/*
			if(propObj->getMatchName()) {
				
				// Find the first element with correct tag & name attribute
				string objName = object.getName();
				elmt = XMLNode::GetFirstChildElementByTagName(myObjectElement, object.getType(), &objName);
			} else {
				// Find the first element with correct tag (name not important)
				elmt = XMLNode::GetFirstChildElementByTagName(myObjectElement, object.getType());
			}

			if(!elmt && !property->getUseDefault()) {
				elmt = XMLNode::InsertNewElementWithComment(myObjectElement, object.getType(), object.getName(), property->getComment(), aNodeIndex);
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}

			if(elmt) {
				// If it's not inlined, hopefully calling updateXMLNode will be enough...
				// (it probably won't touch the referring element, only the offline document)
				if(object.getInlined()) object.setXMLNode(elmt);
				else object._refNode = elmt;
				object.updateXMLNode(myObjectElement);
			}*/
			break; }

		// ObjArray AND ObjPtr (handled very similarly)
		case(AbstractProperty::ObjArray) : {
			Property2< ArrayPtrs<Object> > *propertyObjArray = dynamic_cast<Property2< ArrayPtrs<Object> > *>(abstractProperty);
			// Set all the XML nodes to NULL, and then update them all
			// in order, with index=0 so each new one is added to the end
			// of the list (more efficient than inserting each one into
			// the proper slot).
			SimTK::Xml::Element objectArrayElement(propertyObjArray->getName());
			myObjectElement.insertNodeAfter(myObjectElement.node_end(), objectArrayElement);
			ArrayPtrs<Object> &objArray = propertyObjArray->updValue();
			for(int j=0; j < objArray.getSize(); j++)
				objArray.get(j)->updateXMLNode(objectArrayElement);
			break; }
		case(AbstractProperty::ObjPtr) : {
			Property2<Object *> *propertyObjPtr = static_cast<Property2<Object *> *>(abstractProperty);
			Object *object = propertyObjPtr->updValue();
			SimTK::Xml::Element objectBaseElement(propertyObjPtr->getName());
			myObjectElement.insertNodeAfter(myObjectElement.node_end(), objectBaseElement);
			if(object) { // Add node for base classHEREHEREHERE
				object->updateXMLNode(objectBaseElement);
			}
			break; }

		// NOT RECOGNIZED
		default :
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;
			break;
		}
	}
}
//_____________________________________________________________________________
/**
 * Update the XML node for defaults object.
 */
void Object::
updateDefaultObjectsXMLNode(SimTK::Xml::Element& aParent)
{
	if (_document==NULL || !_document->hasDefaultObjects())
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
/**
 * Get the document for this object.
 *
 * @return Document set for this object.
 */
XMLDocument* Object::
getDocument() const
{
	return(_document);
}
//_____________________________________________________________________________
/**
 * Get the document's filename
 *
 * @return Document's filename for this object.
 */
string Object::
getDocumentFileName() const
{
	return _document ? _document->getFileName() : "";
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
	if (_document==NULL)
		_document = new XMLDocument();
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
	// In theory we might be able to re-use an existing _document node rather than deleting and re-creating one,
	// but currently if you try that you will get "ERROR- document already has root" from AppendNewElementWithComment, called by generateXMLDocument
	// TODO: use DOMDocument::adoptNode(DOMNode *source) to be able to switch owner documents...
	// For now it's safest to delete all XML structures
	XMLDocument* oldDocument=NULL;
	if (!_inlined){
		oldDocument = _document;
	}
	if (oldDocument)
		delete oldDocument;

	_inlined = aInlined;
	if(!_inlined) {
		_document = new XMLDocument();
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
	for(int i=0;i<_propertySet.getSize();i++) {

		Property *property = _propertySet.get(i);
		property->setUseDefault(aUseDefault);
		Property::PropertyType type = property->getType();

		// VALUE
		switch(type) {

		case(Property::Bool) :
		case(Property::Int) :
		case(Property::Dbl) :
		case(Property::Str) :
		case(Property::BoolArray) :
		case(Property::IntArray) :
		case(Property::DblArray) :
		case(Property::StrArray) :
		case(Property::DblVec) :
		case(Property::Transform) :
			break; // Nothing to do for the basic types

		// Obj
		case(Property::Obj) : {
			Object &object = property->getValueObj();
			object.setAllPropertiesUseDefault(aUseDefault);
			break;
		}

		// ObjArray
		case(Property::ObjArray) :
			for(int j=0;j<property->getArraySize();j++)
				property->getValueObjPtr(j)->setAllPropertiesUseDefault(aUseDefault);
			break;

		// ObjPtr
		case(Property::ObjPtr) : {
			Object *object = property->getValueObjPtr();
			if(object) object->setAllPropertiesUseDefault(aUseDefault);
			break;
		}

		// NOT RECOGNIZED
		default :
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;
			break;
		}
	}
	Array<AbstractProperty *> propertyArray = getPropertyArray();
	for(int i=0;i<_propertyTable.getSize();i++) {

		AbstractProperty *abstractProperty = propertyArray[i];
		abstractProperty->setUseDefault(aUseDefault);
		AbstractProperty::PropertyType type = abstractProperty->getPropertyType();

		// VALUE
		switch(type) {

		case(AbstractProperty::Bool) :
		case(AbstractProperty::Int) :
		case(AbstractProperty::Dbl) :
		case(AbstractProperty::Str) :
		case(AbstractProperty::BoolArray) :
		case(AbstractProperty::IntArray) :
		case(AbstractProperty::DblArray) :
		case(AbstractProperty::StrArray) :
		case(AbstractProperty::DblVec3) :
		case(AbstractProperty::Transform) :
			break; // Nothing to do for the basic types

		// Obj
		case(AbstractProperty::Obj) : {
			Property2<Object> *propertyObj = static_cast<Property2<Object> *>(abstractProperty);
			Object &object = propertyObj->updValue();
			object.setAllPropertiesUseDefault(aUseDefault);
			break;
		}

		// ObjArray
		case(AbstractProperty::ObjArray) : {
			Property2< ArrayPtrs<Object> > *propertyObjArray = dynamic_cast<Property2< ArrayPtrs<Object> > *>(abstractProperty);
			ArrayPtrs<Object> &objects = propertyObjArray->updValue();
			for(int j=0; j<objects.getSize(); j++)
				objects.get(j)->setAllPropertiesUseDefault(aUseDefault);
			break;
		}

		// ObjPtr
		case(AbstractProperty::ObjPtr) : {
			Property2<Object *> *propertyObjPtr = dynamic_cast<Property2<Object *> *>(abstractProperty);
			Object *object = propertyObjPtr->updValue();
			if(object) object->setAllPropertiesUseDefault(aUseDefault);
			break;
		}

		// NOT RECOGNIZED
		default :
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;
			break;
		}
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
print(const string &aFileName)
{
	// Temporarily change current directory so that inlined files are written to correct relative directory
	std::string savedCwd = IO::getCwd();
	IO::chDir(IO::getParentDirectory(aFileName));
	try {
		XMLDocument* oldDoc = NULL;
		if (_document != NULL){
			oldDoc = _document;
		}
		_document = new XMLDocument();
		if (oldDoc){
			_document->copyDefaultObjects(*oldDoc);
			delete oldDoc;
		}
		SimTK::Xml::Element e = _document->getRootElement(); 
		updateXMLNode(e);
	} catch (const Exception &ex) {
		// Important to catch exceptions here so we can restore current working directory...
		// And then we can rethrow the exception
		IO::chDir(savedCwd);
		throw(ex);
	}
	IO::chDir(savedCwd);
	if(_document==NULL) return false;
	_document->print(aFileName);
	return true;
}

//-----------------------------------------------------------------------------
// PRINT PROPERTY INFORMATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Print property information for registered classes.  This method will
 * print the comment field of a property to an output stream. Input is a
 * class name and property name.  If the property name is the empty string,
 * information for all properties in the class is printed.  If the class
 * name is empty, information in all properties of all registered classes
 * is printed.
 *
 * @param aOStream Output stream to which info is printed.
 * @param aClassNameDotPropertyName A string combining the class name
 * and property name. The two names should be seperated by a period
 * (ClassName.PropertyName).  If no property is specified, the information
 * for all properties in the class is printed.  If no class is specified,
 * the information for the properties of all registered classes is
 * printed.
 */
void Object::
PrintPropertyInfo(ostream &aOStream,
						const string &aClassNameDotPropertyName)
{
	// PARSE NAMES
	string compoundName = aClassNameDotPropertyName;

  	string::size_type delimPos = compoundName.find(".");
	string className = compoundName.substr(0,delimPos);
	string propertyName = "";
	if(delimPos!=string::npos) {
		propertyName = compoundName.substr(delimPos+1);
	}
	//cout<<"PrintPropertyInfo:  className="<<className<<" propName="<<propertyName<<endl;

	PrintPropertyInfo(aOStream,className,propertyName);
}
//_____________________________________________________________________________
/**
 * Print property information for registered classes.  This method will
 * print the comment field of a property to an output stream. Input is a
 * class name and property name.  If the property name is the empty string,
 * information for all properties in the class is printed.  If the class
 * name is empty, information in all properties of all registered classes
 * is printed.
 *
 * @param aOStream Output stream to which info is printed.
 * @param aClassName Class for which to print properties.  If an empty
 * string, the property information for all registered classes is
 * printed.
 * @pram aPropertyName Property for which to print information.  If an
 * empty string, information for all properties in the specified class
 * is printed.
 */
void Object::
PrintPropertyInfo(ostream &aOStream,
						const string &aClassName,const string &aPropertyName)
{
	// NO CLASS
	if(aClassName=="") {
		int size = _Types.getSize();
		aOStream<<"REGISTERED CLASSES ("<<size<<")\n";
		Object *obj;
		for(int i=0;i<size;i++) {
			obj = _Types.get(i);
			if(obj==NULL) continue;
			aOStream<<obj->getType()<<endl;
		}
		aOStream<<"\n\nUse '-PropertyInfo ClassName' to list the properties of a particular class.\n\n";
		return;
	}

	// FIND CLASS
	Object* object = _mapTypesToDefaultObjects[aClassName];
	if(object==NULL) {
		aOStream<<"\nA class with the name '"<<aClassName<<"' was not found.\n";
		aOStream<<"\nUse '-PropertyInfo' without specifying a class name to print a listing of all registered classes.\n";
		return;
	}

	// NO PROPERTY
	PropertySet propertySet = object->getPropertySet();
	Property *property;
	AbstractProperty *abstractProperty;
	if((aPropertyName=="")||(aPropertyName=="*")) {
		int propertySetSize = propertySet.getSize();
		int propertyTableSize = object->_propertyTable.getSize();
		int size = propertySetSize + propertyTableSize;
		aOStream<<"\nPROPERTIES FOR "<<aClassName<<" ("<<size<<")\n";
		string comment;
		int i;
		for(i=0;i<propertySetSize;i++) {
			property = object->_propertySet.get(i);
			if(property==NULL) continue;
			if(aPropertyName=="") {
				aOStream<<i+1<<". "<<property->getName()<<endl;
			} else {
				aOStream<<"\n"<<i+1<<". "<<property->getName()<<"\n";
				comment = property->getComment();
				if(!comment.empty()) {
					string formattedComment = IO::formatText(comment,"\t",80);
					aOStream<<"\t"<<formattedComment<<"\n";
				}
			}
		}
		Array<AbstractProperty *> propertyArray = object->getPropertyArray();
		for(;i<size;i++) {
			abstractProperty = propertyArray[i];
			if(abstractProperty==NULL) continue;
			if(aPropertyName=="") {
				aOStream<<i+1<<". "<<abstractProperty->getName()<<endl;
			} else {
				aOStream<<"\n"<<i+1<<". "<<abstractProperty->getName()<<"\n";
				comment = abstractProperty->getComment();
				if(!comment.empty()) {
					string formattedComment = IO::formatText(comment,"\t",80);
					aOStream<<"\t"<<formattedComment<<"\n";
				}
			}
		}
		aOStream<<"\n\nUse '-PropertyInfo ClassName.PropertyName' to print info for a particular property.\n";
		if(aPropertyName!="*") {
			aOStream<<"Use '-PropertyInfo ClassName.*' to print info for all properties in a class.\n";
		}
		return;
	}

	// FIND PROPERTY
	try {
		property = propertySet.get(aPropertyName);
		// OUTPUT
		//aOStream<<"\nPROPERTY INFO FOR "<<aClassName<<"\n";
		aOStream<<endl<<aClassName<<"."<<aPropertyName<<"\n"<<property->getComment()<<"\n";
	} catch(...) {
		try {
			abstractProperty = object->_propertyTable.getPropertyPtr(aPropertyName);
			// OUTPUT
			//aOStream<<"\nPROPERTY INFO FOR "<<aClassName<<"\n";
			aOStream<<endl<<aClassName<<"."<<aPropertyName<<"\n"<<abstractProperty->getComment()<<"\n";
		} catch (...) {
			aOStream<<"\nPrintPropertyInfo: no property with the name "<<aPropertyName;
			aOStream<<" was found in class "<<aClassName<<".\n";
			aOStream<<"Omit the property name to get a listing of all properties in a class.\n";
			return;
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
	try{
		XMLDocument *doc = new XMLDocument(aFileName);
		string rootName = doc->getRootTag();
		bool newFormat=false;
		if (rootName == "OpenSimDocument"){	// New format, get child node instead
			rootName = doc->getRootElement().element_begin()->getElementTag();
			newFormat=true;
		}
		Object* newObject = newInstanceOfType(rootName);
		if(!newObject) throw Exception("Unrecognized XML element '"+rootName+"' and root of file '"+aFileName+"'",__FILE__,__LINE__);
		newObject->_document=doc;
		if (newFormat)
			newObject->updateFromXMLNode(*doc->getRootElement().element_begin(), doc->getDocumentVersion());
		else { 
			SimTK::Xml::Element e = doc->getRootElement();
			newObject->updateFromXMLNode(e, 10500);
		}
		return (newObject);
	}
	catch(Exception &x) {
		x.print(cout);
		return 0;
	}
	catch(...){	// Document couldn't be opened, or something went really bad
		return 0;
	}

}

/**
 * Create a new instance of the type indicated by aType.
 * The instance is initialized to the default Object of corresponding type.
 */
Object* Object::
newInstanceOfType(const std::string &aType)
{
	stringsToObjects::const_iterator find_Iter = _mapTypesToDefaultObjects.find(aType);
	Object* newObj=0;
	if (find_Iter != _mapTypesToDefaultObjects.end()){
		Object* defaultObject = find_Iter->second;
		// This object has proper type;
		newObj = defaultObject->copy();
	}
	if (newObj==0){
		cout << "Cant create a new instance of object type (" << aType << ") likely a typo in xml/osim file" << endl;
	}
	return (newObj);
}

/**
 * getRegisteredTypenames is a utility to retrieve all the typenames registered so far.
 * This is done by traversing the registered objects map, so only concrete classes are dealt with.
 * The result returned in rTypeNames should not be cached while more dlls are loaded as they get stale
 * instead the list should be constructed whenever in doubt
 */
void Object::
getRegisteredTypenames(Array<std::string>& rTypeNames)
{
	stringsToObjects::const_iterator find_Iter = _mapTypesToDefaultObjects.begin();
	while (find_Iter != _mapTypesToDefaultObjects.end()){
		std::string nextTypeName = find_Iter->first;
		if (_deprecatedTypes.findIndex(nextTypeName)==-1)
			rTypeNames.append(nextTypeName);
		find_Iter++;
	}
}


void Object::updateFromXMLDocument()
{
	assert(_document!= 0);
	
	SimTK::Xml::Element e = _document->getRootDataElement(); 
	updateFromXMLNode(e, _document->getDocumentVersion());
}

void Object::
addProperty(AbstractProperty &abstractProperty)
{
	_propertyTable.addProperty(abstractProperty);
}

std::string Object::
getPropertyType(const std::string &name) const
{
	return _propertyTable.getPropertyType(name);
}

std::string Object::
getPropertyComment(const std::string &name) const
{
	return _propertyTable.getPropertyComment(name);
}

Array<AbstractProperty *> Object::
getPropertyArray()
{
	return _propertyTable.getArray();
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
    
static osimCommonInstantiator instantiator; 
/// @endcond
