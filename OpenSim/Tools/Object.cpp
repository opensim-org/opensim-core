// Object.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


//============================================================================
// INCLUDES
//============================================================================
#include <fstream>
#include "Object.h"
#include "Exception.h"
#include "Property.h"
#include "PropertyInt.h"
#include "PropertyStr.h"
#include "IO.h"


using namespace OpenSim;
using namespace std;

//=============================================================================
// STATICS
//=============================================================================
ArrayPtrs<Object> Object::_Types;
Array<XMLCh *> Object::_typeNames(0);

stringsToObjects Object::_mapTypesToDefaultObjects;
defaultsReadFromFile	Object::_defaultsReadFromFile;
bool Object::_serializeAllDefaults=false;

#include <vector>
#include <algorithm>  // Include algorithms

static vector<std::string> recognizedTypes;

//============================================================================
// CONSTANTS
//============================================================================
const string Object::DEFAULT_NAME(ObjectDEFAULT_NAME);
static const bool Object_DEBUG = false;

void stripExtraWhiteSpace(std::string &aBuffer)
{
	int front = aBuffer.find_first_not_of(" \t\r\n");
	if(front==-1) {
		aBuffer = "";
	} else if (front > 0) {
		aBuffer.erase(0, front);
	}

	std::size_t back = aBuffer.find_last_not_of(" \t\r\n");
	if (back < aBuffer.size() - 1)
		aBuffer.erase(back + 1);
}


//=============================================================================
// CONSTRUCTOR(S)
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Object::~Object()
{
	if(_document!=NULL) { delete _document;  _document=NULL; }
	if (_observable!=0) { delete _observable; _observable=0; }
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
Object::Object(const string &aFileName)
{
	// INITIALIZATION
	setNull();

	// CREATE DOCUMENT
	{
		// Check file exists before trying to parse it. Is there a faster way to do this?
		// This maybe slower than we like but definitely faster than 
		// going all the way down to the parser to throw an exception for null document!
		// -Ayman 8/06
		ifstream fileExists(aFileName.c_str(), ios_base::in);
		if (fileExists.good()==false){
			string msg =
				"Object: ERR- Could not open file " + aFileName+ ". It may not exist or you don't have permission to read it.";
			throw Exception(msg,__FILE__,__LINE__);
		}
		fileExists.close();	
	}

	_document = new XMLDocument(aFileName);

	//try {
	// CONSTRUCT BASED ON ROOT ELEMENT
	DOMDocument *doc = _document->getDOMDocument();
	if(doc==0) {
		string msg =
			"Object: ERR- Failed to construct object from file " + aFileName;
		throw Exception(msg,__FILE__,__LINE__);
	}

	// GET DOCUMENT ELEMENT
	_node = doc->getDocumentElement();

	// Build array of type names to avoid calling xmlTranscode repeatedly with 
	// The side effect of allocating and releasing buffers. 
	// This should've been done right after registration however this seems to crash
	// (probably because XML initialization is not done until we try to read an object from a file).
	buildTypeNamesTable();
	// UPDATE OBJECT
	updateFromXMLNode();
	//}
	//catch(Exception &x) {
		//x.print(cout);
	//}

}
//_____________________________________________________________________________
/**
 * Construct an object from a document.
 *
 * The document is copied and this object, including its derived classes,
 * are constructed based on the nodes within the document.
 */
Object::Object(const XMLDocument *aDocument)
{
	setNull();

	// CHECK DOCUMENT
	if(aDocument==NULL) {
		cout<<"Object(aDocument): ERROR- document was null.\n";
		return;
	}

	// COPY DOCUMENT
	_document = new XMLDocument(*aDocument);

	// CONSTRUCT BASED ON ROOT ELEMENT
	DOMDocument *doc = _document->getDOMDocument();
	_node = doc->getDocumentElement();
	updateFromXMLNode();
}
//_____________________________________________________________________________
/**
 * Construct an object from an DOMElement.
 */
Object::Object(DOMElement *aElement)
{
	setNull();

	// NODE
	_node = aElement;
	updateFromXMLNode();
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
 * 3) A call to generateDocument().
 * This method generates an XML document for the Object from scratch.
 * Only the essential document nodes are created (that is, nodes that
 * correspond directly to member variables.).
 *
 * @param aObject Object to be copied.
 * @see Object(const XMLDocument *aDocument)
 * @see Object(const char *aFileName)
 * @see generateDocument()
 */
Object::Object(const Object &aObject)
{
	setNull();

	// COPY TYPE AND NAME
	*this = aObject;
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
//_____________________________________________________________________________
/**
 * Copy this object and modify the copy so that it is consistent
 * with a specified XML element node.
 *
 * The copy is constructed by first using the contructor for the DOMElement
 * in order to establish the relationship of the control with the
 * XML node.  Then, the assignment operator is used to set all member variables
 * of the copy to the values of this object.  Finally, the data members of
 * the copy are updated from the DOMElment using updateObject().
 *
 * @param aElement XML element. 
 * @return Pointer to a copy of this object.
 */
Object* Object::
copy(DOMElement *aElement) const
{
	// ESTABLISH RELATIONSHIP WITH XML NODE
	Object *object = new Object(aElement);

	// ASSIGNMENT OPERATOR
	*object = *this;

	// UPDATE BASED ON NODE
	object->updateFromXMLNode();

	return(object);
}

//_____________________________________________________________________________
/**
 * Build table of Type names to avoid allocating and deallocating
 * memory for XML tags repeatedly.
 */
void Object::
buildTypeNamesTable()
{
	// Fill up the _typeNames array for quick indexing
	if (_typeNames.getSize() != _Types.getSize()){
		for(int i=0;i<_Types.getSize();i++) {
			Object *object = _Types.get(i);
			string objType = object->getType();
			XMLCh *tagName = XMLString::transcode(objType.c_str());
			_typeNames.append(tagName);
			recognizedTypes.push_back(objType);
		}
		sort(recognizedTypes.begin(), recognizedTypes.end());
	}
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
	_node = NULL;
	_refNode = NULL;
	_inLined = true;
	_propertySet.clear();
	_description = "";

	_observable=0;
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
	return(true);
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
 * @return True if this property is less than the other, false otherwise.
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
	if(_type.size()>NAME_LENGTH) _type.resize(NAME_LENGTH);
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
 *	3)	virtual Object* copy(DOMElement*) const,
 *	4)	<class>& operator=() (where the class name is substituted for <class>),
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
	// Keep track if the object being registered originated from a file vs. programmatically
	// for future use in the deserialization code.
	if (aObject._node!= 0)
		_defaultsReadFromFile[aObject.getType()] = true;
	else
		_defaultsReadFromFile[aObject.getType()] = false;

	// REPLACE IF A MATCHING TYPE IS ALREADY REGISTERED
	int i;
	for(i=0;i<_Types.getSize();i++) {
		Object *object = _Types.get(i);
		if(object->getType() == type) {
			if(Object_DEBUG) {
				cout<<"Object.RegisterType: replacing registered object of type ";
				cout<<type;
				cout<<"\n\twith a new default object of the same type."<<endl;
			}
			_Types.set(i,aObject.copy());
			_Types.get(i)->setName(DEFAULT_NAME);
			_mapTypesToDefaultObjects[aObject.getType()]= aObject.copy();
			return;
		} 
	}

	// APPEND
	Object *defaultObj = aObject.copy();
	_Types.append(defaultObj);
	_mapTypesToDefaultObjects[aObject.getType()]= defaultObj;
	_Types.getLast()->setName(DEFAULT_NAME);//0x00c067d8, 0x003b84f8
}


//=============================================================================
// XML NEW
//=============================================================================
//-----------------------------------------------------------------------------
// DEFAULT OBJECTS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Determine whether a specified object is a valid or recognized type
 * for this object.
 *
 * This method is used during XML serialization to determine whether or not
 * objects in the type registry (@see Object::RegisterType()) should be
 * written to a header element containing the default objects for this
 * object.  An object is generally a valid type if that object could be a
 * member or sub-member of this object.  The header containing the default
 * objects has the tag name "defaults".
 *
 * The default elements allow a user to see all the available properties for
 * an object of a particular type and also allow a user to specify default
 * values for any member objects.
 *
 * The implementation of this method in Object always returns true.
 * This results in every object in the type registery being written to
 * the defaults header.  To be more restrictive about which objects are
 * valid (and written to the defaults header), this method should be
 * overridden by derived classes.  For example, class ActuatorSet might
 * override this method to allow only object derived from Actuator to be
 * valid.
 *
 * @param aObject Object to be tested as valid or invalid default type
 * for this object.
 * @see updateFromXMLNode()
 * @see updateXMLNode()
 * @see RegisterType()
 */
bool Object::
isValidDefaultType(const Object *aObject) const
{
	return(true);
}

//-----------------------------------------------------------------------------
// UPDATE OBJECT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 * Added parameter sourceNode to handle external documents
 * For non-inlined objects the root of the childDocument is passed in else NULL.
 */
void Object::
updateFromXMLNode()
{
	if(_node==NULL) return;
	if(_type=="Object") return;

	// NAME 
	char *str = XMLNode::GetAttribute(_node,"name");
	string name = str;
	setName(name);
	if(str!=NULL) delete[] str;

	// UPDATE DEFAULT OBJECTS
	updateDefaultObjectsFromXMLNode();

	// LOOP THROUGH PROPERTIES
	DOMElement *elmt;
	for(int i=0;i<_propertySet.getSize();i++) {

		Property *property = _propertySet.get(i);

		// TYPE
		Property::PropertyType type = property->getType();

		// NAME
		string name = property->getName();
		if(Object_DEBUG) {
			cout << "Object.updateFromXMLNode: updating property " << name << endl;
		}


		//----------- TRY BLOCK FOR PROPERTIES ------------
		try {

		// VALUE
		switch(type) {

		// Bool
		case(Property::Bool) : {
			property->setUseDefault(true);
			bool value;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt!=NULL) {
				value = XMLNode::GetBool(elmt);
				property->setValue(value);
				property->setUseDefault(false);
			}
			break; }

		// Int
		case(Property::Int) : {
			property->setUseDefault(true);
			int value;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt!=NULL) {
				value = XMLNode::GetInt(elmt);
				property->setValue(value);
				property->setUseDefault(false);
			}
			break; }

		// Dbl
		case(Property::Dbl) : {
			property->setUseDefault(true);
			double value;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			// The following if statement prevents an exception from being thrown
			// in GetDbl.  It is also computationally much faster.
			if(elmt!=NULL) {
				value = XMLNode::GetDbl(elmt);
				property->setValue(value);
				property->setUseDefault(false);
			}
			break; }

		// Str
		case(Property::Str) : {
			property->setUseDefault(true);
			// Did code transformation to avoid trying to parse elmt
			// if it's known to be NULL to avoid exception throwing overhead.
			// -Ayman 8/06
			char *value=0;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if (elmt != 0)
				value = XMLNode::GetStr(elmt);
			if(value!=0) {
				string valueStr = value;
				stripExtraWhiteSpace(valueStr);
				property->setValue(valueStr);
				property->setUseDefault(false);
				delete[] value;
			}
			break; }

		// Obj
		case(Property::Obj) : {
			property->setUseDefault(true);
			Object &object = property->getValueObj();
			elmt = object.getXMLNode();

			//-----------Begin inline support---------------------------
			// Collect inlining attributes
			bool inLinedObject = true;
			DOMElement *refNode;
			XMLDocument *childDocument;
			if (elmt){
				char *fileAttrib = XMLNode::GetAttribute(elmt, "file");
				if (fileAttrib!= NULL && strlen(fileAttrib) > 0){
					// Change _node to refer to the root of the external file
					refNode = elmt;
					childDocument = new XMLDocument(fileAttrib);
					elmt = childDocument->getDOMDocument()->getDocumentElement();
					inLinedObject = false;
				}
				if(fileAttrib!=NULL) delete[] fileAttrib;
			}
			//-----------End inline support---------------------

			// OBJECT ALREAD HAS AN ASSOCIATED XML NODE
			if(elmt!=NULL) {
				object.updateFromXMLNode();

			// NEED TO CONSTRUCT BASED ON NODE
			} else {

				// VARIABLES
				XMLCh *tagName;
				DOMElement *objNode = NULL;
				DOMNodeList *list;

				// GET XML NODE
				string objType = object.getType();
				tagName = XMLString::transcode(objType.c_str());
				list = _node->getElementsByTagName(tagName);
				if(tagName!=NULL) delete[] tagName;

				// CHECK NAME
				string objName = object.getName();
				unsigned int listLength = list->getLength();
				for(unsigned int j=0;j<listLength;j++) {

					// GET ELEMENT
					elmt = (DOMElement*) list->item(j);
					if(elmt==NULL) continue;
					// Make sure this is not the default
					DOMNode *parent = elmt->getParentNode();
					string parentName = transcode(parent->getNodeName());
					// Add check for _type so that only nodes with proper parent are used.
					// A more robust solution is traversing only immediate children of _node
					// outside the loop.
					if (parentName=="defaults" || parentName!=_type) {
						continue;
					}
					// NAME ATTRIBUTE
					char *elmtName =
						XMLNode::GetAttribute(elmt,"name");
					if(Object_DEBUG) cout<<"\nFound element "<<elmtName<<endl;
					if(objName == elmtName) {
						objNode = elmt;
						delete[] elmtName;
						break;
					}

					// CLEAN UP
					delete[] elmtName;
				}

				// WAS A NODE NOT FOUND?
				if(objNode==NULL) {
					if (Object_DEBUG) {
						cout<<"Object.updateFromXMLNode: ERROR- could not find node ";
						cout<<objName<<" of type "<<objType<<"."<<endl;
					}
					break;
				}

				property->setUseDefault(false);
				// CONSTRUCT TEMPORARY OBJECT BASED ON NODE
				//Object *objTmp = object.copy(objNode);
				object.setXMLNode(objNode);
				//set default values
				object.updateFromXMLNode();

				// USE EQUALITY OPERATOR
				//object = (*objTmp);


				// Set inlining attributes on final object
				if (!inLinedObject){
					object._inLined = inLinedObject;
					object._refNode = refNode;
					object._document = childDocument;
				}

				// SET THE XML NODE
				//object.setXMLNode(objTmp->getXMLNode());

				// CLEAN UP
				//delete objTmp;

			}
			break; }

		// BoolArray
		case(Property::BoolArray) : {
			property->setUseDefault(true);
			int n=0;
			bool *value=NULL;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt!=NULL) {
				property->setUseDefault(false);
				n = XMLNode::GetBoolArray(elmt,value);
				property->setValue(n,value);
				if(n>0) delete[] value;
			}
			break; }

		// IntArray
		case(Property::IntArray) : {
			property->setUseDefault(true);
			int n=0;
			int *value=NULL;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt!=NULL) {
				property->setUseDefault(false);
				n = XMLNode::GetIntArray(elmt,value);
				// Moved setting and cleanup inside the if block so that values set by constructor are kept 
				// if property is not specified in the xml file
				property->setValue(n,value);
				if(n>0) delete[] value;
			}
			break; }

		// DblArray
		case(Property::DblArray) : {
			property->setUseDefault(true);	// Indicate not read from file
			int n=0;
			double *value=NULL;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt!=NULL) {
				property->setUseDefault(false);
				n = XMLNode::GetDblArray(elmt,value);
				// Moved setting and cleanup inside the if block so that values set by constructor are kept 
				// if property is not specified in the xml file
				property->setValue(n,value);
				if(n>0) delete[] value;
			}
			break; }

		// StrArray
		case(Property::StrArray) : {
			property->setUseDefault(true);
			int n=0;
			string *value=NULL;
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt!=NULL) {
				property->setUseDefault(false);
				n = XMLNode::GetStrArray(elmt,value);
				// Moved setting and cleanup inside the if block so that values set by constructor are kept 
				// if property is not specified in the xml file
				property->setValue(n,value);
				if(n>0) delete[] value;
			}
			break; }

		// ObjArray
		case(Property::ObjArray) : {
			property->setUseDefault(true);

			// GET ENCLOSING ELEMENT
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt==NULL) {
				if (Object_DEBUG) {
					cout<<"Object.updateFromXMLNode: ERR- failed to find element ";
					cout<<name<<endl;
				}
				break;
			}

			// CLEAR EXISTING OBJECT ARRAY
			// Eran: Moved after elmt check above so that values set by constructor are kept if
			// property is not specified in the xml file
			ArrayPtrs<Object> &objArray = property->getValueObjArray();
			objArray.setSize(0);

			// VARIABLES
			Object *defaultObject,*object;
			{
				// If top element has a "file attribute", the document needs to be opened
				// and elmt must be switched to the root of that document
				char *fileAttrib = XMLNode::GetAttribute(elmt, "file");
				bool inLinedObject = true;
				DOMElement *refNode;
				XMLDocument *childDocument;
				if ((fileAttrib!=NULL) && (strlen(fileAttrib)>0)){
					// Change _node to refer to the root of the external file
					refNode = elmt;
					childDocument = new XMLDocument(fileAttrib);
					elmt = childDocument->getDOMDocument()->getDocumentElement();
					inLinedObject = false;
				}
			}


			// LOOP THROUGH SUPPORTED OBJECT TYPES
			DOMNodeList *list = elmt->getChildNodes();
			unsigned int listLength = list->getLength();
			for(unsigned int j=0;j<listLength;j++) {
				// getChildNodes() returns all types of DOMNodes including comments, text, etc., but we only want
				// to process element nodes
				if (!list->item(j) || (list->item(j)->getNodeType() != DOMNode::ELEMENT_NODE)) continue;
				DOMElement *objElmt = (DOMElement*) list->item(j);

				const XMLCh *objType = objElmt->getTagName();
				string objectType = transcodeAndTrim(objType);

				if ( find(recognizedTypes.begin(), recognizedTypes.end(), objectType)== recognizedTypes.end()){
					continue;
				}
				defaultObject = _mapTypesToDefaultObjects[objectType];

					// If object is from non-inlined, detect it and set attributes
					// However we need to do that on the finalized object as copying
					// does not keep track of XML related issues
					//-----------Begin inline support---------------------------
					// Collect inlining attributes
					char *fileAttrib = XMLNode::GetAttribute(objElmt, "file");
					bool inLinedObject = true;
					DOMElement *refNode;
					XMLDocument *childDocument;
					if ((fileAttrib!=NULL) && (strlen(fileAttrib)>0)){
						// Change _node to refer to the root of the external file
						refNode = objElmt;
						childDocument = new XMLDocument(fileAttrib);
						objElmt = childDocument->getDOMDocument()->getDocumentElement();
						inLinedObject = false;
					}
					if(fileAttrib!=NULL) delete[] fileAttrib;
					//-----------End inline support---------------------
					// CHECK THAT THE ELEMENT IS AN IMMEDIATE CHILD
					DOMNode *parent = objElmt->getParentNode();
					if( (parent!=elmt) && (parent!=NULL) && (_node!=NULL) &&
						(parent->getOwnerDocument()==_node->getOwnerDocument()) ) {
						if(Object_DEBUG) {
							string elmtName = transcode(objElmt->getNodeName());
							string parentName = transcode(parent->getNodeName());
							string nodeName = transcode(elmt->getNodeName());
							cout<<"Object.updateFromXMLNode: "<<elmtName;
							cout<<" is a child of "<<parentName<<", not of ";
							cout<<nodeName<<endl;
						}
						continue;
					}
					
					property->setUseDefault(false);

					// CONSTRUCT THE OBJECT BASED ON THE ELEMENT
					object = defaultObject->copy(objElmt);

					// Set inlining attributes on final object
					if (!inLinedObject){
						object->_inLined = inLinedObject;
						object->_refNode = refNode;
						object->_document = childDocument;
					}

					// ADD
					if(object!=NULL) {
						objArray.append(object);
					}
				}
				
			break; }

		// NOT RECOGNIZED
		default : {
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<endl;

			break; }
		}

		//-----------------------------------------------------
		} catch(Exception x) {
			if(Object_DEBUG) x.print(cout);
		}
		//----------- END TRY BLOCK FOR PROPERTIES ------------
	}
}

//-----------------------------------------------------------------------------
// UPDATE DEFAULT OBJECTS FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update the registerred default objects based on an object's XML node.
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
	if(_node!=_document->getDOMDocument()->getDocumentElement()) return;

	// VARIABLES
	DOMElement *defaultsElmt,*elmt;
	Object *defaultObject,*object;

	// GET DEFAULTS ELEMENT
	defaultsElmt = XMLNode::GetFirstChildElementByTagName(_node,"defaults");
	if(defaultsElmt==NULL) return;

	// LOOP THROUGH SUPPORTED OBJECT TYPES
	for(int i=0;i<_Types.getSize();i++) {

		// GET DEFAULT OBJECT
		defaultObject = _Types.get(i);
		if(defaultObject==NULL) continue;
		if(!isValidDefaultType(defaultObject)) continue; // unused

		// GET ELEMENT
		const string &type = defaultObject->getType();
		elmt = XMLNode::GetFirstChildElementByTagName(defaultsElmt,type);
		if(elmt==NULL) continue;

		// CHECK THAT THE ELEMENT IS AN IMMEDIATE CHILD
		DOMNode *parent = elmt->getParentNode();
		if(parent != defaultsElmt) {
			if(Object_DEBUG) {
				string elmtName = transcode(elmt->getNodeName());
				string parentName = transcode(parent->getNodeName());
				string nodeName = transcode(_node->getNodeName());
				cout<<"Object.updateFromXMLNode: "<<elmtName;
				cout<<" is a child of "<<parentName<<", not of ";
				cout<<nodeName<<endl;
			}
			continue;
		}

		// CONSTRUCT AND REGISTER DEFAULT OBJECT
		object = defaultObject->copy(elmt);
		if(object!=NULL) {
			object->setName(DEFAULT_NAME);
			RegisterType(*object);
			delete object;
		}
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
updateXMLNode(DOMElement *aParent)
{
	// If object is not inlined we don't want to generate node in original document
	// Handle not-inlined objects first.
	if (aParent==NULL && !getInlined()) {
		cout<<"Root node must be inlined"<<*this<<endl;
	}

	// Can we make this more efficient than recreating the node again?
	// We can possibly check when setInlined() is invoked if we need to do it or not
	if (!getInlined() && aParent){
		// Create a new document and write object to it
		const char *offLineFileName = getDocument()->getFileName();
		// The problem is that generateChildXMLDocument makes a root which allows print
		// to do its job but root is duplicated. If we don't create the node then generateDocument
		// is invoked which messes up the whole _childDocument mechanism as _document is overwritten.
		_inLined=true;
		print(offLineFileName);
		_inLined=false;
		
		if (!_refNode){
			_refNode = XMLNode::AppendNewElementWithComment(aParent, getType(),getName());
			XMLNode::SetAttribute(_refNode,"file",offLineFileName);
		}
		return;
	}

	// GENERATE XML NODE?
	if(_node==NULL) {
		if(Object_DEBUG) cout<<"Generating XML node for "<<*this<<endl;
		generateXMLNode(aParent);
	}

	// CHECK THAT IT IS AN ELEMENT NODE
	if(_node->getNodeType()!=DOMNode::ELEMENT_NODE) return;


	// NAME
	XMLNode::SetAttribute(_node,"name",getName());

	// DEFAULT OBJECTS
	DOMElement *elmt;
	string defaultsTag = "defaults";
	elmt = XMLNode::GetFirstChildElementByTagName(_node,defaultsTag);
	// Not root element- remove defaults
	if((aParent!=NULL) && (elmt!=NULL)) {
		DOMNode *defaultsParent = elmt->getParentNode();
		defaultsParent->removeChild(elmt);
	// Root element- write valid defaults
	} else if(aParent==NULL) {
		bool createdNewElement = false;
		if(elmt==NULL) {
			elmt = XMLNode::AppendNewElementWithComment(_node,defaultsTag);
			createdNewElement = true;
		}
		XMLNode::RemoveChildren(elmt);
		for(int i=0;i<_Types.getSize();i++) {
			Object *defaultObject = _Types.get(i);
			if( isValidDefaultType(defaultObject) && 
				(Object::getSerializeAllDefaults() || _defaultsReadFromFile[defaultObject->getType()])) {
				defaultObject->setXMLNode(NULL);
				defaultObject->updateXMLNode(elmt);
			}
		}
		// If it will end up an empty <defaults/> tag then we just get rid of it
		if(createdNewElement && !elmt->hasChildNodes()) {
			_node->removeChild(elmt);
			// can we delete it?? it seems to be crashing...
			//delete elmt;
			elmt = NULL;
		}
	}

	// LOOP THROUGH PROPERTIES
	for(int i=0;i<_propertySet.getSize();i++) {

		Property *property = _propertySet.get(i);

		// TYPE
		Property::PropertyType type = property->getType();

		// NAME
		string name = property->getName();

		// VALUE
		switch(type) {

		// Bool
		case(Property::Bool) : {
			bool &value = property->getValueBool();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt = XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetBoolArray(elmt,1,&value);
			break; }

		// Int
		case(Property::Int) : {
			int &value = property->getValueInt();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetIntArray(elmt,1,&value);
			break; }

		// Dbl
		case(Property::Dbl) : {
			double &value = property->getValueDbl();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetDblArray(elmt,1,&value);
			break; }

		// Str
		case(Property::Str) : {
			string &value = property->getValueStr();
			char *str = (char *)value.c_str();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetStrArray(elmt,1,&str);
			break; }

		// Obj
		case(Property::Obj) : {
			Object &object = property->getValueObj();
			elmt = object._node;
			if((elmt==NULL) && (!property->getUseDefault())) {
				XMLNode::AppendNewCommentElement(_node, property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			if(elmt || !property->getUseDefault())
				object.updateXMLNode(_node);
#if 0
			elmt = XMLNode::GetFirstChildElementByTagName(_node,object.getType());
			if(!property->getComment().empty()) {
				if(elmt) XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
				else XMLNode::AppendNewCommentElement(_node, property->getComment());
			}
			object.updateXMLNode(_node);
#endif
			break; }

		// BoolArray
		case(Property::BoolArray) : {
			Array<bool> &value = property->getValueBoolArray();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetBoolArray(elmt,value.getSize(),&value[0]);
			break; }

		// IntArray
		case(Property::IntArray) : {
			Array<int> &value = property->getValueIntArray();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetIntArray(elmt,value.getSize(),&value[0]);
			break; }

		// DblArray
		case(Property::DblArray) : {
			Array<double> &value = property->getValueDblArray();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetDblArray(elmt,value.getSize(),&value[0]);
			break; }

		// StrArray
		case(Property::StrArray) : {
			Array<string> &value = property->getValueStrArray();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if((elmt==NULL) && (!property->getUseDefault())) {
				elmt= XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			XMLNode::SetStrArray(elmt,value.getSize(),value.get());
			break; }

		// ObjArray
		case(Property::ObjArray) : {
			ArrayPtrs<Object> &value = property->getValueObjArray();
			elmt = XMLNode::GetFirstChildElementByTagName(_node,name);
			if(elmt==NULL) { // I guess we always append the element even if it has default value
				elmt = XMLNode::AppendNewElementWithComment(_node, name, "", property->getComment());
			} else if (elmt && !property->getComment().empty()) {
				XMLNode::UpdateCommentNodeCorrespondingToChildElement(elmt,property->getComment());
			}
			for(int j=0;j<value.getSize();j++) {
				value.get(j)->updateXMLNode(elmt);
			}
			break; }

		// NOT RECOGNIZED
		default : {
			cout<<"Object.UpdateObject: WARN- unrecognized property type."<<
				endl;

			break; }
		}
	}
}

//_____________________________________________________________________________
/**
 * Generate an XML node to represent this object.
 *
 * If the parent node is NULL the intent is to generate a new document as
 * well as a new node.  Howerver, for this request to be successful,
 * a document must not already be associated with the object.
 *
 * If the parent node is not NULL and this object already has a node, the
 * node and the parent must be from the same document.  Requesting that a
 * new node be generated when one already exists is useful when it is
 * desired to remake a node entirely including elements for all the
 * properties of an object.
 *
 * @param aParent Intended parent of the node to be generated.  If aParent is
 * NULL, the intent is for this object to serve as the root element of
 * a new document.
 */
void Object::
generateXMLNode(DOMElement *aParent)
{
	// CHECK FOR DIFFERENT DOCUMENTS ERROR
	if((aParent!=NULL)&&(_node!=NULL)) {
		if(aParent->getOwnerDocument()!=_node->getOwnerDocument()) {
			printf("Object.generateNode: ERROR- object cannot switch ");
			printf("documents.\n");
			return;
		}
	}

	// REQUEST FOR NEW DOCUMENT BUT DOCUMENT ALREADY EXISTS ERROR
	if((aParent==NULL)&&(_node!=NULL)) {
		printf("Object.generateNode: ERROR- request to generate new ");
		printf("document, but a document already exists.\n\n");
		return;
	}

	// CREATE NEW DOCUMENT
	if(aParent==NULL) {
		generateXMLDocument();

	// ONLY CREATE A NEW NODE
	} else {

		// REMOVE EXISTING NODE
		if(_node!=NULL) {
			printf("Object.generateNode: WARN- node already existed.\n");
			printf("\t...Removing old node.\n");
			DOMNode *parent = _node->getParentNode();
			if(parent!=NULL) parent->removeChild(_node);
		}

		// GENERATE NEW NODE
		_node = XMLNode::AppendNewElementWithComment(aParent,getType(),getName(),"");
	}
}

//-----------------------------------------------------------------------------
// NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the XML node for this object.
 *
 * This method is not intended for general use.  Generally, the XML node
 * of an object is assigned during construction.  In rare instances, however,
 * it is convenient to have the ability to set the XML node of an object.
 * This functionality is private and only accessible from within Object.
 *
 * @return Node set for this object.
 */
void Object::
setXMLNode(DOMElement *aNode)
{
	_node = aNode;
}
//_____________________________________________________________________________
/**
 * Get the XML node for this object.
 *
 * @return Node set for this object.
 */
DOMElement* Object::
getXMLNode() const
{
	return(_node);
}

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
	if(_node!=NULL) {
		printf("Object.generateDocument: ERROR- request to generate new ");
		printf("document, but a document already exists.\n\n");
		return;
	}

	// CREATE NEW DOCUMENT
	if (_document==NULL)
		_document = new XMLDocument();
	// NEW ROOT ELEMENT
	_node = XMLNode::AppendNewElementWithComment(_document->getDOMDocument(),
		getType(),getName(),"");

}
//_____________________________________________________________________________
/**
 * NULL out the _node pointers for an object and all its descendents
 */
void Object::
clearXMLNodes()
{
	// recursively traverse tree of objects under current object and reset _node to NULL
	_node = NULL;

	// LOOP THROUGH PROPERTIES
	for(int i=0;i<_propertySet.getSize();i++) {
		Property *property = _propertySet.get(i);

		// TYPE, we're only interested in Obj and ObjArray
		Property::PropertyType type = property->getType();

		//----------- TRY BLOCK FOR PROPERTIES ------------
		try {

		// VALUE
		switch(type) {

		// Obj
		case(Property::Obj) : {
			Object &object = property->getValueObj();
			object.clearXMLNodes();
			break; }

		// ObjArray
		case(Property::ObjArray) : {

			// CLEAR EXISTING OBJECT ARRAY
			ArrayPtrs<Object> &objArray = property->getValueObjArray();

			for(int j=0;j<objArray.getSize();j++) {
				objArray[j]->clearXMLNodes();
			}

			break; }

		// Not an object so doesn't have a _node to worry about
		default : {

			break; }
		}

		//-----------------------------------------------------
		} catch(Exception x) {
			if(Object_DEBUG) x.print(cout);
		}
		//----------- END TRY BLOCK FOR PROPERTIES ------------
	}
}


//=============================================================================
// XML support for inlining/offlining objects
//=============================================================================
/**
 * Mark the object as inlined (written to same xml file as parent or not
 * If the object will not be inlined then a file name to associate with the
 * object is passed in.
 */
void Object::
setInlined(const bool aInLined, const char *aFileName)
{
	if (_inLined != aInLined ){	// Object needs to be regenerated
		if (_inLined==true){
			// Replace original node
			if (_node){
				DOMElement *aParent = (DOMElement *)_node->getParentNode();
				const XMLCh *parentName = aParent->getTagName();
				cout << "Removing Node "<< getType() << " from " << parentName << endl;
				aParent->removeChild(_node);
				// May need to remove other "space" siblings here as well

				// create a refNode in parent document that has file name and object name
				_refNode = XMLNode::AppendNewElementWithComment(aParent, getType(),getName(),"");
				XMLNode::SetAttribute(_refNode,"file",aFileName);
				// Zero out _node and children nodes so they get regenerated as well
				clearXMLNodes();
			}
			_document = new XMLDocument(); // Force regeneratopn of document
			_document->setFileName(aFileName);
			_node = NULL;
		}
		else {	//_inLined was false. 
			// Remove refNode and NULL the rest for regeneration
			if (_refNode){
				DOMElement *aParent = (DOMElement *)_refNode->getParentNode();
				aParent->removeChild(_refNode);
				_refNode = NULL;

			}
			if (_document){
				delete _document;
				_document = NULL;
			}
			// Zero out _node and children nodes so they get regenerated as well
			clearXMLNodes();
		}

	}
	_inLined = aInLined;
}
/**
 * Get the value of the inlined flag
 */
bool Object::
getInlined() const
{
	return _inLined;
}
/** 
 * If an object is not inlined, management of Offline file name to readFrom/writeTo */
const char *Object::
getOffLineFileName() const
{
	if(getDocument()!= NULL)
		return getDocument()->getFileName();
	return NULL;
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
	//	if(_node==NULL) 
	// Check removed per Clay so that users don't have to manually call
	// updateXMLNode for subsequent saves.  Ayman 5/07/04.
	updateXMLNode(NULL);
	if(_document==NULL) return false;
	return _document->print(aFileName);
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
	if((aPropertyName=="")||(aPropertyName=="*")) {
		int size = propertySet.getSize();
		aOStream<<"\nPROPERTIES FOR "<<aClassName<<" ("<<size<<")\n";
		string comment;
		for(int i=0;i<size;i++) {
			property = propertySet.get(i);
			if(property==NULL) continue;
			if(aPropertyName=="") {
				aOStream<<i+1<<". "<<property->getName()<<endl;
			} else {
				aOStream<<"\n"<<i+1<<". "<<property->getName()<<"\n";
				comment = property->getComment();
				if(!comment.empty()) {
					string formattedComment = IO::formatComment(comment,"\t",80);
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
	} catch(...) {
		aOStream<<"\nPrintPropertyInfo: no property with the name "<<aPropertyName;
		aOStream<<" was found in class "<<aClassName<<".\n";
		aOStream<<"Omit the property name to get a listing of all properties in a class.\n";
		return;
	}

	// OUTPUT
	//aOStream<<"\nPROPERTY INFO FOR "<<aClassName<<"\n";
	aOStream<<endl<<aClassName<<"."<<aPropertyName<<"\n"<<property->getComment()<<"\n";
}


//=============================================================================
// Utilities, factory methods
//=============================================================================
string Object::
transcode(const XMLCh *aCh)
{
	char *buffer = XMLString::transcode(aCh);
	string str(buffer);
	delete[] buffer;
	return str;
}

string Object::
transcodeAndTrim(const XMLCh *aCh)
{
	char *buffer = XMLString::transcode(aCh);
	XMLString::trim(buffer);
	string str(buffer);
	delete[] buffer;
	return str;
}

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
		DOMElement* elt = doc->getDOMDocument()->getDocumentElement();
		string rootName = transcode(elt->getNodeName());
		Object* newObject = newInstanceOfType(rootName);
		newObject->_document=doc;
		newObject->setXMLNode(elt);
		newObject->buildTypeNamesTable();
		newObject->updateFromXMLNode();
		return (newObject);
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
		rTypeNames.append(find_Iter->first);
	}
}
/** 
    * The following code accounts for an object made up to call 
    * RegisterTypes_rdTools function on entry to the DLL in a cross platform manner 
    * 
    * @todo Figure out if there're scenarios due to static initialization order that breaks this. 
    * -Ayman May 06 
    */ 
     
class rdToolsInstantiator 
{ 
public: 
        rdToolsInstantiator(); 
private: 
        void registerDllClasses(); 
}; 
    
rdToolsInstantiator::rdToolsInstantiator() 
{ 
        registerDllClasses(); 
} 
    
extern "C" RDTOOLS_API void RegisterTypes_rdTools(); 
void rdToolsInstantiator::registerDllClasses() 
{ 
        RegisterTypes_rdTools(); 
} 
    
static rdToolsInstantiator instantiator; 
