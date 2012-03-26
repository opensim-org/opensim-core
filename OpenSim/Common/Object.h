#ifndef OPENSIM_OBJECT_H_
#define OPENSIM_OBJECT_H_
// Object.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005-12, Stanford University. All rights reserved. 
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#pragma warning( disable : 4786 )
#pragma warning( disable : 4660 )
#endif

// INCLUDES

#include "osimCommonDLL.h"
#include "XMLDocument.h"
#include "PropertySet.h"
#include "PropertyTable.h"
#include "Property2.h"

#include <cstring>
#include <cassert>
#include <map>

// DISABLES MULTIPLE INSTANTIATION WARNINGS


// EXPORT LINE FOR MICROSOFT VISUAL C++
#ifdef WIN32
#ifndef SWIG
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Object>;
#endif
#endif


#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
	#define SWIG_DECLARE_EXCEPTION throw(OpenSim::Exception)
#else
	#define SWIG_DECLARE_EXCEPTION
#endif


namespace OpenSim { 

typedef std::map<std::string, OpenSim::Object* > StringsToObjects;

// CONSTANTS
const char ObjectDEFAULT_NAME[] = "default";

class VisibleObject;
class XMLDocument;

//==============================================================================
//                                 OBJECT
//==============================================================================
// TODO: this class should be renamed "SerializableObject" or something like 
// that.

/** This is the base class for all %OpenSim objects that are serializable 
(meaning they can be written to and read back from files). In particular, all 
ModelComponents derive from %Object. It provides a common base class from which
to derive serializable objects and also some basic functionality, such as 
writing to files in XML format, managing properties, and the equality, 
less than, and output operators.

An %Object maintains a list of "Properties" that know how to read themselves 
from XML and write themselves to XML. The available Property types are
  -# Primitive data types (int, bool, double, std::string, ...) 
  -# Properties that are other Objects, 
  -# Arrays of either of the previous 2 categories

An %Object type needs to be "registered" by calling "RegisterType" with an 
instance of a concrete object so that the serialization infrastructure knows 
what kind of %Object to create when it encounters a specific XML tag. The 
registration process is normally done during dynamic library (dll) loading.

Defaults mechanism: When an %Object is registered (either programmatically, or
overridden in the defaults section of a document), a copy of it is maintained 
in a dictionary as a "default" object of its class. When new instances of this 
class are requested, the contents of the defaut object are used to populate the
new instance before deserialization. This allows for specifying default values
that will be commonly used in one place in the XML file rather then with each 
object which leads to smaller files that are easier to read.

@version 1.0
@author Frank C. Anderson, Ayman Habib 
**/
class OSIMCOMMON_API Object  
{
//------------------------------------------------------------------------------
// PUBLIC METHODS
//------------------------------------------------------------------------------
public:
	/** 
     * Constructor from a file, normally called from other constructors that 
     * take a file as input. 
     */
	Object(const std::string& fileName, 
           bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;

	/**
	 * Copy constructor for Object, used by Arrays of Objects
	 */
	Object(const Object &aObject);

	/**
	 * Constructor Object from an Xml element that describes this Object. Assumes latest XML file format
	 */
	explicit Object(SimTK::Xml::Element& aElement);

	/**
	 * Virtual destructor for cleanup
	 */
	virtual ~Object();

	/**
	 * copy method that returns a clone of the Object. Used to grow Arrays of Objects
	 */
	virtual Object* copy() const;

	static Object* SafeCopy(const Object *aObject) { return aObject ? aObject->copy() : 0; }

	/**
	 * Methods to support making the object displayable in the GUI or Visualizer
	 * Implemented only in few objects
	 */
	/** Get const pointer to VisibleObject that contains geometry */
	virtual const VisibleObject *getDisplayer() const { return 0; };
	/** get Non const pointer to VisibleObject */
	virtual VisibleObject *updDisplayer() { return 0; };


	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
	/**
	 * Equality operator wrapper for use from languages not supporting operator
     * overloading.
	 */
	bool isEqualTo(const Object &aObject) const
	{
		return ((*this)==aObject);
	}

    #ifndef SWIG
    /** Copy assignment copies only the simple base class fields, not the 
    properties. **/
	Object& operator=(const Object &aObject);
    /** Determine if two objects are equal. They are equal if all the simple
    base class members are equal, both objects have the same number of 
    properties and corresponding properties are equal, and if the objects
    are the same concrete type and the concrete class says they are equal. 
    Concrete object classes must override this if they have any fields to
    compare, but be sure to invoke the base class operator too. **/
	virtual bool operator==(const Object &aObject) const;
    /** Provide an ordering for objects so they can be put in sorted
    containers. **/
	virtual bool operator<(const Object &aObject) const;
    /** Write the type and name of this object into the given output stream. **/
	friend std::ostream& operator<<(std::ostream &aOut,
									const Object &aObject) {
		aOut << aObject.getType() << " " << aObject.getName();
		return(aOut);
	};
    #endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	/** Get type of current concrete Object as a string suitable for use as
    an XML tag for that object. */
	const std::string& getType() const;
	/** Set the name of the Object. */
	void setName(const std::string& name);
	/** Get the name of this Object. */
	const std::string& getName() const;
	/** Set description, a one-liner summary. */
	void setDescription(const std::string& description);
	/** Get description, a one-liner summary. */
	const std::string& getDescription() const;

	/** Get Authors of this Object */
	const std::string& getAuthors() const { return _authors; };
	/** Set Authors of this object, call this method in your constructor if needed */
	void setAuthors(const std::string& authors) { _authors=authors; };

	/** Get references or publications to cite if using this object. */
	const std::string& getReferences() const { return _references; };
	/** Set references or publications to cite if using this object. */
	void setReferences(const std::string& references) 
    {   _references=references; };


	//--------------------------------------------------------------------------
    /** @name              Public access to properties
    Methods in this section are for public access to the properties maintained
    by this OpenSim %Object. These are returned as AbstractProperty objects
    which support various type-independent property services. If you know the
    type you can use templatized methods to deal with the value directly. **/
    /**@{**/
    /** Determine how many properties are stored with this %Object. These
    are numbered 0..n-1 in the order they were created. **/
    int getNumProperties() const;
    /** Get a const reference to a property by its index number, returned as
    an AbstractProperty. **/
    const AbstractProperty& getPropertyByIndex(int propertyIndex) const;
    /** Get a writable reference to a property by its index number, returned as
    an AbstractProperty. **/
    AbstractProperty& updPropertyByIndex(int propertyIndex);
    /** Get a const reference to a property by its name, returned as
    an AbstractProperty. An exception is thrown if no property by this name
    is present in this %Object. **/
    const AbstractProperty& getPropertyByName(const std::string& name) const;
    /** Get a writable reference to a property by its name, returned as
    an AbstractProperty.  An exception is thrown if no property by this name
    is present in this %Object. **/
    AbstractProperty& updPropertyByName(const std::string& name);
    /**@}**/
	//--------------------------------------------------------------------------


	//--------------------------------------------------------------------------
	// REGISTRATION OF TYPES AND DEFAULT OBJECTS
	//--------------------------------------------------------------------------
	/** Register an instance of a class, if the class is already registered it will throw an Exception */
	static void RegisterType(const Object &aObject);

	/** Support versioning by associating new Object type with old name */
	static void RenameType(const std::string& oldTypeName, const Object& aObjectOfNewType);

	static void setDebugLevel(int newLevel) {
		_debugLevel=newLevel; 
	};
	static int getDebugLevel() {
		return _debugLevel; 
	};

	/** Create a new instance of the type whose tag name is given as \a type. 
	The instance is initialized to the default object of corresponding type. **/
	static Object* newInstanceOfType(const std::string& type);

	/** Retrieve all the typenames registered so far. This is done by traversing
    the registered objects map, so only concrete classes that have registered 
    instances are dealt with. The result returned in \c typeNames should not be 
    cached while more shared libraries or plugins are loaded, because more types
    may be registered as a result. Instead the list should be reconstructed 
    whenever in doubt. **/
	static void getRegisteredTypenames(Array<std::string>& typeNames);

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	/** Create an %OpenSim object whose type is based on the tag at the root 
    node of the XML file passed in. This is useful since the constructor of 
    %Object doesn't have the proper type info. This works by using the defaults 
    table so that %Object does not need to know about its derived classes. It 
    uses the defaults table to get an instance. **/
	static Object* makeObjectFromFile(const std::string& fileName);


	/** Use this method only if you're deserializing from a file and the object
    is at the top level; that is, primarily in constructors that take a file
    name as input. **/
	void updateFromXMLDocument();


	/** Use this method to deserialize an object from a SimTK::Xml::Element. The 
    element is assumed to be in the format consistent with passed in 
    \a versionNumber. **/
	virtual void updateFromXMLNode(SimTK::Xml::Element& node, 
                                   int                  versionNumber);

    /** Serialize this object into the XML node that represents it.   
    @param      parent 
        Parent XML node of this object. Sending in a parent node allows an XML 
        node to be generated for this object if it doesn't already have one. If 
        no parent node is supplied and this object doesn't already have an XML 
        node, this object will become the root node for a new XML document. If 
        this object already has an XML node associated with it, no new nodes 
        are ever generated and the parent node is not used. 
    **/
	virtual void updateXMLNode(SimTK::Xml::Element& parent);

	/** Inlined means an in-memory Object that is not associated with
    an XMLDocument. **/
	bool getInlined() const;
    /** Mark this as inlined or not and optionally provide a file name
    to associate with the new XMLDocument for the non-inline case. If 
    there was already a document associated with this object it is
    deleted. **/
	void setInlined(bool aInlined, const std::string &aFileName="");
    /** Unconditionally set the XMLDocument associated with this object.
    Use carefully -- if there was already a document its heap space is
    lost here. **/
    void setDocument(XMLDocument* doc) {_document=doc;}
    /** Get a const pointer to the document (if any) associated with this
    object. **/
    const XMLDocument* getDocument() const {return _document;}
    /** Get a writable pointer to the document (if any) associated with this
    object. **/
    XMLDocument* updDocument() {return _document;}
    /** If there is a document associated with this object then return the
    file name maintained by the document. Otherwise return an empty string. **/
	std::string getDocumentFileName() const;
	void setAllPropertiesUseDefault(bool aUseDefault);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
	bool print(const std::string &aFileName);
	static void PrintPropertyInfo(std::ostream &aOStream,
					const std::string &aClassNameDotPropertyName);
	static void PrintPropertyInfo(std::ostream &aOStream,
					const std::string &aClassName,const std::string &aPropertyName);

	/* Static functions to specify and query if all registered objects are written 
	 * to the defaults section of output files rather than only those 
	 * explicitly specified by the user in input files */
	static void setSerializeAllDefaults(bool aBoolean)
	{
		_serializeAllDefaults = aBoolean;
	}
	static bool getSerializeAllDefaults()
	{
		return _serializeAllDefaults;
	}

	static bool isKindOf(const char *type) 
	{ 
		return (strcmp("Object",type)==0);
	} 
	virtual bool isA(const char *type) const
	{ 
		return this->isKindOf(type); 
	} 
	/**
	 * Method to return instances of Objects that derives from a given type, useful for example
	 * to find all Joints, Constraints, Analyses, ... etc.
	 */
	template<class T> static void getRegisteredObjectsOfGivenType(ArrayPtrs<T>& rArray) {
		rArray.setSize(0);
		rArray.setMemoryOwner(false);
		for(int i=0; i<_Types.getSize(); i++)
			if(dynamic_cast<T*>(_Types[i]))
				rArray.append(dynamic_cast<T*>(_Types[i]));
	}

	//--------------------------------------------------------------------------
    /** @name                      Advanced/Obscure
    Methods in this section are for specialized purposes not of interest to
    most OpenSim API users. For example, some of these are services needed by
    the OpenSim GUI which is written in Java. **/
    /**@{**/

    /** Wrapper to be used on Java side to display objects in tree; this returns
    just the object's name. **/
	const std::string& toString() const;

	/** OBSOLETE: Get a reference to the PropertySet maintained by the 
    Object. **/
	PropertySet& getPropertySet() { return _propertySet; }
	const PropertySet& getPropertySet() const { return _propertySet; }
    /**@}**/
	//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// PROTECTED METHODS
//--------------------------------------------------------------------------
protected:
	/** The default constructor is only for use by constructors of 
    derived types. Sets type string to "Object" and initializes all base class
    data members to innocuous values. **/
	Object();

    /** Concrete derived objects can use this to tell the base class their 
    type as a string suitable for use as an XML tag. **/
	void setType(const std::string& type);

	/** Get property \a name of known type T as a const reference; the property
    must exist. **/
	template <class T> const Property2<T>& 
    getProperty(const std::string& name) const;
	/** Get property \a name of known type T as a writable reference; the 
    property must exist. **/
	template <class T> Property2<T>& 
    updProperty(const std::string& name);

    /** Define a new property of known type T, with the given \a name and
    associated \a comment. **/
	template <class T> void addProperty(const std::string& name, 
                                        const std::string& comment, 
                                        const T&           value);
    /** Obtain a const reference to the value of known type T that is stored
    in the property named \a name, which must exist. **/
	template <class T> const T& getPropertyValue(const std::string& name) const;
    /** Obtain a writable reference to the value of known type T that is stored
    in the property named \a name, which must exist. **/
	template <class T> T& updPropertyValue(const std::string& name);
    /** Set the value of the property named \a name, which must exist and be
    a property of type T. **/
	template <class T> void 
    setPropertyValue(const std::string& name, const T& value);

    /** Obtain the property type as a string; this is just for display and
    debugging. **/
	const std::string& getPropertyTypeAsString(const std::string& name) const;
    /** Obtain the comment that was associated with this property at the time
    it was defined. **/
	const std::string& getPropertyComment(const std::string& name) const;

//--------------------------------------------------------------------------
// PRIVATE METHODS
//--------------------------------------------------------------------------
private:
	void setNull();
	void setupProperties();
	void init();

	// Functions to support deserialization. 
	void generateXMLDocument();
	static void InitializeObjectFromXMLNode(Property_Deprecated *aProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber);
	static void InitializeObjectFromXMLNode2(AbstractProperty *aAbstractProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber);
	void updateDefaultObjectsFromXMLNode();
	void updateDefaultObjectsXMLNode(SimTK::Xml::Element& aParent);


//==============================================================================
// DATA
//==============================================================================
public:
    #ifndef SWIG
	/** Name used for default objects when they are serialized. */
	static const std::string DEFAULT_NAME;
    #endif

protected:
	/** OBSOLETE: Property set for serializable member variables of this and
	derived classes. */
	PropertySet _propertySet;

private:
	// Array of all registered object types. Each object type only appears once 
    // in this array. TODO: use map or hash table. 
	static ArrayPtrs<Object> _Types;

	// Map from type (tag) name to a default object of that type.
	static StringsToObjects _mapTypesToDefaultObjects;

	// A list of types that have been deprecated so we can take them out when
    // writing.
	static Array<std::string> _deprecatedTypes;

	// Global flag to indicate if all registered objects are written in the 
    // defaults section.
	static bool _serializeAllDefaults;

	// Debug level: 
	//	0: Hides non fatal warnings 
	//  1: Shows illegal tags 
	//  2: level 1 + registration troubleshooting
	//  3: 2 + more verbose troubleshooting of Object (de)serialization. When 
    //     used from ava wrapping in GUI/Matlab this catches all exceptions 
    //     thrown by the low level libraries which is slower but helpful in 
    //     troubleshooting.
	static int _debugLevel;

	// Property table for serializable properties of this and derived classes.
	PropertyTable _propertyTable;
	// The type of this object as a string suitable for use as an XML tag.
	std::string _type;
	// The name of this object.
	std::string _name;
	// A short description of the object.
	std::string _description;

	// List of authors who contributed to the concrete object.
	std::string _authors;
	// List of references that should be cited when using this concrete object.
	std::string _references;

	// The XML document, if any, associated with this object.
	XMLDocument *_document;
	// Flag indicating whether the object is serialized to this _document or 
    // to another fresh document.
	bool _inlined;

//=============================================================================
};	// END of class Object


template <class T> const T& AbstractProperty::getValue() const {
    //TODO: temporary support for obsolete properties
    const Property_Deprecated* pd = 
        dynamic_cast<const Property_Deprecated*>(this);
    if (pd) return pd->getValue<T>();

    const Property2<T>* p = dynamic_cast<const Property2<T>*>(this);
    if (p == NULL)
        throw Exception("AbstractProperty::getValue(): property "
                        + getName() + " is not of type " 
                        + std::string(TypeHelper<T>::name()));
    return p->getValue();
}

template <class T> T& AbstractProperty::updValue() {
    //TODO: temporary support for obsolete properties
    Property_Deprecated* pd = dynamic_cast<Property_Deprecated*>(this);
    if (pd) return pd->getValue<T>();

    Property2<T>* p = dynamic_cast<Property2<T>*>(this);
    if (p == NULL)
        throw Exception("AbstractProperty::updValue(): property "
                        + getName() + " is not of type " 
                        + std::string(TypeHelper<T>::name()));
    return p->updValue();
}

template <class T> const Property2<T>& Object::
getProperty(const std::string& name) const
{
	return _propertyTable.getProperty<T>(name);
}

template <class T> Property2<T>& Object::
updProperty(const std::string& name)
{
	return _propertyTable.updProperty<T>(name);
}

template <class T> void Object::
addProperty(const std::string& name, const std::string& comment, 
            const T& value)
{
	_propertyTable.addProperty(name, comment, value);
}

template <class T> const T& Object::
getPropertyValue(const std::string& name) const
{
	return _propertyTable.getPropertyValue<T>(name);
}

template <class T> T& Object::
updPropertyValue(const std::string& name)
{
	return _propertyTable.updPropertyValue<T>(name);
}

template <class T> void Object::
setPropertyValue(const std::string& name, const T& value)
{
	_propertyTable.setPropertyValue(name, value);
}

/**
 * Add public static method declaration in class derived from a
 * parent to assist in downcasting objects of the parent type to the 
 * derived type as well as support dynamic casting across JNI.
 */
#define OPENSIM_DECLARE_DERIVED(thisClass,parentclass) \
	typedef parentclass Parent; \
  public: \
  static bool isKindOf(const char *type) \
  { \
    if (strcmp(#thisClass,type)==0) \
      { \
      return true; \
      } \
    return Parent::isKindOf(type); \
  } \
  virtual bool isA(const char *type) const \
  { \
    return this->thisClass::isKindOf(type); \
  } \
  static thisClass* safeDownCast(OpenSim::Object *obj) \
  { \
      return dynamic_cast<thisClass *>(obj); \
  } \
  virtual void copy(const Object &aObject) \
  { \
	  if (aObject.isA(#thisClass)) { \
		  *this = *((const thisClass*)(&aObject)); \
     } else { \
	  throw Exception(std::string(#thisClass)+"::copy() called with object (name = "+ \
              aObject.getName()+", type = "+aObject.getType()+").", \
              __FILE__,__LINE__); \
     } \
  }


}; //namespace

#endif // OPENSIM_OBJECT_H_
