#ifndef _Object_h_
#define _Object_h_
// Object.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
1
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
#include <cstring>
#include <assert.h>
#include <map>
#include "osimCommonDLL.h"
#include "XMLDocument.h"
#include "PropertySet.h"
#include "PropertyTable.h"
#include "Property2.h"

// DISABLES MULTIPLE INSTANTIATION WARNINGS


// EXPORT LINE FOR MICROSOFT VISUAL C++
#ifdef WIN32
#ifndef SWIG
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Object>;
#endif
#endif

typedef std::map<std::string, OpenSim::Object* > stringsToObjects;
typedef std::map<std::string, bool> defaultsReadFromFile;


// CONSTANTS
const char ObjectDEFAULT_NAME[] = "default";

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

/** VisibleObject needs Object for persistence while Object
	needs VisibleObject so that anything can be made visible.
	Ideally VisibleObject would be broken into interfaces
	so that the behavior only (not the serialization) is used here.*/
class VisibleObject;
class XMLDocument;
//=============================================================================
//=============================================================================
/**
 * Class Object is intended to be used as the base class for all
 * OpenSim objects.that are serializable  It provides a common object from which
 * to derive and also some basic functionality, such as writing to files
 * in XML format and the equality, less than, and output operators.
 * Future enhancements to Object might include thread functionality.
 *
 * @version 1.0
 * @author Frank C. Anderson, Ayman Habib
 * 
 */
class OSIMCOMMON_API Object  
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Array of all registered object types.  Each object type only appears
	once in this array.  Future enhancements could be using a hash table
	instead of an array. */
	static ArrayPtrs<Object> _Types;

	/** 
	 * A Hash map that maps an std::string& to the corresponding default object.
	 */
	static stringsToObjects _mapTypesToDefaultObjects;
	/**
	 * A list of types that has been depreacted so we can take them out when writing
	 */
	static Array<std::string> _deprecatedTypes;

	/**
	 * Global flag to indicate if all registered objects are written in the defaults section
	 */
	static bool _serializeAllDefaults;

	/**
	 * Debug level: 
	 *	0: Hides non fatal warnings 
	 *  1: Shows illegal tags 
	 *  2: level 1 + registration troubleshooting
	 *  3: 2 + more verbose troubleshooting of Object (de)serialization
	 */
	static int _debugLevel;

public:
#ifndef SWIG
	/** Name used for default objects when they are serialized. */
	static const std::string DEFAULT_NAME;
#endif
protected:
	/** Property set for serializable member variables of this and
	derived classes. */
	PropertySet _propertySet;
	PropertyTable _propertyTable;
	/** Type. */
	std::string _type;
	/** Name */
	std::string _name;
	/** A description of the object */
	std::string _description;

	/** A List of authors */
	std::string _authors;
	/** A List of references */
	std::string _references;

	/** XML document.*/
	XMLDocument *_document;
	/** flag indicating whether the object is serialized to this _document or to another fresh document */
	bool _inlined;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	/** Constrctor to be used by no arg constructor of derived types. Sets type to Object and initializes 
	 * PropertyList. */
	Object();

	/**
	 * Constructor from a File, normally called from other constructors that take a file as input
	 */
	Object(const std::string &aFileName, bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;

	/**
	 * Copy constructor for Object, used by Arrays of Objects
	 */
	Object(const Object &aObject);

	/**
	 * Constructor Object from an Xml element that describes this Object. Assumes latest XML file format
	 */
	Object(SimTK::Xml::Element& aElement);

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

private:
	void setNull();
	void setupProperties();
	void init();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	/**
	 * Equality operator wrapper for use from languages not supporting operator overloading
	 */
	virtual bool isEqualTo(const Object &aObject) const
	{
		return ((*this)==aObject);
	}
#ifndef SWIG
	Object& operator=(const Object &aObject);
	virtual bool operator==(const Object &aObject) const;
	virtual bool operator<(const Object &aObject) const;
	friend std::ostream& operator<<(std::ostream &aOut,
												const Object &aObject) {
		aOut << aObject.getType() << " " << aObject.getName();
		return(aOut);
	};
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:
	void setType(const std::string &aType);
public:
	/** Get type of current Object */
	const std::string& getType() const;
	/** Set the name of the Object */
	void setName(const std::string &aName);
	/** Get the name */
	const std::string& getName() const;
	/** Set description, a one liner summary. Currently unused */
	void setDescription(const std::string &aDescrip);
	/** Get description, a one liner summary. Currently unused */
	const std::string& getDescription() const;

	/** get Authors of this Object */
	const std::string& getAuthors() const { return _authors; };
	/** set Authors of this object, call this method in your constructor if needed */
	void setAuthors(const std::string &aAuthors) { _authors=aAuthors; };

	/** get References or Publications to site if using this object */
	const std::string& getReferences() const { return _references; };
	/** set References or Publications to site if using this object */
	void setReferences(const std::string &aReferences) { _references=aReferences; };

	const std::string& toString() const;
	PropertySet& getPropertySet() { return(_propertySet); }
#ifndef SWIG
	const PropertySet& getPropertySet() const { return(_propertySet); }
#endif

	//--------------------------------------------------------------------------
	// REGISTRATION OF TYPES AND DEFAULT OBJECTS
	//--------------------------------------------------------------------------
	static void RegisterType(const Object &aObject);
	static void RenameType(const std::string& oldTypeName, const Object& aObjectOfNewType);

	static void setDebugLevel(int newLevel) {
		_debugLevel=newLevel; 
	};
	static int getDebugLevel() {
		return _debugLevel; 
	};
	/*=============================================================================
	 * makeObjectFromFile creates an OpenSim object based on the tag at the root
	 * node of the XML file passed in. This is useful since the constructor of Object 
	 * doesn't have the proper type info. This works by using the defaults table so 
	 * that "Object" does not need to know about 
	 * derived classes. It uses the defaults table to get an instance.
	 * =============================================================================*/
	static Object* makeObjectFromFile(const std::string &aFileName);

	/*=============================================================================
	 * newInstanceOfType Creates a new instance of the type indicated by aType. 
	 * The instance is initialized to the default Object
	 * of corresponding type.
	 =============================================================================*/
	static Object* newInstanceOfType(const std::string &aType);

	/*=============================================================================
	* getRegisteredTypenames is a utility to retrieve all the typenames registered so far.
	* This is done by traversing the registered objects map, so only concrete classes that 
	* have registered instances are dealt with.
	* The result returned in rTypeNames should not be cached while more dlls are loaded 
	* as they get stale
	* instead the list should be constructed whenever in doubt
	=============================================================================*/
	static void getRegisteredTypenames(Array<std::string>& rTypeNames);

	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	/**
	 * Use this method to serialize an object from a SimTK::Xml::Element,
	 * The element is assumed to be on the format consistent with passed in versionNumber
	 */
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber);
	/**
	 * Use this method only if you're deserializing from a file and the object is TopLevel
	 *  that is primarily in constructors that take fileName as input
	 */
	void updateFromXMLDocument();
	virtual void updateXMLNode(SimTK::Xml::Element& aParent);
	// Inline support
	bool getInlined() const;
	void setInlined(bool aInlined, const std::string &aFileName="");
	XMLDocument* getDocument() const;
	std::string getDocumentFileName() const;
	void setAllPropertiesUseDefault(bool aUseDefault);
private:
	/**
	 * Functions to support deserialization 
	 */
	void generateXMLDocument();
	static void InitializeObjectFromXMLNode(Property *aProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber);
	static void InitializeObjectFromXMLNode2(AbstractProperty *aAbstractProperty, const SimTK::Xml::element_iterator& rObjectElement, Object *aObject, int versionNumber);
	void updateDefaultObjectsFromXMLNode();
	void updateDefaultObjectsXMLNode(SimTK::Xml::Element& aParent);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	bool print(const std::string &aFileName);
	static void PrintPropertyInfo(std::ostream &aOStream,
					const std::string &aClassNameDotPropertyName);
	static void PrintPropertyInfo(std::ostream &aOStream,
					const std::string &aClassName,const std::string &aPropertyName);


public:
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
public: 
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
	template<class T> static void getRegisteredObjectsOfGivenType(ArrayPtrs<T> &rArray) {
		rArray.setSize(0);
		rArray.setMemoryOwner(false);
		for(int i=0; i<_Types.getSize(); i++)
			if(dynamic_cast<T*>(_Types[i]))
				rArray.append(dynamic_cast<T*>(_Types[i]));
	}

protected:
	/** Methods to handle Properties */
	void addProperty(AbstractProperty &abstractProperty);
	template <class T> const Property2<T>& getProperty(const std::string &name) const;
	template <class T> Property2<T>& updProperty(const std::string &name);
	template <class T> void addProperty(const std::string &name, const std::string &type, const std::string &comment, const T &value);
	template <class T> const T& getPropertyValue(const std::string &name) const;
	template <class T> T& updPropertyValue(const std::string &name) const;
	template <class T> void setPropertyValue(const std::string &name, const T &value);
	std::string getPropertyType(const std::string &name) const;
	std::string getPropertyComment(const std::string &name) const;
	Array<AbstractProperty *> getPropertyArray();

//=============================================================================
};	// END of class Object

template <>
inline AbstractProperty::PropertyType Property2<Object>::getPropertyType() const { return Obj; }

template <>
inline AbstractProperty::PropertyType Property2< ArrayPtrs<Object> >::getPropertyType() const { return ObjArray; }

template <>
inline AbstractProperty::PropertyType Property2<Object *>::getPropertyType() const { return ObjPtr; }

template <class T>
const Property2<T>& Object::
getProperty(const std::string &name) const
{
	return _propertyTable.getProperty<T>(name);
}

template <class T>
Property2<T>& Object::
updProperty(const std::string &name)
{
	return _propertyTable.updProperty<T>(name);
}

template <class T>
void Object::
addProperty(const std::string &name, const std::string &type, const std::string &comment, const T &value)
{
	_propertyTable.addProperty(name, type, comment, value);
}

template <class T>
const T& Object::
getPropertyValue(const std::string &name) const
{
	return _propertyTable.getPropertyValue<T>(name);
}

template <class T>
T& Object::
updPropertyValue(const std::string &name) const
{
	return _propertyTable.updPropertyValue<T>(name);
}

template <class T>
void Object::
setPropertyValue(const std::string &name, const T &value)
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

#endif //__Object_h__
