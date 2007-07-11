#ifndef _Object_h_
#define _Object_h_
// Object.h
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#pragma warning( disable : 4786 )
#pragma warning( disable : 4660 )
#endif

// INCLUDES
#include <assert.h>
#include <map>
#include "Observable.h"
#include "Event.h"
#include "osimCommonDLL.h"
#include "PropertySet.h"

// Only the necessary Xerces includes/defines
#include <xercesc/util/XercesDefs.hpp>
#ifndef SWIG
XERCES_CPP_NAMESPACE_BEGIN
class DOMElement;
XERCES_CPP_NAMESPACE_END
XERCES_CPP_NAMESPACE_USE
#else
class DOMElement;	// should be in proper namespace if we care
					// namespace has XERCES version encoded in it
					// but we never use DOMStuff from Java anyway.
					// Swig barfs on the above syntax otherwise. -Ayman 2/07
#endif
// DISABLES MULTIPLE INSTANTIATION WARNINGS


// EXPORT LINE FOR MICROSOFT VISUAL C++
#ifdef WIN32
#ifndef SWIG
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Object>;
#endif
#endif

typedef std::map<std::string, OpenSim::Object*, std::less<std::string> > stringsToObjects;
typedef std::map<std::string, bool, std::less<std::string> > defaultsReadFromFile;


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

//=============================================================================
//=============================================================================
/**
 * Class rdOject is intended to be used as the base class for all
 * Realistic Dynamics, Inc. objects.  It provides a common object from which
 * to derive and also some basic functionality, such as writing to files
 * in XML format and the equality, less than, and output operators.
 * Future enhancements to Object might include thread functionality.
 *
 * @version 1.0
 * @author Frank C. Anderson
 * @todo Use a hash table to store registered object types.
 */
namespace OpenSim { 

/** VisibleObject needs Object for persistence while Object
	needs VisibleObject so that anything can be made visible.
	Ideally VisibleObject would be broken into interfaces
	so that the behavior only (not the serialization) is used here.*/
class VisibleObject;
class XMLDocument;

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
	 * A Hash map that maps an std::string& to the flag specifying if default objects are user specified in file
	 */
	static defaultsReadFromFile	_defaultsReadFromFile;
	/**
	 * Global flag to indicate if all registered objects are written in the defaults section
	 */
	static bool _serializeAllDefaults;
	/**
	* A pointer to the observable object implementation. Null if not needed to minimize
	* memory overhead 
	*/
	Observable	*_observable;
public:
	/** A length limit for a name. */
#ifndef SWIG
	enum { NAME_LENGTH=128 };
	/** Name used for default objects when they are serialized. */
	static const std::string DEFAULT_NAME;
#endif
protected:
	/** Property set for serializable member variables of this and
	derived classes. */
	PropertySet _propertySet;
	/** Type. */
	std::string _type;
	/** Name. Made protected so that classes can customize their setName.  
	-Ayman 04/04 */
	std::string _name;
	/** A description of the object. -> rdSerializable interface */
	std::string _description;
	/** XML document. -> rdSerializable interface */
	XMLDocument *_document;
	/** XML element node. -> rdSerializable interface */
	DOMElement *_node;
	/** Inlined object -> rdSerializable interface */
	bool _inLined;
	/** For non inlined objects their _node and _document refer to external file
	 * _refNode contains the type and reference to file name -> rdSerializable interface */
	DOMElement *_refNode;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~Object();
	Object();
	Object(const std::string &aFileName, bool aUpdateFromXMLNode = true) SWIG_DECLARE_EXCEPTION;
	Object(const XMLDocument *aDocument);
	Object(DOMElement *aNode);
	Object(const Object &aObject);
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aNode) const;
	virtual void copy(const Object& aObject);
	static Object* SafeCopy(const Object *aObject) { return aObject ? aObject->copy() : 0; }
//	static Object* ConstructObject(DOMElement *aNode);
	virtual VisibleObject *getDisplayer() const { return 0; };
private:
	void setNull();
	void setupProperties();
	void init();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
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
public:
	void setType(const std::string &aType);
	const std::string& getType() const;
	void setName(const std::string &aName);
	const std::string& getName() const;
	void setDescription(const std::string &aDescrip);
	const std::string& getDescription() const;
	const std::string& toString() const;
	PropertySet& getPropertySet() { return(_propertySet); }
#ifndef SWIG
	const PropertySet& getPropertySet() const { return(_propertySet); }
#endif

	//--------------------------------------------------------------------------
	// REGISTRATION OF TYPES AND DEFAULT OBJECTS
	//--------------------------------------------------------------------------
	static void RegisterType(const Object &aObject);

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
	// XML NEW
	//--------------------------------------------------------------------------
	virtual bool isValidDefaultType(const Object *aObject) const;
	virtual void updateFromXMLNode();
	virtual void updateDefaultObjectsFromXMLNode();
	virtual void updateXMLNode(DOMElement *aParent);
	virtual void updateDefaultObjectsXMLNode(DOMElement *aParent);
	virtual void generateXMLNode(DOMElement *aParent);
	// Inline support
	bool getInlined() const;
	XMLDocument* getDocument() const;
	std::string getDocumentFileName() const;
	DOMElement* getXMLNode() const;
protected:
	void setXMLNode(DOMElement *aNode);
private:
	void generateXMLDocument();
	static bool parseFileAttribute(DOMElement *aElement, DOMElement *&rRefNode, XMLDocument *&rChildDocument, DOMElement *&rChildDocumentElement, bool aVerifyTagName = true);
	static void InitializeObjectFromXMLNode(Property *aProperty, DOMElement *&rObjectElement, Object *aObject);

	//--------------------------------------------------------------------------
	// IO
	//--------------------------------------------------------------------------
public:
	bool print(const std::string &aFileName);
	static void PrintPropertyInfo(std::ostream &aOStream,
					const std::string &aClassNameDotPropertyName);
	static void PrintPropertyInfo(std::ostream &aOStream,
					const std::string &aClassName,const std::string &aPropertyName);

	//--------------------------------------------------------------------------
	// Observable Interface
	//--------------------------------------------------------------------------
	// Manage Observers
	//--------------------------------------------------------------------------
	void addObserver(Object& aObserver)
	{
		if (!_observable)
			_observable = new Observable(*this);	// Lazy creation of observable
		assert(_observable);
		_observable->addObserver(aObserver);
	};
	void deleteObserver(Object& aObserver)
	{
		assert(_observable);
		_observable->deleteObserver(aObserver);
	};
	/**
	 * Eventually observers will have to specify what event to observe so that they're not
	 * called unnecessarily. Will do this after the Event class hierarchy matures.
	 */
	void addObserverEvent(Object& aObserver, Event& aEvent)
	{};
	void notifyObservers(Event& aEvent)
	{
		if (!_observable)	// No observer has been added
			return;
		assert(_observable);
		_observable->notifyObservers(aEvent);
	};

	void deleteObservers()
	{
		assert(_observable);
		_observable->deleteObservers();
	};
	int countObservers() const
	{
		if (!_observable)	// No observer has been added
			return 0;		
		assert(_observable);
		return _observable->countObservers();
	};
protected:
	//--------------------------------------------------------------------------
	// Manage _changed flag
	//--------------------------------------------------------------------------
	void setChanged()
	{
		if (!_observable)	// No observer has been added
			return;		
		assert(_observable);
		_observable->setChanged();
	};
	void clearChanged()
	{
		if (!_observable)	// No observer has been added
			return;		
		assert(_observable);
		_observable->clearChanged();
	};
	bool hasChanged()
	{
		if (!_observable)	// No observer has been added
			return false;		
		assert(_observable);
		return (_observable->hasChanged());
	};
public:
	virtual void update(const Object& aObject, Event& aEvent) {};
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

//=============================================================================
};	// END of class Object
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
