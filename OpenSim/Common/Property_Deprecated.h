#ifndef OPENSIM_PROPERTY_DEPRECATED_H_
#define OPENSIM_PROPERTY_DEPRECATED_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  Property_Deprecated.h                       *
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


// INCLUDES
#include "osimCommonDLL.h"
#include "AbstractProperty.h"
#include "Exception.h"

#include <string>

namespace OpenSim { 
/**
 * A property consists of a type, name, and a value or an array of values.
 *
 * Class Property is an abstract base class that provides the functionality
 * common to all property types.
 *
 * At the time of the first formulation of the property classes, the only
 * property types that were envisioned are for a few fundamental data types
 * and for Object:\n\n
 * \tbool\n
 * \tint\n
 * \tfloat\n
 * \tdouble\n
 * \tstring\n
 * \tObject\n
 * \tObject pointer\n
 * \tarray of bools\n
 * \tarray of ints\n
 * \tarray of floats\n
 * \tarray of doubles\n
 * \tarray of strings\n
 * \tarray of Objects\n\n
 *
 * As additional property types are needed, they may be added however.
 *
 * TODO: Make default constructors for all derived classes
 * @version 1.0
 * @author Frank C. Anderson
 */

class Object;
template <class T> class Array;

#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
    #endif
    #define OSIMCOMMON_API
#endif

#ifdef _WIN32
#pragma warning( disable : 4290 )   // VC++ non-ANSI Exception handling
#pragma warning( disable : 4251 )   // VC2010 no-dll export of std::string

#endif

#define Property_PROPERTY_TYPE_MISMATCH() \
    throw Exception(std::string(__FUNCTION__)+": Property type mismatch. This property is of type "+getTypeName()+".",__FILE__,__LINE__);

class OSIMCOMMON_API Property_Deprecated : public AbstractProperty
{
public:
    /** Enumeration of recognized types. */
    enum PropertyType
    {
        None=0, Bool, Int, Dbl, Str, Obj, ObjPtr,
        BoolArray, IntArray, DblArray, StrArray, ObjArray,
        DblVec, DblVec3,
        Transform // 3 BodyFixed X,Y,Z Rotations followed by 3 Translations
    };

//=============================================================================
// DATA
//=============================================================================
private:
    /** Type of the property. */
    PropertyType _propertyType;
    bool         _matchName;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Property_Deprecated();
    Property_Deprecated(PropertyType aType,const std::string &aName);
    Property_Deprecated(const Property_Deprecated &aProperty);


    /** Return the enum value corresponding to the concrete property. **/
    virtual PropertyType getPropertyType() const {return _propertyType;}

    /** Return true if this is an array property. **/
    virtual bool isArrayProperty() const {return false;}

    /** By default deprecated PropertyObj properties will ignore the name
    associated with the read-in object. This forces the name to match one
    specified for the property. Note that this is handled differently in
    the new Property system. */
    void setMatchName(bool aMatchName) { _matchName = aMatchName; }
    /** Return the value of the matchName flag for this (deprecated) 
    property. **/
    bool getMatchName() const { return _matchName; }

    //--------------------------------------------------------------------------
    // Implement the AbstractProperty interface.

    bool isEqualTo(const AbstractProperty& other) const override {
        return operator==(dynamic_cast<const Property_Deprecated&>(other));
    }

    // Property_Deprecated does not implement AbstractProperty::clone(); that 
    // is left to concrete Property_Deprecated objects like PropertyInt.
    Property_Deprecated* clone() const override = 0;

    virtual void readFromXMLElement
       (SimTK::Xml::Element& propertyElement,
        int                  versionNumber) override
    {assert(!"Property_Deprecated::readFromXMLElement not implemented");}

    virtual void writeToXMLElement
       (SimTK::Xml::Element& propertyElement) const override
    {assert(!"Property_Deprecated::writeToXMLElement not implemented");}

    // Override for array types.
    int getNumValues() const override {return 1;}
    void clearValues() override {assert(!"implemented");}

    bool isUnnamedProperty() const override {return false;}
    bool isObjectProperty() const override {return false;}
    bool isAcceptableObjectTag
        (const std::string& objectTypeTag) const override {return false;}
    const Object& getValueAsObject(int index=-1) const override
    {   Property_PROPERTY_TYPE_MISMATCH(); }
    Object& updValueAsObject(int index=-1) override
    {   Property_PROPERTY_TYPE_MISMATCH(); }
    void setValueAsObject(const Object& obj, int index=-1) override
    {   Property_PROPERTY_TYPE_MISMATCH(); }

    //--------------------------------------------------------------------------

    void setNull();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    Property_Deprecated& operator=(const Property_Deprecated &aProperty);
    virtual bool operator==(const Property_Deprecated &aProperty) const;
    virtual bool operator<(const Property_Deprecated &aProperty) const;
    friend std::ostream& operator<<(std::ostream &aOut,const Property_Deprecated &aProperty) {
        aOut << aProperty.getTypeName() << " " << aProperty.getName();
        return(aOut);
    };
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:

    // TYPE
    void setType(PropertyType aType);
    PropertyType getType() const;
    std::string getTypeName() const override =0;

    // VALUE
    // Textual representation
    std::string toString() const override =0;

    // These methods have been given default implementations, rather than being made pure virtual
    // so that all classes derived from Property will not have to implement each method.
    // Bool
    virtual void setValue(bool aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual bool& getValueBool() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    virtual const bool& getValueBool() const { Property_PROPERTY_TYPE_MISMATCH(); }
    // Int
    virtual void setValue(int aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual int& getValueInt() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    virtual const int& getValueInt() const { Property_PROPERTY_TYPE_MISMATCH(); }
    // Dbl
    virtual void setValue(double aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual double& getValueDbl() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    virtual const double& getValueDbl() const { Property_PROPERTY_TYPE_MISMATCH(); }
    // Str
    virtual void setValue(const std::string &aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual std::string& getValueStr() { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    virtual const std::string& getValueStr() const { Property_PROPERTY_TYPE_MISMATCH(); }
    // Bool Array
    virtual void setValue(int aSize,const bool aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual void setValue(const Array<bool> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual Array<bool>& getValueBoolArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual const Array<bool>& getValueBoolArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    // Int Array
    virtual void setValue(int aSize,const int aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual void setValue(const Array<int> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual Array<int>& getValueIntArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual const Array<int>& getValueIntArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    // Dbl Array
    virtual void setValue(int aSize,const double aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual void setValue(const Array<double> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual Array<double>& getValueDblArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual const Array<double>& getValueDblArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    // Str Array
    virtual void setValue(int aSize,const std::string aArray[]) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual void setValue(const Array<std::string> &aArray) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual Array<std::string>& getValueStrArray() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual const Array<std::string>& getValueStrArray() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif

    //--------------------------------------------------------------------------
    // Obj, ObjPtr, and ObjArray require more careful treatment
    //--------------------------------------------------------------------------
    virtual bool isValidObject(const Object *aValue) const { Property_PROPERTY_TYPE_MISMATCH(); }
    // Obj
    // Got rid of setValue(Obj) since it would be dangerous to do so given that users of
    // PropertyObj would continue to hold a reference to the (deleted) object - Eran.
    virtual Object& getValueObj() { Property_PROPERTY_TYPE_MISMATCH(); }
#ifndef SWIG
    virtual const Object& getValueObj() const { Property_PROPERTY_TYPE_MISMATCH(); }
#endif
    // ObjPtr
    virtual void setValue(Object *aValue) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual const Object* getValueObjPtr() const { Property_PROPERTY_TYPE_MISMATCH(); }

    // Obj Array
    virtual const Object* getValueObjPtr(int index) const { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual void appendValue(Object *obj) { Property_PROPERTY_TYPE_MISMATCH(); }
    virtual void clearObjArray() { Property_PROPERTY_TYPE_MISMATCH(); }

    // Generic way to get number of elements
    virtual int getArraySize() const { Property_PROPERTY_TYPE_MISMATCH(); }

    // Templates for get & set
    template<class T> T &getValue();
    template<class T> const T &getValue() const;
    template<class T> Array<T> &getValueArray();
    template<class T> const Array<T> &getValueArray() const;

//=============================================================================
};  // END of class Property_Deprecated

// Specializations of template get/set
// Must be inline! (Trying to put function bodies in cpp fails with an internal compiler error in VC7.1)
template<> inline bool& Property_Deprecated::getValue() { return getValueBool(); }
template<> inline const bool& Property_Deprecated::getValue() const { return getValueBool(); }
template<> inline int& Property_Deprecated::getValue() { return getValueInt(); }
template<> inline const int& Property_Deprecated::getValue() const { return getValueInt(); }
template<> inline double& Property_Deprecated::getValue() { return getValueDbl(); }
template<> inline const double& Property_Deprecated::getValue() const { return getValueDbl(); }
template<> inline std::string& Property_Deprecated::getValue() { return getValueStr(); }
template<> inline const std::string& Property_Deprecated::getValue() const { return getValueStr(); }

template<> inline Array<bool>& Property_Deprecated::getValue() { return getValueBoolArray(); }
template<> inline const Array<bool>& Property_Deprecated::getValue() const { return getValueBoolArray(); }
template<> inline Array<int>& Property_Deprecated::getValue() { return getValueIntArray(); }
template<> inline const Array<int>& Property_Deprecated::getValue() const { return getValueIntArray(); }
template<> inline Array<double>& Property_Deprecated::getValue() { return getValueDblArray(); }
template<> inline const Array<double>& Property_Deprecated::getValue() const { return getValueDblArray(); }
template<> inline Array<std::string>& Property_Deprecated::getValue() { return getValueStrArray(); }
template<> inline const Array<std::string>& Property_Deprecated::getValue() const { return getValueStrArray(); }

template<> inline Array<bool>& Property_Deprecated::getValueArray() { return getValueBoolArray(); }
template<> inline const Array<bool>& Property_Deprecated::getValueArray() const { return getValueBoolArray(); }
template<> inline Array<int>& Property_Deprecated::getValueArray() { return getValueIntArray(); }
template<> inline const Array<int>& Property_Deprecated::getValueArray() const { return getValueIntArray(); }
template<> inline Array<double>& Property_Deprecated::getValueArray() { return getValueDblArray(); }
template<> inline const Array<double>& Property_Deprecated::getValueArray() const { return getValueDblArray(); }
template<> inline Array<std::string>& Property_Deprecated::getValueArray() { return getValueStrArray(); }
template<> inline const Array<std::string>& Property_Deprecated::getValueArray() const { return getValueStrArray(); }

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_DEPRECATED_H_
