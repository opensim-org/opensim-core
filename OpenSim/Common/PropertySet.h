#ifndef _PropertySet_h_
#define _PropertySet_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PropertySet.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
 * Author: Frank C. Anderson, Ajay Seth 
 */
#ifdef _WIN32
#pragma warning( disable : 4251 )
#endif

// INCLUDES
#include "osimCommonDLL.h"
#include "ArrayPtrs.h"
//#include "Property_Deprecated.h"
#include "PropertyDblVec.h"


#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
        #define OSIMCOMMON_API
    #endif
#endif

#ifndef SWIG
#ifdef _WIN32
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Property_Deprecated>;
#endif
#endif


namespace OpenSim { 

class Property_Deprecated;

// convenient abbreviations
typedef PropertyDblVec_<2> PropertyDblVec2;
typedef PropertyDblVec_<3> PropertyDblVec3;
typedef PropertyDblVec_<4> PropertyDblVec4;
typedef PropertyDblVec_<5> PropertyDblVec5;
typedef PropertyDblVec_<6> PropertyDblVec6;

//=============================================================================
//=============================================================================
/**
 * A property set is simply a set of properties.  It provides methods for
 * adding, removing, and retrieving properties from itself.
 *
 * @version 1.0
 * @author Frank C. Anderson
 * @see Property
 */
class OSIMCOMMON_API PropertySet  
{

//=============================================================================
// DATA
//=============================================================================
public:
    /** Set of properties. */
    ArrayPtrs<Property_Deprecated> _array;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertySet();
    PropertySet(const PropertySet &aSet);
    virtual ~PropertySet() { _array.setSize(0); };

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    friend std::ostream& operator<<(std::ostream &aOut,
                                                            const PropertySet &aSet) {
        aOut << "\nProperty Set:\n";
        for(int i=0;i<aSet.getSize();i++) aOut << *aSet.get(i) << "\n";
        return(aOut);
    }
#endif
    //--------------------------------------------------------------------------
    // ACCESS
    //--------------------------------------------------------------------------
public:
    // Empty?
    bool isEmpty() const;
    // Number of properties
    int getSize() const;
    // Get
    virtual Property_Deprecated* get(int i) throw (Exception);
#ifndef SWIG
    virtual const Property_Deprecated* get(int i) const;
#endif
    virtual Property_Deprecated* get(const std::string &aName) throw (Exception);
#ifndef SWIG
    virtual const Property_Deprecated* get(const std::string &aName) const;
#endif
    virtual const Property_Deprecated* contains(const std::string& aName) const;
#ifndef SWIG
    virtual Property_Deprecated* contains(const std::string& aName);
#endif
    // Append
    virtual void append(Property_Deprecated *aProperty);
    virtual void append(Property_Deprecated *aProperty, const std::string& aName);
    // Remove
    virtual void remove(const std::string &aName);
    // Clear
    virtual void clear();

//=============================================================================
};  // END of class PropertySet

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertySet_h__
