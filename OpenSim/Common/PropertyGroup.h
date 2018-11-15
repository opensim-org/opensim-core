#ifndef __PropertyGroup_h__
#define __PropertyGroup_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyGroup.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

#ifdef _WIN32
#pragma warning( disable : 4251 )
#endif

// INCLUDE
#include "osimCommonDLL.h"
#include "Property_Deprecated.h"
#include "Array.h"

namespace OpenSim {

#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
        #define OSIMCOMMON_API
    #endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class implementing a property group.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API PropertyGroup
{

//=============================================================================
// DATA
//=============================================================================
private:
    /** Name of the group. */
    std::string _name;

protected:
    /** Pointers to the properties in the group. */
    Array<Property_Deprecated*> _properties;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyGroup();
    PropertyGroup(std::string& aName);
    PropertyGroup(const PropertyGroup &aGroup);
    virtual ~PropertyGroup();
    virtual PropertyGroup* clone() const;

#ifndef SWIG
    PropertyGroup& operator=(const PropertyGroup &aGroup);
    bool operator<(const PropertyGroup &aGroup) const;
    bool operator==(const PropertyGroup& aGroup) const;
#endif
   void copyData(const PropertyGroup &aGroup);
    void clear();

    bool contains(const std::string& aName) const;
    void add(Property_Deprecated* aProperty);
    void remove(Property_Deprecated* aProperty);
#ifndef SWIG
    const Array<Property_Deprecated*>& getProperties() const { return _properties; }
#endif
    Property_Deprecated* get(int aIndex);
    int getPropertyIndex(Property_Deprecated* aProperty) const;

    // NAME
    void setName(const std::string &aName) { _name = aName; }
    const std::string& getName() const { return _name; }

private:
    void setNull();
//=============================================================================
};  // END of class PropertyGroup
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PropertyGroup_h__


