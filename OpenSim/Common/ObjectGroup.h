#ifndef __ObjectGroup_h__
#define __ObjectGroup_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ObjectGroup.h                           *
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


// INCLUDE
#include <string>
#include "osimCommonDLL.h"
#include "PropertyStrArray.h"
#include "Object.h"

namespace OpenSim {

template <class T> class ArrayPtrs;

#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
        #define OSIMCOMMON_API
    #endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class implementing an object group. For most uses, object groups are
 * owned and managed by the Set that contains the object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API ObjectGroup : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(ObjectGroup, Object);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
    PropertyStrArray _memberNamesProp;
    Array<std::string>& _memberNames;

    Array<const Object*> _memberObjects;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ObjectGroup();
    ObjectGroup(const std::string& aName);
    ObjectGroup(const ObjectGroup &aGroup);
    virtual ~ObjectGroup();

#ifndef SWIG
    ObjectGroup& operator=(const ObjectGroup &aGroup);
#endif
   void copyData(const ObjectGroup &aGroup);

    bool contains(const std::string& aName) const;
    void add(const Object* aObject);
    void remove(const Object* aObject);
    void replace(const Object* aOldObject, const Object* aNewObject);
    void setupGroup(ArrayPtrs<Object>& aObjects);
    const Array<const Object*>& getMembers() const { return _memberObjects; }

private:
    void setNull();
    void setupProperties();

//=============================================================================
};  // END of class ObjectGroup
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ObjectGroup_h__


