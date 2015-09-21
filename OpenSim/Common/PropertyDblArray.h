#ifndef OPENSIM_PROPERTY_DBL_ARRAY_H_
#define OPENSIM_PROPERTY_DBL_ARRAY_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PropertyDblArray.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

#include "osimCommonDLL.h"
#include <string>
#include "Property_Deprecated.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * Class PropertyDblArray extends class Property.  It consists of an
 * array of doubles (i.e., Array<double>) and the methods for accessing
 * and modifying this array.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
class OSIMCOMMON_API PropertyDblArray : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Array of doubles. */
    Array<double> _array;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PropertyDblArray();
    PropertyDblArray(const std::string &aName,
        const Array<double> &aArray);
    PropertyDblArray(const std::string &aName,
        int aSize,const double aArray[]);
    PropertyDblArray(const PropertyDblArray &aProperty);

    bool isArrayProperty() const override {return true;}

    PropertyDblArray* clone() const override;

    int getNumValues() const override {return getArraySize();}

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    PropertyDblArray& operator=(const PropertyDblArray &aProperty);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
public:
    // TYPE
    std::string getTypeName() const override;
    // VALUE
    virtual void setValue(const Array<double> &aArray) override;
    virtual void setValue(int aSize,const double aArray[]) override;

    virtual Array<double>& getValueDblArray() override;
#ifndef SWIG
    virtual const Array<double>& getValueDblArray() const override;
#endif
    // VALUE as String
    virtual std::string toString() const override;
    // SIZE
    virtual int getArraySize() const override { return _array.getSize(); }
    void clearValues() override {
        _array.setSize(0);
    }

//=============================================================================
};  // END of class PropertyDblArray

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PROPERTY_DBL_ARRAY_H_
