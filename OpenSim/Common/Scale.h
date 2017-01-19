#ifndef _Scale_h_
#define _Scale_h_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Scale.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

/*  
 * Author:  
 */


#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyStr.h"
#include "PropertyDblArray.h"
#include "PropertyDbl.h"
#include "PropertyBool.h"
#include "PropertyDblVec.h"

//=============================================================================
/*
 * A Class representing scale factors for an object
 *
 * @author Ayman Habib
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API Scale : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(Scale, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
    // PROPERTIES
    /** A list of 3 scale factors */
    PropertyDblVec3 _propScaleFactors;
    /** Name of object to scale */
    PropertyStr     _propSegmentName;
    /** Whether or not to apply this scale */
    PropertyBool        _propApply;

    // REFERENCES
    SimTK::Vec3&    _scaleFactors;
    std::string&        _segmentName;
    bool&                   _apply;

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Scale();
    Scale(const Scale &aMarker);
    Scale( const std::string& scaleFileName);
    virtual ~Scale(void);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    Scale& operator=(const Scale &aMarker);
#endif  
private:
    void setNull();
    void setupProperties();

public:
    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
    const std::string& getSegmentName() const;
    void setSegmentName(const std::string& aSegmentName);

    void getScaleFactors(SimTK::Vec3& aScaleFactors) const;
    SimTK::Vec3& getScaleFactors() { return _scaleFactors; }
    void setScaleFactors(const SimTK::Vec3& aScaleFactors);
    bool getApply(void) const { return _apply; }
    void setApply(bool state) { _apply = state; }
};

}; //namespace
#endif
