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

#include "Object.h"
#include "Property.h"
#include "osimCommonDLL.h"

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
public:
OpenSim_DECLARE_PROPERTY(scales, SimTK::Vec3, "scale factors");
OpenSim_DECLARE_PROPERTY(segment, std::string, "segment name");
OpenSim_DECLARE_PROPERTY(apply, bool, "whether or not to apply the scale");

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Scale();
    Scale( const std::string& scaleFileName);
    virtual ~Scale(void);

private:
    void constructProperties();

public:
    //--------------------------------------------------------------------------
    // SET AND GET
    //--------------------------------------------------------------------------
    const std::string& getSegmentName() const { return get_segment(); };
    void setSegmentName(const std::string& aSegmentName) {
        set_segment(aSegmentName);
    };

    const SimTK::Vec3& getScaleFactors() const { return get_scales(); }
    void setScaleFactors(const SimTK::Vec3& aScaleFactors) {
        set_scales(aScaleFactors);
    };
    bool getApply(void) const { return get_apply(); }
    void setApply(bool state) { set_apply(state); }
};

}; //namespace
#endif
