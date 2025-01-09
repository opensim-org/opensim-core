#ifndef OPENSIM_PATH_WRAP_H_
#define OPENSIM_PATH_WRAP_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  PathWrap.h                            *
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
#include "PathWrapPoint.h"
#include "WrapResult.h"

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

class Model;
class WrapObject;
class GeometryPath;

/** @cond **/ // hide from Doxygen

//=============================================================================
//=============================================================================
/**
 * A class implementing an instance of muscle wrapping. That is, it is owned
 * by a particular muscle, and contains parameters for wrapping that muscle
 * over a particular wrap object.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathWrap : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrap, ModelComponent);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    OpenSim_DECLARE_PROPERTY(wrap_object, std::string,
        "A WrapObject that this PathWrap interacts with.");
    OpenSim_DECLARE_PROPERTY(method, std::string,
        "The wrapping method used to solve the path around the wrap object.");

    // TODO Range should not be exposed as far as one can tell, since all instances
    // are (-1, -1), which is the default value, and that means the PathWrap is
    // ignoring/overwriting this property anyways.
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(range, int, 2,
        "The range of indices to use to compute the path over the wrap object.")

    enum WrapMethod {
        hybrid,
        midpoint,
        axial
    };

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PathWrap();
    ~PathWrap();

#ifndef SWIG
    void setStartPoint( const SimTK::State& s, int aIndex);
    void setEndPoint( const SimTK::State& s, int aIndex);
#endif
    int getStartPoint() const { return get_range(0); }
    int getEndPoint() const { return get_range(1); }
    const std::string& getWrapObjectName() const { return get_wrap_object(); }
    const WrapObject* getWrapObject() const { return _wrapObject; }
    void setWrapObject(WrapObject& aWrapObject);

    const PathWrapPoint& getWrapPoint1() const {
        return getMemberSubcomponent<PathWrapPoint>(_wrapPoint1Ix);
    }
    const PathWrapPoint& getWrapPoint2() const {
        return getMemberSubcomponent<PathWrapPoint>(_wrapPoint2Ix);
    }
    PathWrapPoint& updWrapPoint1() { 
        return updMemberSubcomponent<PathWrapPoint>(_wrapPoint1Ix);
    }
    PathWrapPoint& updWrapPoint2() {
        return updMemberSubcomponent<PathWrapPoint>(_wrapPoint2Ix);
    }

    WrapMethod getMethod() const { return _method; }
    void setMethod(WrapMethod aMethod);
    const std::string& getMethodName() const { return get_method(); }

    const WrapResult& getPreviousWrap() const { return _previousWrap; }
    void setPreviousWrap(const WrapResult& aWrapResult);
    void resetPreviousWrap();

private:
    void constructProperties();
    void extendConnectToModel(Model& model) override;
    void setNull();

private:
    WrapMethod _method;

    const WrapObject* _wrapObject;
    const GeometryPath* _path;

    WrapResult _previousWrap;  // results from previous wrapping

    MemberSubcomponentIndex _wrapPoint1Ix{
        constructSubcomponent<PathWrapPoint>("pwpt1") };
    MemberSubcomponentIndex _wrapPoint2Ix{
        constructSubcomponent<PathWrapPoint>("pwpt2") };
//=============================================================================
};  // END of class PathWrap
//=============================================================================
//=============================================================================

/** @endcond **/

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_H_


