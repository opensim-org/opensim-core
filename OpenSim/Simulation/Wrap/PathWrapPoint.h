#ifndef OPENSIM_PATH_WRAP_POINT_H_
#define OPENSIM_PATH_WRAP_POINT_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PathWrapPoint.h                          *
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
#include <OpenSim/Simulation/Model/PathPoint.h>

namespace OpenSim {

class WrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a path wrapping point, which is a path point that
 * is produced by a PathWrap.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathWrapPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrapPoint, PathPoint);
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PathWrapPoint() {}
    virtual ~PathWrapPoint() {}

    Array<SimTK::Vec3>& getWrapPath() { return _wrapPath; }
    double getWrapLength() const { return _wrapPathLength; }
    void setWrapLength(double aLength) { _wrapPathLength = aLength; }
    const WrapObject* getWrapObject() const override { return _wrapObject.get(); }
    void setWrapObject(const WrapObject* wrapObject) { _wrapObject.reset(wrapObject); }

//=============================================================================
// DATA
//=============================================================================
private:
    // points defining muscle path on surface of wrap object
    Array<SimTK::Vec3> _wrapPath{};
    // length of _wrapPath TODO this should be a cache variable!
    double _wrapPathLength{ 0.0 };

    // the wrap object this point is on
    SimTK::ReferencePtr<const WrapObject> _wrapObject; 

//=============================================================================
};  // END of class PathWrapPoint
//=============================================================================
//=============================================================================


} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_POINT_H_


