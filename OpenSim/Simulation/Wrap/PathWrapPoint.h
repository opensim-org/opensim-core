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
#include <OpenSim/Simulation/Model/AbstractPathPoint.h>
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/Array.h>

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
class OSIMSIMULATION_API PathWrapPoint final : public AbstractPathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrapPoint, AbstractPathPoint);
//=============================================================================
// METHODS
//=============================================================================
public:
    PathWrapPoint();

    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    const Array<SimTK::Vec3>& getWrapPath(const SimTK::State& s) const;
    void setWrapPath(const SimTK::State& s, const Array<SimTK::Vec3>& src) const;
    void clearWrapPath(const SimTK::State& s) const;

    double getWrapLength(const SimTK::State& s) const;
    void setWrapLength(const SimTK::State& s, double aLength) const;

    const WrapObject* getWrapObject() const override;
    void setWrapObject(const WrapObject* wrapObject);

    SimTK::Vec3 getLocation(const SimTK::State& s) const override;
    void setLocation(const SimTK::State& s, const SimTK::Vec3& loc) const;

    SimTK::Vec3 getdPointdQ(const SimTK::State& s) const override;

private:
    SimTK::Vec3 calcLocationInGround(const SimTK::State& state) const override;
    SimTK::Vec3 calcVelocityInGround(const SimTK::State& state) const override;
    SimTK::Vec3 calcAccelerationInGround(const SimTK::State& state) const override;

//=============================================================================
// DATA
//=============================================================================

    // points defining the wrapped muscle path on surface of wrap object
    mutable CacheVariable<Array<SimTK::Vec3>> _wrapPath;

    // overall length of _wrapPath
    mutable CacheVariable<double> _wrapPathLength;

    // location of the first tangent point of the path wrap (i.e. where to connect to)
    mutable CacheVariable<SimTK::Vec3> _location;

    // the wrap object this point is on
    SimTK::ReferencePtr<const WrapObject> _wrapObject;

//=============================================================================
};  // END of class PathWrapPoint
//=============================================================================
//=============================================================================


} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_POINT_H_


