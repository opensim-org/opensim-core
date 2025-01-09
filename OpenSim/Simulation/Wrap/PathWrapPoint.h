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
 * Copyright (c) 2005-2022 Stanford University and the Authors                *
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

#include <OpenSim/Simulation/Model/AbstractPathPoint.h>

// legacy: this used to be included (older codebases may transitively depend on it)
#include <OpenSim/Simulation/Model/PathPoint.h>

namespace OpenSim {

class WrapObject;

/**
 * A class implementing a path wrapping point, which is a path point that
 * is produced by a PathWrap.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API PathWrapPoint final : public AbstractPathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(PathWrapPoint, AbstractPathPoint);

public:
    void extendAddToSystem(SimTK::MultibodySystem&) const override;

    const WrapObject* getWrapObject() const override;
    void setWrapObject(const WrapObject*);

    const Array<SimTK::Vec3>& getWrapPath(const SimTK::State&) const;
    void setWrapPath(const SimTK::State&, const Array<SimTK::Vec3>&) const;
    void clearWrapPath(const SimTK::State&) const;

    double getWrapLength(const SimTK::State&) const;
    void setWrapLength(const SimTK::State&, double newLength) const;

    // careful: Although this class effectively represents a *sequence*
    //          of many wrapping points, these methods still need to be
    //          here for legacy compatability with `AbstractPathPoint`,
    //          which is used extensively by `GeometryPath`.
    //
    // The implementation uses the first, or last, tangent point of the path
    // wrap as a "best guess" of the location. `GeometryPath`, itself, handles
    // stuffing the relevant data data into this class.
    //
    // (later iterations of `GeometryPath` may ideally (e.g.) hold a sequence
    //  of path *segments*, rather than path *points*)
    SimTK::Vec3 getLocation(const SimTK::State&) const override;
    void setLocation(const SimTK::State&, const SimTK::Vec3&) const;
    SimTK::Vec3 getdPointdQ(const SimTK::State&) const override;

private:
    SimTK::Vec3 calcLocationInGround(const SimTK::State&) const override;
    SimTK::Vec3 calcVelocityInGround(const SimTK::State&) const override;
    SimTK::Vec3 calcAccelerationInGround(const SimTK::State&) const override;

    // points defining muscle path on surface of wrap object
    mutable CacheVariable<Array<SimTK::Vec3>> _wrapPath;

    // length of _wrapPath
    mutable CacheVariable<double> _wrapPathLength;

    // location of the first tangent point of the path wrap
    mutable CacheVariable<SimTK::Vec3> _location;

    // the wrap object this point is on
    SimTK::ReferencePtr<const WrapObject> _wrapObject; 
};

} // end of namespace OpenSim

#endif // OPENSIM_PATH_WRAP_POINT_H_


