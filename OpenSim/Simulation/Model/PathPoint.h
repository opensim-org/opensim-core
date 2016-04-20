#ifndef OPENSIM_PATH_POINT_H_
#define OPENSIM_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  PathPoint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "OpenSim/Simulation/Model/Point.h"
#include "OpenSim/Simulation/Model/Station.h"
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

class PhysicalFrame;
class Model;
class GeometryPath;
class SimbodyEngine;
class WrapObject;

//=============================================================================
//=============================================================================
/**
 * A class implementing a path point.
 *
 * @author Peter Loan
 * @version 1.0
 */

//Preserve old PathPoint behavior while basing on concrete Points

class OSIMSIMULATION_API PathPoint : public Station {
OpenSim_DECLARE_CONCRETE_OBJECT(PathPoint, Station);

//=============================================================================
// DATA
//=============================================================================
protected:

    GeometryPath* _path; // the path that owns this location point

    // TODO: Remove this temporary hack to support transition to new Component
    // interface and properties handling.
    SimTK::Vec3 _location;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    PathPoint() :  Station() {
        _location = this->get_location();
    }

    virtual void init(const PathPoint& point) {
        *this = point;
    }
   void copyData(const PathPoint &aPoint);

#ifndef SWIG
    const SimTK::Vec3& getLocation() const { return _location; }
#endif
    SimTK::Vec3& getLocation()  { return _location; }

    const double& getLocationCoord(int aXYZ) const {
        assert(aXYZ>=0 && aXYZ<=2); return _location[aXYZ];
    }

    // A variant that uses basic types for use by GUI
    void setLocationCoord(int aXYZ, double aValue) {
        assert(aXYZ>=0 && aXYZ<=2);
        _location[aXYZ]=aValue;
    }

    void setLocation( const SimTK::State& s, const SimTK::Vec3& location) {
        _location = location;
    }
    void setLocation( const SimTK::State& s, int aCoordIndex, double aLocation) {
        if (aCoordIndex >= 0 && aCoordIndex <= 2)
            _location[aCoordIndex] = aLocation;
    }
    void setLocation( const SimTK::State& s, double pt[]){ // A variant that uses basic types for use by GUI
        setLocation(s,SimTK::Vec3::updAs(pt));
    }
    void setBody(const PhysicalFrame& body) {
        this->setParentFrame(body);
    }

    void changeBodyPreserveLocation(const SimTK::State& s, PhysicalFrame& body){
        if (!hasParent()) {
            throw Exception("PathPoint::changeBodyPreserveLocation attempted to "
                " change the body on PathPoint which was not assigned to a body.");
        }
        // if it is already assigned to aBody, do nothing
        const PhysicalFrame& currentFrame = getParentFrame();

        if (currentFrame == body)
            return;

        // Preserve location means to switch bodies without changing
        // the location of the point in the inertial reference frame.
        _location = currentFrame.findLocationInAnotherFrame(s, _location, body);

        // now assign this point's body to point to aBody
        setParentFrame(body);
    }

    const PhysicalFrame& getBody() const { return getParentFrame(); }
    const std::string& getBodyName() const { return getParentFrame().getName(); }

    GeometryPath* getPath() const { return _path; }

    virtual void scale(const SimTK::State& s, const SimTK::Vec3& scaleFactors) {
        for (int i = 0; i < 3; i++)
            _location[i] *= scaleFactors[i];
    }

    virtual const WrapObject* getWrapObject() const { return NULL; }

    virtual bool isActive(const SimTK::State& s) const { return true; }
    virtual void connectToModelAndPath(const Model& aModel, GeometryPath& aPath) {};
    virtual void update(const SimTK::State& s) { }

    // get the relative velocity of the path point with respect to the body
    // it is connected to.
    virtual void getVelocity(const SimTK::State& s, SimTK::Vec3& velocity) {
        velocity[0] = velocity[1] = velocity[2] = 0.0;
    }
    // get the partial of the point location w.r.t. to the coordinates (Q)
    // it is dependent on.
    virtual SimTK::Vec3 getdPointdQ(const SimTK::State& s) const
        { return SimTK::Vec3(0); }

    virtual void updateGeometry() {}

    // Utility
    static PathPoint* makePathPointOfType(PathPoint* aPoint,
        const std::string& aNewTypeName);

    static void deletePathPoint(PathPoint* aPoint) { if (aPoint) delete aPoint; }

//=============================================================================
};  // END of class PathPoint_
//=============================================================================
//=============================================================================


} // end of namespace OpenSim

#endif // OPENSIM_PATH_POINT_H_
