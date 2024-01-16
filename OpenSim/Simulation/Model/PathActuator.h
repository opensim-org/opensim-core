#ifndef OPENSIM_PATH_ACTUATOR_H_
#define OPENSIM_PATH_ACTUATOR_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  PathActuator.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "AbstractGeometryPath.h"
#include "Actuator.h"
#include "GeometryPath.h"

//=============================================================================
//=============================================================================

namespace OpenSim { 

class Coordinate;
class ForceSet;
class Model;

/**
 * This is the base class for actuators that apply controllable tension along 
 * a path. %PathActuator has no states; the control is simply the tension to be
 * applied along a path (i.e. tensionable rope).
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API PathActuator : public ScalarActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(PathActuator, ScalarActuator);
public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(path, AbstractGeometryPath,
        "The path of the actuator which defines length and lengthening speed.");
    OpenSim_DECLARE_PROPERTY(optimal_force, double,
        "The maximum force this actuator can produce.");

    OpenSim_DECLARE_OUTPUT(tension, double, computeActuation,
                           SimTK::Stage::Acceleration);

//=============================================================================
// PUBLIC METHODS
//=============================================================================
    PathActuator();

    // default destructor, copy constructor, copy assignment

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // Path
    AbstractGeometryPath& updPath() { return upd_path(); }
    const AbstractGeometryPath& getPath() const { return get_path(); }

    template <typename PathType>
    PathType& updPath() {
        return dynamic_cast<PathType&>(upd_path());
    }
    template <typename PathType>
    const PathType& getPath() const {
        return dynamic_cast<const PathType&>(get_path());
    }

    template <typename PathType>
    PathType* tryUpdPath() {
        return dynamic_cast<PathType*>(&upd_path());
    }
    template <typename PathType>
    const PathType* tryGetPath() const {
        return dynamic_cast<const PathType*>(&get_path());
    }

    GeometryPath& updGeometryPath() {
        return updPath<GeometryPath>();
    }
    const GeometryPath& getGeometryPath() const {
        return getPath<GeometryPath>();
    }
    
    bool hasVisualPath() const override { return getPath().isVisualPath(); };

    // OPTIMAL FORCE
    void setOptimalForce(double aOptimalForce);
    double getOptimalForce() const override;

    // Length and Speed of actuator
    virtual double getLength(const SimTK::State& s) const;
    virtual double getLengtheningSpeed(const SimTK::State& s) const;

    //--------------------------------------------------------------------------
    // Implement ScalarActuator Interface
    //--------------------------------------------------------------------------
    double getSpeed(const SimTK::State& s) const final
    {
        return getLengtheningSpeed(s);
    }

    // Power: Since lengthening is positive and tension always shortens, positive power
    // is when muscle is shortening under tension.
    double getPower(const SimTK::State& s) const override
    {   return -getActuation(s)*getSpeed(s); }


    // STRESS
    double getStress(const SimTK::State& s) const override;

    // Convenience method to add PathPoints
    /** @note This function does not maintain the State and so should be used
     * only before a valid State is created.
     * @note Only valid if the `path` owned by this PathActuator supports
     * PathPoint%s (e.g., GeometryPath). */
    void addNewPathPoint(const std::string& proposedName,
                         const PhysicalFrame& aBody,
                         const SimTK::Vec3& aPositionOnBody);

    //--------------------------------------------------------------------------
    // APPLICATION
    //--------------------------------------------------------------------------
    virtual void computeForce(const SimTK::State& state,
                               SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
                               SimTK::Vector& mobilityForces) const override;

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    double computeActuation(const SimTK::State& s) const override;
    virtual double computeMomentArm(const SimTK::State& s, Coordinate& aCoord) const;

protected:
    /** Override this method if you would like to calculate a color for use when
    the %PathActuator's path is displayed in the visualizer. You do not have 
    to invoke the base class ("Super") method, just replace it completely. This
    method will be invoked during realizeDynamics() so the supplied \a state has 
    already been realized through Stage::Velocity and you can access time, 
    position, and velocity dependent quantities. You must \e not attempt to 
    realize the passed-in \a state any further since we are already in the 
    middle of realizing here. Return SimTK::Vec3(SimTK::NaN) if you want to 
    leave the color unchanged (that's what the base class implementation does).

    @param[in] state    
        A SimTK::State already realized through Stage::Velocity. Do not 
        attempt to realize it any further.
    @returns 
        The desired color for the path as an RGB vector with each
        component ranging from 0 to 1, or NaN to indicate that the color
        should not be changed. **/
    virtual SimTK::Vec3 computePathColor(const SimTK::State& state) const;

    /** Extension of parent class method; derived classes may extend further. **/
    void extendRealizeDynamics(const SimTK::State& state) const override;

private:
    void setNull();
    void constructProperties();

//=============================================================================
};  // END of class PathActuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_PATH_ACTUATOR_H_


