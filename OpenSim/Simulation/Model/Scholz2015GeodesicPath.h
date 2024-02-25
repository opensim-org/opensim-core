#ifndef OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
#define OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
/* -------------------------------------------------------------------------- *
 *                     OpenSim: Scholz2015GeodesicPath.h                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco, Pepijn van den Bos, Andreas Scholz             *
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

#include <OpenSim/Simulation/Model/AbstractGeometryPath.h>
#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Simulation/GeodesicWrapping/GeodesicWrapSurface.h>
#include <OpenSim/Simulation/GeodesicWrapping/GeodesicState.h>
#include <OpenSim/Common/Exception.h>

namespace OpenSim {

// TODO
class GeodesicWrapSolver {
};

class OSIMSIMULATION_API GeodesicPathSegment : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(GeodesicPathSegment, ModelComponent);

using VectorVec3 = SimTK::Vector_<SimTK::Vec3>;

public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(origin, Station,
            "TODO");
    OpenSim_DECLARE_SOCKET(insertion, Station,
            "TODO");
    OpenSim_DECLARE_LIST_SOCKET(surfaces, GeodesicWrapSurface,
            "TODO");

//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(initial_conditions, GeodesicInitialConditions,
            "TODO must match number and order of surfaces");

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION AND DESTRUCTION
    GeodesicPathSegment();
    ~GeodesicPathSegment() override;

    GeodesicPathSegment(const GeodesicPathSegment& other);
    GeodesicPathSegment& operator=(const GeodesicPathSegment& other);

    GeodesicPathSegment(GeodesicPathSegment&&);
    GeodesicPathSegment& operator=(GeodesicPathSegment&&);

    // GET AND SET
    void addWrapObject(const GeodesicWrapSurface& surface,
                       const GeodesicInitialConditions& initialConditions);

    double getLength(const SimTK::State& s) const;
    double getLengtheningSpeed(const SimTK::State& s) const;
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const;

protected:
    // MODEL COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendConnectToModel(Model& model) override;
    void extendInitStateFromProperties(SimTK::State& s) const override;

private:
    // CONVENIENCE METHODS
    void constructProperties();
    void calcWrappingPath(const SimTK::State& s) const;

    // MEMBER VARIABLES
    std::vector<GeodesicWrapObject> _wrapObjects;
    GeodesicWrapSolver _solver;

    // CACHE VARIABLES
    mutable CacheVariable<GeodesicWrapResult> _resultCV;
    // TODO: avoid this
    mutable std::vector<GeodesicInitialConditions> _initialConditions;
};


class OSIMSIMULATION_API Scholz2015GeodesicPath : public AbstractGeometryPath {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeodesicPath, AbstractGeometryPath);

using GeodesicWrapSurfaces = std::vector<std::reference_wrapper<GeodesicWrapSurface>>;

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    Scholz2015GeodesicPath();

    // GET AND SET
    // TODO: use this to create the first path segment.
    void addPathSegment(const GeodesicWrapSurfaces& surfaces,
            const std::vector<GeodesicInitialConditions>& initialConditions,
            const Station& origin, const Station& insertion);
    // TODO: use this to create subsequent path segments.
    void addPathSegment(const GeodesicWrapSurfaces& surfaces,
            const std::vector<GeodesicInitialConditions>& initialConditions,
            const Station& insertion);

    // ABSTRACT GEOMETRY PATH INTERFACE
    bool isVisualPath() const override;
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    void addInEquivalentForces(const SimTK::State& state,
            const double& tension,
            SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
            SimTK::Vector& mobilityForces) const override;
    double computeMomentArm(const SimTK::State& s,
            const Coordinate& aCoord) const override;

private:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // CONVENIENCE METHODS
    void computeLength(const SimTK::State& s) const;
    void computeLengtheningSpeed(const SimTK::State& s) const;

    // CACHE VARIABLES
    mutable CacheVariable<double> _lengthCV;
    mutable CacheVariable<double> _lengtheningSpeedCV;

}; // class Scholz2015GeodesicPath

} // namespace OpenSim


#endif // OPENSIM_SCHOLZ2015_GEODESIC_PATH_H
