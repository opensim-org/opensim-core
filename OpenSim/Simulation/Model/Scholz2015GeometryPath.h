#ifndef OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
#define OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  Scholz2015GeometryPath.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
 * Contributor(s): Pepijn van den Bos, Andreas Scholz                         *
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

#include "OpenSim/Simulation/Model/AbstractGeometryPath.h"
#include "OpenSim/Simulation/Model/Station.h"
#include <simbody/internal/CableSpan.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>

namespace OpenSim {

class OSIMSIMULATION_API Scholz2015GeometryPathObstacle : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPathObstacle, Component);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(obstacle, ContactGeometry,
            "The list of obstacles that the path segment may intersect.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(contact_hint, SimTK::Vec3,
            "The contact hint for the obstacle.");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION
    Scholz2015GeometryPathObstacle();

    // ACCESSORS
    const ContactGeometry& getObstacle() const;

private:
    void constructProperties();
};

class OSIMSIMULATION_API Scholz2015GeometryPathSegment : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPathSegment, Component);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(origin, Station,
            "The origin station of the path segment.");
    OpenSim_DECLARE_SOCKET(insertion, Station,
            "The insertion station of the path segment.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(obstacles, Scholz2015GeometryPathObstacle,
            "The list of obstacles that the path segment may intersect.");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION
    Scholz2015GeometryPathSegment();

    // ACCESSORS
    int getNumObstacles() const;
    const Station& getOrigin() const;
    const Station& getInsertion() const;

private:
    // CONVENIENCE METHODS
    void constructProperties();
};

//=============================================================================
//                       SCHOLZ 2015 GEOMETRY PATH
//=============================================================================
/**
 * A concrete class representing a path for muscles, ligaments, etc., based on
 * Scholz et al. (2015) wrapping geometry algorithm.
 */
class OSIMSIMULATION_API Scholz2015GeometryPath : public AbstractGeometryPath {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPath, AbstractGeometryPath);

public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(origin, Station,
        "The origin station of the path.");
    OpenSim_DECLARE_SOCKET(insertion, Station,
        "The insertion station of the path.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(via_points, Station,
        "The list of via points that the path may pass through.");
    OpenSim_DECLARE_LIST_PROPERTY(segments, Scholz2015GeometryPathSegment,
        "The list of segments that the path may pass through.");
    OpenSim_DECLARE_PROPERTY(algorithm, SimTK::CableSpanAlgorithm,
        "The algorithm used to compute the path.");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION
    Scholz2015GeometryPath();
    Scholz2015GeometryPath(const Station& origin, const Station& insertion);

    // GET AND SET
    void setOrigin(const Station& origin);
    const Station& getOrigin() const;

    void setInsertion(const Station& insertion);
    const Station& getInsertion() const;

    void addObstacle(const ContactGeometry& obstacle,
            const SimTK::Vec3& contactHint);
    void addViaPoint(const Station& viaPoint);

    void setAlgorithm(SimTK::CableSpanAlgorithm algorithm);
    SimTK::CableSpanAlgorithm getAlgorithm() const;

    // ABSTRACT PATH INTERFACE
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    double computeMomentArm(const SimTK::State& s,
            const Coordinate& coord) const override;

    // FORCE PRODUCER INTERFACE
    void produceForces(const SimTK::State& s, double tension,
            ForceConsumer& consumer) const override;

    bool isVisualPath() const override { return true; }

private:
    // MODEL COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override;

    // CONVENIENCE METHODS
    const SimTK::CableSpan& getCableSpan() const;
    SimTK::CableSpan& updCableSpan();
    void constructProperties();

    // MEMBER VARIABLES
    mutable SimTK::ResetOnCopy<SimTK::CableSpanIndex> _index;
    mutable SimTK::ResetOnCopy<SimTK::Array_<SimTK::CableSpanObstacleIndex>>
    _obstacleIndexes;
    mutable SimTK::ResetOnCopy<SimTK::Array_<SimTK::CableSpanViaPointIndex>>
    _viaPointIndexes;

    class MomentArmSolver {
    public:
        MomentArmSolver(const Model& model);
        double solve(const SimTK::State& state, const Coordinate& coordinate,
            const Scholz2015GeometryPath& path) const;

        const Model& getModel() const { return *_model; }

    private:
        SimTK::ReferencePtr<const Model> _model;
        mutable SimTK::State _state;
    };
    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver>> _maSolver;
};


} // namespace OpenSim

#endif // OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
