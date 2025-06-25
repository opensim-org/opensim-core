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
 * Author(s): Andreas Scholz, Pepijn van den Bos                              *
 * Contributor(s): Nicholas Bianco                                            *
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
#include <OpenSim/Simulation/MomentArmSolver.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>

namespace OpenSim {

//=============================================================================
//                    SCHOLZ 2015 GEOMETRY PATH SEGMENT
//=============================================================================
/**
 * TODO
 */
class OSIMSIMULATION_API Scholz2015GeometryPathSegment : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPathSegment, ModelComponent);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(origin, Station, 
            "The origin station of the path segment.");
    OpenSim_DECLARE_SOCKET(insertion, Station,
            "The insertion station of the path segment.");
    OpenSim_DECLARE_LIST_SOCKET(obstacles, ContactGeometry,
            "The list of obstacles that the path segment may intersect.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(contact_hints, SimTK::Vec3,
            "The list of contact hints for the path segment.");

//=============================================================================
// METHODS
//=============================================================================
    
    // CONSTRUCTION
    Scholz2015GeometryPathSegment();

    // GET AND SET
    const Station& getOrigin() const;
    const Station& getInsertion() const;

    int getNumObstacles() const;
    const ContactGeometry& getObstacle(int index) const;

    SimTK::Real getLength(const SimTK::State& s) const;
    SimTK::Real getLengtheningSpeed(const SimTK::State& s) const;

    void calcOriginUnitForce(const SimTK::State& state, 
            SimTK::SpatialVec& unitForce_G) const;
    void calcInsertionUnitForce(const SimTK::State& state, 
            SimTK::SpatialVec& unitForce_G) const;
    void calcCurveSegmentUnitForce(
            const SimTK::State& state,
            SimTK::CableSpanObstacleIndex ix,
            SimTK::SpatialVec& unitForce_G) const;

    bool isInContactWithObstacle(const SimTK::State& state,
            SimTK::CableSpanObstacleIndex ix) const;

private:
    // MODEL COMPONENT INTERFACE
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override;

    // CONVENIENCE METHODS
    void constructProperties();
    const SimTK::CableSpan& getCableSpan() const;
    
    mutable SimTK::ResetOnCopy<SimTK::CableSpanIndex> _index;
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
// METHODS
//=============================================================================

    // CONSTRUCTION
    Scholz2015GeometryPath();

    // GET AND SET
    void createInitialPathSegment(const std::string& name,
            const Station& origin, const Station& insertion);
    void appendPathSegment(const std::string& name,
            const Station& insertion);

    void addObstacleToPathSegment(const std::string& segmentName,
            const ContactGeometry& obstacle, 
            const SimTK::Vec3& contactHint = SimTK::Vec3{SimTK::NaN});

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
    // PROPERTIES
    OpenSim_DECLARE_LIST_PROPERTY(path_segments, Scholz2015GeometryPathSegment,
            "The list of path segments.");

    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    // CONVENIENCE METHODS
    void constructProperties();
    void computeLength(const SimTK::State& s) const;
    void computeSpeed(const SimTK::State& s) const;

    // MEMBER VARIABLES
    std::unordered_map<std::string, int> _segmentNameToIndexMap;

    // CACHE VARIABLES
    mutable CacheVariable<double> _lengthCV;
    mutable CacheVariable<double> _lengtheningSpeedCV;

    // Solver used to compute moment-arms. The Scholz2015GeometryPath owns this 
    // object, but we cannot simply use a unique_ptr because we want the pointer    
    // to be cleared on copy.
    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver>> _maSolver;
};

} // namespace OpenSim

#endif // OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
