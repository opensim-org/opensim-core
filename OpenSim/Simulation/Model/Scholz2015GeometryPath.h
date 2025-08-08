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

#include <OpenSim/Simulation/Model/AbstractGeometryPath.h>

#include <OpenSim/Simulation/Model/Station.h>
#include <OpenSim/Simulation/Model/ContactGeometry.h>
#include <OpenSim/Simulation/MomentArmSolver.h>

#include <simbody/internal/CableSpan.h>
namespace OpenSim {

/**
 * \section Scholz2015GeometryPathObstacle
 * A class representing an obstacle in a `Scholz2015GeometryPath`.
 *
 * A path obstacle consists of a `Socket` connected to a `ContactGeometry` and
 * an initial guess for the wrapping solver via the `contact_hint` property. The
 * `ContactGeometry` is used internally to provide an equivalent
 * `SimTK::ContactGeometry` which is needed by `SimTK::CableSpan` to define a
 * wrapping surface. The contact hint is a `SimTK::Vec3` defining a
 * point on the surface, in the surface coordinates, which `SimTK::CableSpan`
 * uses to shoot a zero-length geodesic to initialize the wrapping solver. The
 * contact hint need not lie on the surface, nor does it need to belong to a
 * valid (i.e., converged) wrapping path.
 *
 * Users should not create instances of this class directly. Instead, use the
 * `addObstacle()` method of `Scholz2015GeometryPath` to add obstacles.
 *
 * @see Scholz2015GeometryPathSegment
 * @see Scholz2015GeometryPath
 */
class OSIMSIMULATION_API Scholz2015GeometryPathObstacle : public Component {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPathObstacle, Component);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(contact_geometry, ContactGeometry,
            "The contact geometry representing the obstacle.");

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
    /**
     * Get the ContactGeometry associated with this obstacle.
     */
    const ContactGeometry& getContactGeometry() const;
};

/**
 * \section Scholz2015GeometryPathSegment
 * A class representing a segment in a `Scholz2015GeometryPath`.
 *
 * A path segment consists of two `Socket`s connected to `Station`s that
 * define the origin and insertion of the segment. These `Station`s should
 * typically be owned by the `Scholz2015GeometryPath` that this path segment
 * belongs to. Path segments also store a list of
 * `Scholz2015GeometryPathObstacle`s via the `obstacles` property. The order of
 * the obstacles in the list is important, as it defines the order in which
 * the path wraps over the obstacles.
 *
 * Users should not create instances of this class directly, nor should they
 * need to explicitly add and manage path segments within a
 * `Scholz2015GeometryPath`. Instead, path segments are automatically created
 * upon the construction of a `Scholz2015GeometryPath` and when via points are
 * added.
 *
 * @see Scholz2015GeometryPathObstacle
 * @see Scholz2015GeometryPath
 */
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
    /**
     * Get the number of `Scholz2015GeometryPathObstacle`s in the path segment.
     */
    int getNumObstacles() const;

    /**
     * Get the origin `Station` of the path segment.
     */
    const Station& getOrigin() const;

    /**
     * Get the insertion `Station` of the path segment.
     */
    const Station& getInsertion() const;
};

//=============================================================================
//                       SCHOLZ 2015 GEOMETRY PATH
//=============================================================================

/**
 * \section Scholz2015GeometryPath
 * A concrete class representing a path object that begins at an origin point
 * fixed to a body, wraps over geometric obstacles and passes through
 * frictionless via points fixed to other bodies, and terminates at an insertion
 * point.
 *
 * The path consists of straight line segments and curved line segments: a
 * curved segment over each obstacle, and straight segments connecting them to
 * each other and to via points and the end points. Each curved segment is
 * computed as a geodesic to give (in some sense) a shortest path over the
 * surface. During a simulation the cable can slide freely over the obstacle
 * surfaces. It can lose contact with a surface and miss that obstacle in a
 * straight line. Similarly the cable can touchdown on the obstacle if the
 * surface obstructs the straight line segment again. The cable will always
 * slide freely through any via points present in the path.
 *
 * The path is computed as an optimization problem using the previous optimal
 * path as the warm start. This is done by computing natural geodesic
 * corrections for each curve segment to compute the locally shortest path,
 * as described in the following publication:
 *
 *     Scholz, A., Sherman, M., Stavness, I. et al (2015). A fast multi-obstacle
 *     muscle wrapping method using natural geodesic variations. Multibody
 *     System Dynamics 36, 195–219.
 *
 * The overall path is locally the shortest, allowing winding over an obstacle
 * multiple times, without flipping to the other side.
 *
 * This class encapsulates `SimTK::CableSpan`, the Simbody implementation of
 * this algorithm. For the full details concerning this class, see the Simbody
 * API documentation.
 *
 * ## Constructing a Scholz2015GeometryPath
 *
 * The simplest path consists of a single line segment from an origin to an
 * insertion point:
 *
 * \code{.cpp}
 * Model model = ModelFactory::createDoublePendulum();
 * Scholz2015GeometryPath* path = new Scholz2015GeometryPath(
 *           model.getGround(), SimTK::Vec3(0.25, 0, 0),
 *           model.getComponent<Body>("/bodyset/b1"), SimTK::Vec3(-0.5, 0.1, 0));
 * model.addComponent(path);
 * \endcode
 *
 * Typically, a Scholz2015GeometryPath will be used to define the geometry of
 * path-based force generating elements, e.g., `PathSpring`, `PathActuator`,
 * `Muscle`, etc. This example shows how to create a `PathSpring` and set the
 * 'path' property to a Scholz2015GeometryPath:
 *
 * \code{.cpp}
 * auto* spring = new PathSpring();
 * spring->setName("path_spring");
 * spring->setRestingLength(0.5);
 * spring->setDissipation(0.5);
 * spring->setStiffness(25.0);
 * spring->set_path(Scholz2015GeometryPath());
 * model.addComponent(spring);
 *
 * auto& path = spring->updPath<Scholz2015GeometryPath>();
 * path.setName("path");
 * path.setOrigin(model.getGround(), SimTK::Vec3(0.05, 0.05, 0));
 * path.setInsertion(model.getComponent<Body>("/bodyset/b1"),
 *                   SimTK::Vec3(-0.25, 0.1, 0));
 * \endcode
 *
 * The origin and insertion points are stored using `Station` properties.
 * Therefore, we update these properties *after* the `Scholz2015GeometryPath`
 * has been added to the `PathSpring`, so that the `Socket` connections in each
 * `Station` remain valid. If we called `set_path()` after each `Station`
 * property was set, the `Socket` connections would be broken when
 * the `Scholz2015GeometryPath` is copied into the `PathSpring`.
 *
 * ## Adding Wrap Obstacles
 *
 * Wrap obstacles are defined by `ContactGeometry` objects which encapsulate
 * a wrapping geometry, the `PhysicalFrame` that the geometry is attached to,
 * and the location and orientation of the geometry in that frame. The following
 * concrete implementations of `ContactGeometry` are recommended for use with
 * Scholz2015GeometryPath:
 * - `ContactSphere`
 * - `ContactCylinder`
 * - `ContactEllipsoid`
 * - `ContactTorus`
 *
 * Use `addObstacle()` to add a `ContactGeometry` wrapping obstacle to path,
 * along with a "contact hint" that is used to initialize the wrapping solver:
 *
 * \code{.cpp}
 * auto* obstacle = new ContactCylinder(0.1,
 *         SimTK::Vec3(-0.5, 0.1, 0), SimTK::Vec3(0),
 *         model.getComponent<Body>("/bodyset/b0"));
 * model.addComponent(obstacle);
 * path.addObstacle(*obstacle, SimTK::Vec3(0., 0.1, 0.));
 * \endcode
 *
 * The contact hint is a `SimTK::Vec3` defining a point on the surface in local
 * surface frame coordinates. The point will be used as a starting point when
 * computing the initial cable path. As such, it does not have to lie on the
 * contact geometry's surface, nor does it have to belong to a valid cable path.
 *
 * ## Adding Via Points
 *
 * Via points are `Station`s owned by `Scholz2015GeometryPath` that the path
 * will freely slide through. Use the `addViaPoint()` method to provide both the
 * `PhysicalFrame` and body-fixed location of a new via point in the path:
 *
 * \code{.cpp}
 * path.addViaPoint(model.getComponent<Body>("/bodyset/b1"),
 *         SimTK::Vec3(-0.75, 0.1, 0.));
 * \endcode
 *
 * The full set of via points is stored in the `via_points` list property.
 *
 * When via points are present, the path is internally divided into
 * multiple path segments. The origin and insertion of each segment are defined
 * by `Socket`s which are connected to either the `origin` or `insertion`
 * properties of `Scholz2015GeometryPath`, or to the `Station` of a via point.
 * Path segments are automatically created and managed internally using
 * `Scholz2015GeometryPathSegment`, which also provides the means for
 * serializing and deserializing `Scholz2015GeometryPath` objects while
 * preserving the correct order of wrapping obstacles and via points. Users need
 * not create or manage `Scholz2015GeometryPathSegment` objects directly.
 *
 * ## Path Ordering
 *
 * The order in which obstacles and via points are added to the path is
 * important. For example, a `Scholz2015GeometryPath` could be constructed with
 * two wrapping obstacles separated by a via point, as follows:
 *
 * \code{.cpp}
 * path.addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
 * path.addViaPoint(model.getComponent<Body>("/bodyset/body"), SimTK::Vec3(0));
 * path.addObstacle(*sphere, SimTK::Vec3(0., 0.5, 0.));
 * \endcode
 *
 * By changing the order of the `addObstacle()` and `addViaPoint()` calls, we
 * could define a different path where the two wrap obstacles exist in the same
 * path segment:
 *
 * \code{.cpp}
 * path.addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
 * path.addObstacle(*sphere, SimTK::Vec3(0., 0.5, 0.));
 * path.addViaPoint(model.getComponent<Body>("/bodyset/body"), SimTK::Vec3(0));
 * \endcode
 *
 * Both paths will have two internal path segments since one via point was
 * added in each. However, in the first path, each segment will have one wrap
 * obstacle, while in the second path, the first segment will have two wrap
 * obstacles and the second segment will have no wrap obstacles.
 *
 * @see Scholz2015GeometryPathSegment
 * @see Scholz2015GeometryPathObstacle
 */
class OSIMSIMULATION_API Scholz2015GeometryPath : public AbstractGeometryPath {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPath, AbstractGeometryPath);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(origin, Station,
        "The origin station of the path.");
    OpenSim_DECLARE_PROPERTY(insertion, Station,
        "The insertion station of the path.");
    OpenSim_DECLARE_LIST_PROPERTY(via_points, Station,
        "The list of via points that the path passes through.");
    OpenSim_DECLARE_LIST_PROPERTY(segments, Scholz2015GeometryPathSegment,
        "The list of path segments.");
    OpenSim_DECLARE_PROPERTY(algorithm, std::string,
        "The algorithm used to compute the path. Options: 'Scholz2015' "
        "(default) or 'MinimumLength'.");
    OpenSim_DECLARE_PROPERTY(curve_segment_accuracy, double,
        "Set the accuracy used by the numerical integrator when computing a "
        "geodesic over an obstacle (default: 1e-9).");
    OpenSim_DECLARE_PROPERTY(smoothness_tolerance, double,
        "The smoothness tolerance used to compute the optimal path "
        "(default: 0.1 degrees).");
    OpenSim_DECLARE_PROPERTY(solver_max_iterations, int,
        "The maximum number of solver iterations for finding the optimal path "
        "(default: 50).");

//=============================================================================
// METHODS
//=============================================================================

    /** Default constructor. */
    Scholz2015GeometryPath();

    /**
     * Construct a Scholz2015GeometryPath with the specified origin and
     * insertion.
     *
     * @param originFrame       the origin's PhysicalFrame.
     * @param originLocation    the origin location of in `originFrame`.
     * @param insertionFrame    the insertion's PhysicalFrame.
     * @param insertionLocation the insertion location in `insertionFrame`.
     */
    Scholz2015GeometryPath(
            const PhysicalFrame& originFrame,
            const SimTK::Vec3& originLocation,
            const PhysicalFrame& insertionFrame,
            const SimTK::Vec3& insertionLocation);

    //** @name Path configuration */
    // @{
    /**
     * %Set the origin `Station` of the path.
     *
     * @param frame        the PhysicalFrame that the origin is attached to.
     * @param location     the location of the origin in `frame`.
     */
    void setOrigin(const PhysicalFrame& frame, const SimTK::Vec3& location);

    /**
     * Get the origin `Station` of the path.
     */
    const Station& getOrigin() const;

    /**
     * %Set the insertion `Station` of the path.
     *
     * @param frame        the PhysicalFrame that the insertion is attached to.
     * @param location     the location of the insertion in `frame`.
     */
    void setInsertion(const PhysicalFrame& frame, const SimTK::Vec3& location);

    /**
     * Get the insertion `Station` of the path.
     */
    const Station& getInsertion() const;

    /**
     * Add an obstacle to the path.
     *
     * The provided `ContactGeometry` is used internally to provide an
     * equivalent `SimTK::ContactGeometry` which is used by the underlying
     * `SimTK::CableSpan` to define a wrapping surface. The contact hint is a
     * `SimTK::Vec3` defining a point on the surface, in the surface coordinates,
     * which `SimTK::CableSpan` uses to shoot a zero-length geodesic to
     * initialize the wrapping solver. The contact hint need not lie on the
     * surface, nor does it need to belong to a valid (i.e., converged) wrapping
     * path.
     *
     * @param contactGeometry  the ContactGeometry representing the obstacle.
     * @param contactHint      the point on the contact geometry surface, in
     *                         surface coordinates, that is used to initialize
     *                         the wrapping solver.
     *
     * @see Scholz2015GeometryPathObstacle
     */
    void addObstacle(const ContactGeometry& contactGeometry,
            const SimTK::Vec3& contactHint);

    /**
     * Get the number of obstacles in the path.
     */
    int getNumObstacles() const;

    /**
     * Add a via point to the path.
     *
     * The via point is added as a `Station` that is connected to the last path
     * segment's insertion. The via point is automatically connected to the
     * list of via points in the path.
     *
     * @param frame        the PhysicalFrame that the via point is attached to.
     * @param location     the location of the via point in `frame`.
     */
    void addViaPoint(const PhysicalFrame& frame, const SimTK::Vec3& location);

    /**
     * Get the number of via points in the path.
     */
    int getNumViaPoints() const;
    // @}

    //** @name Solver configuration */
    // @{

    /**
     * %Set the algorithm used to optimize the path.
     *
     * The choice of algorithm changes the cost function and the computed
     * descending direction for reaching the optimal path.
     * The algorithm can be set to either "Scholz2015" (default) or
     * "MinimumLength". The "Scholz2015" algorithm uses the original algorithm
     * published in Scholz et al. (2015) which drives the error between straight
     * line path segments and curved path segments over obstacles to zero.
     * The "MinimumLength" algorithm finds an optimal path by minimizing the
     * total path length directly while also enforcing the path error
     * constraints.
     *
     * See `SimTK::CableSpan::CableSpanAlgorithm` for more details.
     */
    void setAlgorithm(std::string algorithm);

    /**
     * Get the algorithm used to optimize the path.
     */
    const std::string& getAlgorithm() const;

    /**
     * %Set the accuracy used by the numerical integrator when computing a
     * geodesic over an obstacle.
     */
    void setCurveSegmentAccuracy(double accuracy);

    /**
     * Get the accuracy used by the numerical integrator when computing a
     * geodesic over an obstacle.
     */
    double getCurveSegmentAccuracy() const;

    /**
     * %Set the smoothness tolerance used to compute the optimal path.
     *
     * The (non) smoothness is defined as the angular discontinuity at the points
     * where straight- and curved-line segments meet, measured in radians. When
     * computing the optimal path this smoothness is optimized, and the solver
     * stops when reaching given tolerance.
     */
    void setSmoothnessTolerance(double tolerance);

    /**
     * Get the smoothness tolerance used to compute the optimal path.
     */
    double getSmoothnessTolerance() const;

    /** Get the smoothness of the current cable's path.
     *
     * If via points are present in the cable, the maximum smoothness error
     * across all cable segments is returned.
     *
     * State must be realized to Stage::Position.
     *
     * @see CableSpan::getSmoothnessTolerance.
     */
    double getSmoothness(const SimTK::State& state) const;

    /**
     * Get the maximum number of solver iterations for finding the optimal path.
     */
    int getSolverMaxIterations() const;

    /**
     * Set the maximum number of solver iterations for finding the optimal path.
     */
    void setSolverMaxIterations(int maxIterations);

    /**
     * Get the number of solver iterations used to compute the current cable's
     * path.
     *
     * If via points are present in the cable, the sum of the iterations used to
     * compute the path for each cable segment is returned.
     *
     * State must be realized to Stage::Position.
     */
    int getNumSolverIterations(const SimTK::State& state) const;

    // @}

    /** @name Curve segment computations */
    // @{

    /**
     * Returns true when the cable is in contact with the obstacle.
     *
     * State must be realized to Stage::Position.
     *
     * @param state State of the system.
     * @param ix The index of the obstacle in the path.
     */
    bool isInContactWithObstacle(const SimTK::State& state,
            SimTK::CableSpanObstacleIndex ix) const;

    /**
     * Compute the Frenet frame associated with the obstacle's curve segment at
     * the initial contact point on that obstacle.
     *
     * If the path is not in contact with the obstacle's surface the frame will
     * contain invalid data (NaNs). The Frenet frame is measured relative to
     * ground, with the tangent along the X axis, the surface normal along the Y
     * axis and the binormal along the Z axis.
     *
     * State must be realized to Stage::Position.
     *
     * @param state State of the system.
     * @param ix The index of the obstacle in the path.
     * @return The Frenet frame at the obstacle's initial contact point.
     */
    SimTK::Transform calcCurveSegmentInitialFrenetFrame(
        const SimTK::State& state,
        SimTK::CableSpanObstacleIndex ix) const;

    /**
     * Compute the Frenet frame associated with the obstacle's curve segment at
     * the final contact point on that obstacle.
     *
     * If the path is not in contact with the obstacle's surface the frame will
     * contain invalid data (NaNs). The Frenet frame is measured relative to
     * ground, with the tangent along the X axis, the surface normal along the Y
     * axis and the binormal along the Z axis.
     *
     * State must be realized to Stage::Position.
     *
     * @param state State of the system.
     * @param ix The index of the obstacle in the path.
     * @return The Frenet frame at the obstacle's final contact point.
     */
    SimTK::Transform calcCurveSegmentFinalFrenetFrame(
        const SimTK::State& state,
        SimTK::CableSpanObstacleIndex ix) const;

    /**
     * Get the arc length of the obstacle's curve segment.
     *
     * Returns NaN if the obstacle is not in contact with the path.
     *
     * State must be realized to Stage::Position.
     *
     * @param state State of the system.
     * @param ix The index of the obstacle in the path.
     * @return The arc length.
     */
    SimTK::Real calcCurveSegmentArcLength(
        const SimTK::State& state,
        SimTK::CableSpanObstacleIndex ix) const;

    // @}

    /** @name Via point computations */
    // @{

    /**
     * Compute the location of a via point in the ground frame.
     *
     * State must be realized to Stage::Position.
     *
     * @param state State of the system.
     * @param ix The index of the via point in the path.
     * @return The location of the via point in the ground frame.
     */
    SimTK::Vec3 calcViaPointLocation(
        const SimTK::State& state,
        SimTK::CableSpanViaPointIndex ix) const;

    // @}

    //** @name `AbstractGeometryPath` interface */
    // @{
    double getLength(const SimTK::State& s) const override;
    double getLengtheningSpeed(const SimTK::State& s) const override;
    double computeMomentArm(const SimTK::State& s,
            const Coordinate& coord) const override;
    bool isVisualPath() const override { return true; }
    // @}

    //** @name `ForceProducer` interface */
    // @{
    void produceForces(const SimTK::State& s, double tension,
            ForceConsumer& consumer) const override;
    // @}

private:
    // MODEL COMPONENT INTERFACE
    void extendFinalizeFromProperties() override;
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

    using ObstacleIndexMap = std::unordered_map<int,
            SimTK::ReferencePtr<const PhysicalFrame>>;
    mutable SimTK::ResetOnCopy<ObstacleIndexMap> _obstacleIndexMap;
    using ViaPointIndexMap = std::unordered_map<int,
            SimTK::ReferencePtr<const PhysicalFrame>>;
    mutable SimTK::ResetOnCopy<ViaPointIndexMap> _viaPointIndexMap;

    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver> > _maSolver;
    SimTK::CableSpanAlgorithm _algorithm;
};


} // namespace OpenSim

#endif // OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
