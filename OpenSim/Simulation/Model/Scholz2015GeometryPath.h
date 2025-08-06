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
 * /section Scholz2015GeometryPathObstacle
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
 * /section Scholz2015GeometryPathSegment
 * A class representing a segment in a Scholz2015GeometryPath.
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
/** This class represents the path of a frictionless cable from an origin point
fixed to a body, over geometric obstacles and through via points fixed to other 
bodies, to a final termination point.

The CableSpan's path can be seen as consisting of straight line segments and
curved line segments: A curved segment over each obstacle, and straight segments
connecting them to each other and to via points and the end points. Each curved 
segment is computed as a geodesic to give (in some sense) a shortest path over 
the surface. During a simulation the cable can slide freely over the obstacle 
surfaces. It can lose contact with a surface and miss that obstacle in a 
straight line. Similarly the cable can touchdown on the obstacle if the surface 
obstructs the straight line segment again. The cable will always slide freely 
through any via points present in the path.

The path is computed as an optimization problem using the previous optimal path
as the warm start. This is done by computing natural geodesic corrections for
each curve segment to compute the locally shortest path, as described in the
following publication:

    Scholz, A., Sherman, M., Stavness, I. et al (2015). A fast multi-obstacle
    muscle wrapping method using natural geodesic variations. Multibody System
    Dynamics 36, 195–219.

The overall path is locally the shortest, allowing winding over an obstacle
multiple times, without flipping to the other side.

During initialization the path is assumed to be in contact with each obstacle at
the user defined contact-point-hint. At each contact point a
zero-length-geodesic is computed with the tangent estimated as pointing from the
previous path point to the next path point. From this configuration, the solver
is started, and the geodesics will be corrected until the entire path is smooth.

Important to note is that the cable's interaction with the obstacles is ordered
based on the order in which they were added. That is, if three obstacles are
added to a cable, then, the cable can wrap over the first, then the second, and
then the third. If the first obstacle spatially collides with the path twice it
will not actually wrap over it twice. Use CablePath if this is not the desired
behavior.

When via points are present in the cable, the cable is internally divided into 
cable segments separated by the via points. Obstacles separated by via points 
have no interaction when geodesic corrections are applied during optimization,
and therefore the path for each cable segment is solved independently. This 
approach is advantageous since cable segments with different sets of wrap 
obstacles may require a different number of optimization iterations to converge.

Note that a CableSpan is a geometric object, not a force or constraint element.
That is, a CableSpan alone will not influence the behavior of a simulation.
However, forces and constraint elements can be constructed that make use of a
CableSpan to generate forces.

A CableSpan must be registered with a CableSubsystem which manages their
runtime evaluation. **/


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
 * insertion point. 
 *
 * \code{.cpp}
 * Model model = ModelFactory::createDoublePendulum();
 * Scholz2015GeometryPath* path = new Scholz2015GeometryPath(
 *           model.getGround(), SimTK::Vec3(0.25, 0, 0),
 *           model.getComponent<Body>("/bodyset/b1"), SimTK::Vec3(-0.5, 0.1, 0));
 * model.addComponent(path);
 * \endcode
 *
 * \code{.cpp}
 * auto* actu = new PathActuator();
 * actu->set_path(Scholz2015GeometryPath());
 * model.addComponent(actu);
 *
 * auto& path = actu->updPath<Scholz2015GeometryPath>();
 * path.setOrigin(model.getGround(), SimTK::Vec3(0.25, 0, 0));
 * path.setInsertion(model.getComponent<Body>("/bodyset/b1"), 
 *                   SimTK::Vec3(-0.5, 0.1, 0));
 * \endcode
 *
 * ## Adding Wrap Obstacles
 * 
 * \code{.cpp}
 * auto* ellipsoid = new ContactEllipsoid(SimTK::Vec3(0.1, 0.1, 0.3),
 *         SimTK::Vec3(0., 0.2, 0), SimTK::Vec3(0), model.getGround());
 * model.addComponent(ellipsoid);
 *
 * path.addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
 * \endcode
 *
 * ## Adding Via Points
 *
 * \code{.cpp}
 * path.addViaPoint(model.getComponent<Body>("/bodyset/body"), 
 *         SimTK::Vec3(0.1, 0., 0.));
 * \endcode
 *
 * ## Path Ordering
 * 
 * \code{.cpp}
 * path.addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
 * path.addViaPoint(model.getComponent<Body>("/bodyset/body"), SimTK::Vec3(0));
 * path.addObstacle(*sphere, SimTK::Vec3(0., 0.5, 0.));
 * \endcode
 *
 * \code{.cpp}
 * path.addObstacle(*ellipsoid, SimTK::Vec3(0.1, 0., 0.));
 * path.addObstacle(*sphere, SimTK::Vec3(0., 0.5, 0.));
 * path.addViaPoint(model.getComponent<Body>("/bodyset/body"), SimTK::Vec3(0));
 * \endcode
 *
 * @see Scholz2015GeometryPathSegment
 * @see Scholz2015GeometryPathObstacle
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
    OpenSim_DECLARE_LIST_SOCKET(via_points, Station,
        "The list of via points that the path passes through.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(stations, Station,
        "The list of stations defining the path including the origin, "
        "insertion, and any via points.");
    OpenSim_DECLARE_LIST_PROPERTY(segments, Scholz2015GeometryPathSegment,
        "The list of path segments.");
    OpenSim_DECLARE_PROPERTY(algorithm, std::string,
        "The algorithm used to compute the path. Options: 'Scholz2015' "
        "(default) or 'MinimumLength'.");

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

    //** @name Accessors */
    // @{
    /**
     * %Set the origin `Station` of the path.
     *
     * An exception is thrown if the origin `Socket` is already connected, i.e.,
     * if the origin was previously set by this method or by a constructor.
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
     * An exception is thrown if the insertion `Socket` is already connected,
     * i.e., if the insertion was previously set by this method or by a
     * constructor.
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

    /**
     * &Set the algorithm used to optimize the path.
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
    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver> > _maSolver;
};


} // namespace OpenSim

#endif // OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
