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
 * \section Scholz2015GeometryPathPoint
 * An abstract class representing an element in a `Scholz2015GeometryPath`.
 *
 * Concrete implementations of this class should represent geometric elements
 * in a path (e.g., path points and obstacles). Path elements of a
 * `Scholz2015GeometryPath` are stored as an ordered list in the `path_elements`
 * property.
 *
 * @see Scholz2015GeometryPathPoint
 * @see Scholz2015GeometryPathObstacle
 * @see Scholz2015GeometryPath
 */
class OSIMSIMULATION_API Scholz2015GeometryPathElement : public Component {
OpenSim_DECLARE_ABSTRACT_OBJECT(Scholz2015GeometryPathElement, Component);
};

/**
 * \section Scholz2015GeometryPathPoint
 * A class representing a point in a `Scholz2015GeometryPath`.
 *
 * A path point is represented by a `Station`, which defines a point on a
 * `PhysicalFrame` in the model. Users should not create instances of this class
 * directly. Instead, use the `addPathPoint()` method of
 * `Scholz2015GeometryPath` to add path points.
 *
 * @see Scholz2015GeometryPath
 */
class OSIMSIMULATION_API Scholz2015GeometryPathPoint :
        public Scholz2015GeometryPathElement {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPathPoint,
        Scholz2015GeometryPathElement);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_PROPERTY(station, Station,
            "The Station representing this path point.");

//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTOR(S)
    Scholz2015GeometryPathPoint();

    // ACCESSOR(S)
    /**
     * Get the `Station` representing this path point.
     */
    const Station& getStation() const;
};

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
 * @see Scholz2015GeometryPath
 */
class OSIMSIMULATION_API Scholz2015GeometryPathObstacle :
        public Scholz2015GeometryPathElement {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPathObstacle,
        Scholz2015GeometryPathElement);
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
    // CONSTRUCTOR(S)
    Scholz2015GeometryPathObstacle();

    // ACCESSOR(S)
    /**
     * Get the ContactGeometry associated with this obstacle.
     */
    const ContactGeometry& getContactGeometry() const;

    /**
     * Get the contact hint for the obstacle.
     */
    const SimTK::Vec3& getContactHint() const;
};

//=============================================================================
//                       SCHOLZ 2015 GEOMETRY PATH
//=============================================================================

/**
 * \section Scholz2015GeometryPath
 * A concrete class representing a geometric path object defined by a list of
 * path points and wrapping obstacles.
 *
 * The path consists of straight line segments and curved line segments: a
 * curved segment over each obstacle, and straight segments connecting path
 * points to obstacles. If no obstacle lies between two path points, the points
 * will be connected by a straight line segment. The path is computed as a
 * geodesic to provide a shortest path over the surface. During a simulation,
 * the cable can slide freely over the obstacle surfaces. It can lose contact
 * with a surface and miss that obstacle in a straight line. Similarly the cable
 * can touchdown on the obstacle if the surface obstructs the straight line
 * segment again.
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
 * The simplest valid path consists of two path points: an origin and an
 * insertion point:
 *
 * \code{.cpp}
 * Model model = ModelFactory::createDoublePendulum();
 * Scholz2015GeometryPath* path = new Scholz2015GeometryPath();
 * path.addPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b0"),
 *         SimTK::Vec3(-0.5, 0.1, 0.));
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
 * spring->setRestingLength(0.25);
 * spring->setDissipation(0.75);
 * spring->setStiffness(10.0);
 * spring->set_path(Scholz2015GeometryPath());
 * model.addComponent(spring);
 *
 * auto& path = spring->updPath<Scholz2015GeometryPath>();
 * path.setName("path");
 * path.addPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0));
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b1"),
 *         SimTK::Vec3(-0.25, 0.1, 0));
 * \endcode
 *
 * @note The path points are stored using `Station` properties. Therefore, we
 * add path points *after* the `Scholz2015GeometryPath` has been added to the
 * `PathSpring`, so that the connections for each `Station`'s parent frame
 * `Socket` remain valid. If we instead called `set_path()` after each `Station`
 * property was set, the `Socket` connections would be broken when the
 * `Scholz2015GeometryPath` is copied into the `PathSpring`.
 *
 * ## Adding Wrap Obstacles
 *
 * Wrap obstacles are defined by `ContactGeometry` objects which encapsulate
 * a wrapping geometry, the `PhysicalFrame` that the geometry is attached to,
 * and the location and orientation of the geometry in that frame. The following
 * concrete implementations of `ContactGeometry` are recommended for use with
 * `Scholz2015GeometryPath`:
 * - `ContactSphere`
 * - `ContactCylinder`
 * - `ContactEllipsoid`
 * - `ContactTorus`
 *
 * Use `addObstacle()` to add a `ContactGeometry` wrapping obstacle to path,
 * along with a "contact hint" that is used to initialize the wrapping solver:
 *
 * \code{.cpp}
 * auto* obstacle = new ContactCylinder(0.15,
 *         SimTK::Vec3(-0.2, 0.2, 0), SimTK::Vec3(0),
 *         model.getComponent<Body>("/bodyset/b0"));
 * model.addComponent(obstacle);
 * path.addObstacle(*obstacle, SimTK::Vec3(0., 0.15, 0.));
 * \endcode
 *
 * The contact hint is a `SimTK::Vec3` defining a point on the surface in local
 * surface frame coordinates. The point will be used to provide an initial guess
 * when solving for the path. As such, it does not have to lie on the contact
 * geometry's surface, nor does it have to belong to a valid cable path. For
 * obstacles with symmetry, the choice of the contact hint will determine which
 * side of the obstacle the path will wrap around.
 *
 * A `Scholz2015GeometryPath` must begin and end with a path point. Since we
 * added an obstacle, we must add an additional path point to "close" the path:
 *
 * \code{.cpp}
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b1"),
 *         SimTK::Vec3(-0.5, 0.1, 0.));
 * \endcode
 *
 * ## Path Ordering
 *
 * The order in which obstacles and path points are added to the path is
 * important. For example, consider the path we constructed above:
 *
 * \code{.cpp}
 * path.addPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b0"),
 *         SimTK::Vec3(-0.5, 0.1, 0.));
 * path.addObstacle(*obstacle, SimTK::Vec3(0., 0.15, 0.));
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b1"),
 *         SimTK::Vec3(-0.5, 0.1, 0.));
 * \endcode
 *
 * Using the order of calls above, we have explicitly defined a cylinder
 * obstacle to apply only to the portion of the path between the second and
 * third path points.
 *
 * By changing the order of the `addObstacle()` and `addPathPoint()` calls, we
 * could define a different path where a cylinder obstacle is applied to the
 * portion of the path between the first and second path points:
 *
 * \code{.cpp}
 * path.addPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
 * path.addObstacle(*obstacle, SimTK::Vec3(0., 0.15, 0.));
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b0"),
 *         SimTK::Vec3(-0.5, 0.1, 0.));
 * path.addPathPoint(model.getComponent<Body>("/bodyset/b1"),
 *         SimTK::Vec3(-0.5, 0.1, 0.));
 * \endcode
 *
 * The location of the cylinder obstacle in this new path would likely need to
 * be updated to a new location that is consistent with the portion of the path
 * between the first and second path points to produce the desired wrapping
 * behavior. Note also that both of the path examples above are valid, since
 * each begins and ends with a path point.
 *
 * @see Scholz2015GeometryPathPoint
 * @see Scholz2015GeometryPathObstacle
 */
class OSIMSIMULATION_API Scholz2015GeometryPath : public AbstractGeometryPath {
OpenSim_DECLARE_CONCRETE_OBJECT(Scholz2015GeometryPath, AbstractGeometryPath);

public:
//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_LIST_PROPERTY(path_elements, Scholz2015GeometryPathElement,
        "The list of elements (path points or obstacles) defining the path.");
    OpenSim_DECLARE_PROPERTY(algorithm, std::string,
        "The algorithm used to compute the path. Options: 'Scholz2015' "
        "(default) or 'MinimumLength'.");

//=============================================================================
// METHODS
//=============================================================================

    /** Default constructor. */
    Scholz2015GeometryPath();

    //** @name Path configuration */
    // @{
    /**
     * Add a path point to the path.
     */
    void addPathPoint(const PhysicalFrame& frame, const SimTK::Vec3& location);

    /**
     * Get the origin `Station` of the path (i.e., the first path point).
     *
     * @pre At least one path point has been added to the path.
     */
    const Station& getOrigin() const;

    /**
     * Get the insertion `Station` of the path (i.e., the last path point).
     *
     * @pre At least two path points have been added to the path.
     */
    const Station& getInsertion() const;

    /**
     * Add an obstacle to the path.
     *
     * The contact hint is a `SimTK::Vec3` defining a point on the surface, in
     * the surface coordinates, used to initialize the wrapping solver. The
     * contact hint need not lie on the surface, nor does it need to belong to a
     * valid wrapping path.
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
     * Get the number of path points in the path.
     */
    int getNumPathPoints() const;

    /**
     * Get the number of obstacles in the path.
     */
    int getNumObstacles() const;

    /**
     * Get the total number of path elements (path points and obstacles) in the
     * path.
     */
    int getNumPathElements() const;

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
    void extendConnectToModel(Model& model) override;
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& s,
            SimTK::Array_<SimTK::DecorativeGeometry>& geoms) const override;

    // CONVENIENCE METHODS
    void constructProperties();

    // Get a non-modifiable reference to a path element based on the element
    // index and type. Use this if you certain of the element type; otherwise,
    // use tryGetElement().
    template <typename ElementType>
    const ElementType& getElement(int index) const {
        OPENSIM_ASSERT_ALWAYS(index >= 0 && index < getNumPathElements());
        return dynamic_cast<const ElementType&>(get_path_elements(index));
    }

    // Get a modifiable reference to a path element based on the element index
    // and type. Use this if you certain of the element type; otherwise, use
    // tryUpdElement().
    template <typename ElementType>
    ElementType& updElement(int index) {
        OPENSIM_ASSERT_ALWAYS(index >= 0 && index < getNumPathElements());
        return dynamic_cast<ElementType&>(upd_path_elements(index));
    }

    // Attempt to get a non-modifiable pointer to a path element based on the
    // element index and type. Returns nullptr if the element is not of the
    // requested type.
    template <typename ElementType>
    const ElementType* tryGetElement(int index) const {
        OPENSIM_ASSERT_ALWAYS(index >= 0 && index < getNumPathElements());
        return dynamic_cast<const ElementType*>(&get_path_elements(index));
    }

    // Attempt to get a modifiable pointer to a path element based on the
    // element index and type. Returns nullptr if the element is not of the
    // requested type.
    template <typename ElementType>
    ElementType* tryUpdElement(int index) {
        OPENSIM_ASSERT_ALWAYS(index >= 0 && index < getNumPathElements());
        return dynamic_cast<ElementType*>(&upd_path_elements(index));
    }

    // Internal mutable access to the underlying SimTK::CableSpan.
    const SimTK::CableSpan& getCableSpan() const;
    SimTK::CableSpan& updCableSpan();

    // MEMBER VARIABLES
    mutable SimTK::ResetOnCopy<SimTK::CableSpanIndex> _index;
    using ObstacleIndexes = std::vector<std::tuple<
            SimTK::CableSpanObstacleIndex, int>>;
    mutable SimTK::ResetOnCopy<ObstacleIndexes> _obstacleIndexes;
    using ViaPointIndexes = std::vector<std::pair<
            SimTK::CableSpanViaPointIndex, int>>;
    mutable SimTK::ResetOnCopy<ViaPointIndexes> _viaPointIndexes;

    SimTK::ResetOnCopy<std::unique_ptr<MomentArmSolver> > _maSolver;
};


} // namespace OpenSim

#endif // OPENSIM_SCHOLZ_2015_GEOMETRY_PATH_H
