#ifndef OPENSIM_CONTACT_GEOMETRY_H_
#define OPENSIM_CONTACT_GEOMETRY_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  ContactGeometry.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/PhysicalFrame.h"
#include "Appearance.h"

namespace OpenSim {

class ScaleSet;

/**
 * \section ContactGeometry
 * A base class that represents the physical shape of an object for use in
 * contact modeling and path wrapping.
 *
 * Concrete implementations of `ContactGeometry` define particular geometric
 * representations (e.g., spheres, cylinders, etc.). The geometry is attached to
 * a PhysicalFrame, which is specified using the Socket "frame". Each concrete
 * implementation provides an equivalent `SimTK::ContactGeometry` which can be
 * accessed via the method `createSimTKContactGeometry()`.
 *
 * @note ContactGeometry is not scaled with the Model.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactGeometry : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(ContactGeometry, ModelComponent);
public:
//=============================================================================
// SOCKETS
//=============================================================================
    OpenSim_DECLARE_SOCKET(frame, PhysicalFrame,
        "The frame to which this contact geometry is attached.");

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
        "The location of the contact geometry center in the PhysicalFrame.");
    OpenSim_DECLARE_PROPERTY(orientation, SimTK::Vec3,
        "The orientation of the contact geometry in the PhysicalFrame in "
        "body-fixed XYZ Euler angles.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "The default appearance for this Geometry");

//=============================================================================
// METHODS
//=============================================================================

    // CONSTRUCTION
    /** Construct an empty ContactGeometry. */
    ContactGeometry();

    /**
     * This constructor connects this ContactGeometry to the provided `frame`,
     * and uses the default location and orientation (both `SimTK::Vec3(0)`).
     *
     * @param frame        the PhysicalFrame to which this geometry is attached.
     */
    explicit ContactGeometry(const PhysicalFrame& frame);

    /**
     * This constructor connects this ContactGeometry to the provided `frame`,
     * and sets the location and orientation properties.
     *
     * @param location     the location of the geometry expressed in `frame`.
     * @param orientation  the orientation of the geometry expressed in `frame`
     *                     as XYZ body-fixed Euler angles.
     * @param frame        the PhysicalFrame to which this geometry is attached;
     *                     this constructor connects this ContactGeometry to
     *                     the provided `frame`.
     */
    ContactGeometry(const SimTK::Vec3& location,
                    const SimTK::Vec3& orientation,
                    const PhysicalFrame& frame);

    //** @name Accessors */
    // @{
    /**
     * Get the PhysicalFrame to which this contact geometry is attached.
     */
    const PhysicalFrame& getFrame() const;

    /**
     * Set the PhysicalFrame to which this contact geometry is attached.
     */
    void setFrame(const PhysicalFrame& frame);

    /**
     * Get a Transform representing the position and orientation of the
     * geometry relative to the PhysicalFrame `F` to which this geometry is
     * connected.
     *
     * If you want the transform of this geometry relative to the Frame (or
     * Ground) `B` in which this geometry is fixed, you can use the following
     * code:
     * @code{.cpp}
     * const auto& X_BF = geom.getFrame().findTransformInBaseFrame();
     * const auto X_FP = geom.getTransform();
     * const auto X_BP = X_BF * X_FP;
     * @endcode
     *
     * Prior to OpenSim 4.0, there wwas no intermediate PhysicalFrame `F`, so
     * this method essentially returned `X_BP`. */
    SimTK::Transform getTransform() const;

    /**
     * Create a new SimTK::ContactGeometry based on this object.
     */
    SimTK::ContactGeometry createSimTKContactGeometry() const;
    // @}

protected:
    // CONTACT GEOMETRY INTERFACE
    // Concrete implementations of ContactGeometry must implement this method to
    // provide an equivalent SimTK::ContactGeometry object.
    virtual SimTK::ContactGeometry createSimTKContactGeometryImpl() const = 0;

    // OBJECT INTERFACE
    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber)
        override;

private:
    // INITIALIZATION
    void setNull();
    void constructProperties();
};

/**
 * \section ContactSphere
 * A class that represents a sphere object for use in contact modeling and path
 * wrapping.
 *
 * A `SimTK::ContactGeometry::Sphere` is constructed when
 * `createSimTKContactGeometry()` is called.
 *
 * @author Peter Eastman
 */
class OSIMSIMULATION_API ContactSphere : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactSphere, ContactGeometry);

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(radius, double,
            "The radius of the sphere (default: 0).");

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactSphere.
     */
    ContactSphere();

    /**
     * Construct a ContactSphere.
     *
     * @param radius       the radius of the sphere.
     * @param location     the location of the center of the sphere expressed
     *                     in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactSphere to
     *                     the provided `frame`.
     */
    ContactSphere(double radius, const SimTK::Vec3& location,
            const PhysicalFrame& frame);

    /**
     * Construct a ContactSphere.
     *
     * @param radius       the radius of the sphere.
     * @param location     the location of the center of the sphere expressed
     *                     in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactSphere to
     *                     the provided `frame`.
     * @param name         the name of this object.
     */
    ContactSphere(double radius, const SimTK::Vec3& location,
            const PhysicalFrame& frame, const std::string& name);

    /** @name Accessors */
    // @{
    /**
     * Get the radius of the sphere.
     */
    double getRadius() const;

    /**
     * %Set the radius of the sphere.
     */
    void setRadius(double radius);
    // @}

    /** @name Visualization */
    // @{
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
    // @}

private:
    // CONTACT GEOMETRY INTERFACE
    SimTK::ContactGeometry createSimTKContactGeometryImpl() const override;
};

/**
 * \section ContactCylinder
 * A class that represents a cylinder object for use in path wrapping.
 *
 * A `SimTK::ContactGeometry::Cylinder` is constructed when
 * `createSimTKContactGeometry()` is called.
 *
 * @note Simbody does not define contact interactions between
 * `SimTK::ContactGeometry::Cylinder` and other contact geometries. Therefore,
 * contact forces that rely on `SimTK::Contact` elements from the
 * `SimTK::GeneralContactSubsystem` (e.g., `HuntCrossleyForce`,
 * `ElasticFoundationForce`, etc.) will ignore `ContactCylinder` objects.
 * `SimTK::ContactGeometry::Cylinder` can be used to define wrapping obstacles
 * (e.g., with `SimTK::CableSpan` and `SimTK::CablePath`).
 */
class OSIMSIMULATION_API ContactCylinder : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactCylinder, ContactGeometry);

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(radius, double,
            "The radius of the cylinder (default: 0).");

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactCylinder.
     */
    ContactCylinder();

    /**
     * Construct a ContactCylinder.
     *
     * @param radius       the radius of the cylinder.
     * @param location     the location of the center of the cylinder expressed
     *                     in `frame`.
     * @param orientation  the orientation of the cylinder expressed in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactCylinder to
     *                     the provided `frame`.
     */
    ContactCylinder(double radius, const SimTK::Vec3& location,
            const SimTK::Vec3& orientation, const PhysicalFrame& frame);

    /**
     * Construct a ContactCylinder.
     *
     * @param radius       the radius of the cylinder.
     * @param location     the location of the center of the cylinder expressed
     *                     in `frame`.
     * @param orientation  the orientation of the cylinder expressed in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactCylinder to
     *                     the provided `frame`.
     * @param name         the name of this object.
     */
    ContactCylinder(double radius, const SimTK::Vec3& location,
            const SimTK::Vec3& orientation, const PhysicalFrame& frame,
            const std::string& name);

    /** @name Accessors */
    // @{
    /**
     * Get the radius of the cylinder.
     */
    double getRadius() const;

    /**
     * %Set the radius of the cylinder.
     */
    void setRadius(double radius);
    // @}

    /** @name Visualization */
    // @{
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
    // @}

private:
    // CONTACT GEOMETRY INTERFACE
    SimTK::ContactGeometry createSimTKContactGeometryImpl() const override;
};

/**
 * \section ContactEllipsoid
 * A class that represents an ellipsoidal object for use in contact modeling and
 * path wrapping.
 *
 * A `SimTK::ContactGeometry::Ellipsoid` is constructed when
 * `createSimTKContactGeometry()` is called.
 */
class OSIMSIMULATION_API ContactEllipsoid : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactEllipsoid, ContactGeometry);

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(radii, SimTK::Vec3,
            "The radii of the ellipsoid (default: [0, 0, 0]).");

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactEllipsoid.
     */
    ContactEllipsoid();

    /**
     * Construct a ContactEllipsoid.
     *
     * @param radii        the radii of the ellipsoid.
     * @param location     the location of the center of the ellipsoid expressed
     *                     in `frame`.
     * @param orientation  the orientation of the ellipsoid expressed in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactEllipsoid to
     *                     the provided `frame`.
     */
    ContactEllipsoid(const SimTK::Vec3& radii, const SimTK::Vec3& location,
            const SimTK::Vec3& orientation, const PhysicalFrame& frame);

    /**
     * Construct a ContactEllipsoid.
     *
     * @param radii        the radii of the ellipsoid.
     * @param location     the location of the center of the ellipsoid expressed
     *                     in `frame`.
     * @param orientation  the orientation of the ellipsoid expressed in `frame`.
     * @param frame        the PhysicalFrame this geometry is attached to;
     *                     this constructor connects this ContactEllipsoid to
     *                     the provided `frame`.
     * @param name         the name of this object.
     */
    ContactEllipsoid(const SimTK::Vec3& radii, const SimTK::Vec3& location,
            const SimTK::Vec3& orientation, const PhysicalFrame& frame,
            const std::string& name);

    /** @name Accessors */
    // @{
    /**
     * Get the radii of the ellipsoid.
     */
    const SimTK::Vec3& getRadii() const;

    /**
     * %Set the radii of the ellipsoid.
     */
    void setRadii(const SimTK::Vec3& radii);
    // @}

    /** @name Visualization */
    // @{
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
    // @}

private:
    // CONTACT GEOMETRY INTERFACE
    SimTK::ContactGeometry createSimTKContactGeometryImpl() const override;
};

/**
 * \section ContactTorus
 * A class that represents a toroidal object for use in contact modeling and
 * path wrapping.
 *
 * A `SimTK::ContactGeometry::Torus` is constructed when
 * `createSimTKContactGeometry()` is called.
 *
 * @note Simbody does not define contact interactions between
 * `SimTK::ContactGeometry::Torus` and other contact geometries. Therefore,
 * contact forces that rely on `SimTK::Contact` elements from the
 * `SimTK::GeneralContactSubsystem` (e.g., `HuntCrossleyForce`,
 * `ElasticFoundationForce`, etc.) will ignore `ContactTorus` objects.
 * `SimTK::ContactGeometry::Cylinder` can be used to define wrapping obstacles
 * (e.g., with `SimTK::CableSpan` and `SimTK::CablePath`).
 */
class OSIMSIMULATION_API ContactTorus : public ContactGeometry {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactTorus, ContactGeometry);

//=============================================================================
// PROPERTIES
//=============================================================================
    OpenSim_DECLARE_PROPERTY(torus_radius, double,
            "The radius of the circular centerline of the torus, measure from "
            "the origin (default: 0).");
    OpenSim_DECLARE_PROPERTY(tube_radius, double,
            "The radius of the torus cross-section: perpendicular distance "
            "from the circular centerline to the surface (default: 0).");

public:
//=============================================================================
// METHODS
//=============================================================================
    // CONSTRUCTION
    /**
     * Construct an empty, uninitialized ContactTorus.
     */
    ContactTorus();

    /**
     * Construct a ContactTorus.
     *
     * @param torus_radius  the radius of the circular centerline of the torus.
     * @param tube_radius   the radius of the torus cross-section.
     * @param location      the location of the center of the torus expressed
     *                      in `frame`.
     * @param orientation   the orientation of the torus expressed in `frame`.
     * @param frame         the PhysicalFrame this geometry is attached to;
     *                      this constructor connects this ContactTorus to
     *                      the provided `frame`.
     */
    ContactTorus(double torus_radius, double tube_radius,
            const SimTK::Vec3& location, const SimTK::Vec3& orientation,
            const PhysicalFrame& frame);

    /**
     * Construct a ContactTorus.
     *
     * @param torus_radius  the radius of the circular centerline of the torus.
     * @param tube_radius   the radius of the torus cross-section.
     * @param location      the location of the center of the torus expressed
     *                      in `frame`.
     * @param orientation   the orientation of the torus expressed in `frame`.
     * @param frame         the PhysicalFrame this geometry is attached to;
     *                      this constructor connects this ContactTorus to
     *                      the provided `frame`.
     * @param name          the name of this object.
     */
    ContactTorus(double torus_radius, double tube_radius,
            const SimTK::Vec3& location, const SimTK::Vec3& orientation,
            const PhysicalFrame& frame, const std::string& name);

    /** @name Accessors */
    // @{
    /**
     * Get the radius of the circular centerline of the torus.
     */
    double getTorusRadius() const;

    /**
     * %Set the radius of the circular centerline of the torus.
     */
    void setTorusRadius(double radius);

    /**
     * Get the radius of the torus cross-section.
     */
    double getTubeRadius() const;

    /**
     * %Set the radius of the torus cross-section.
     */
    void setTubeRadius(double radius);
    // @}

    /** @name Visualization */
    // @{
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
        const SimTK::State& s,
        SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
    // @}

private:
    // CONTACT GEOMETRY INTERFACE
    SimTK::ContactGeometry createSimTKContactGeometryImpl() const override;
};

} // namespace OpenSim

#endif // OPENSIM_CONTACT_GEOMETRY_H_
