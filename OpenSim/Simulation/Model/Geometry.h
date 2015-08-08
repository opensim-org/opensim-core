#ifndef _OPENSIM_GEOMETRY_H_
#define _OPENSIM_GEOMETRY_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Geometry.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

#include "SimTKcommon.h"
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/Set.h>
#include "ModelComponent.h"
#include "Appearance.h"

namespace OpenSim { 

class PhysicalFrame;
class Model;
class Frame;
class ModelComponent;
//=============================================================================
//=============================================================================
/**
 Class Geometry is intended to be used as the base class for all
 geometry that needs to be represented in the system, including mesh files, 
 and built in analytic shapes. Any ModelComponent can specify a list of
 Geometry items to represent itself in graphics window. The relation between
 a ModelComponent and specific Geometry utilizes the Component mechanism, as
 the specific pieces of geometry are treated as subcomponents. The placement
 of the Geometry in 3D spae is computed from the Frame that the Geometry is
 "Connected" to.

 Geometry (and all its subclasses) serve as the set of higher level primitives
 available to OpenSim component writers to express the Geometry of interest.
 The Geometry class handles serialization and also the translation to a set of
 DecorativeGeometry objects that gets passed to the Visualization system to be
 rendered. 

 @author Ayman Habib
 */

// Base Geometry
class OSIMSIMULATION_API Geometry : public Component {
    OpenSim_DECLARE_ABSTRACT_OBJECT(Geometry, Component);
public:
    // Properties common to all Geometry types are included as Properties
    // of the base class.
    // Scale factors
    OpenSim_DECLARE_PROPERTY(scale_factors, SimTK::Vec3,
        "Scale factors in X, Y, Z directions respectively.");
    // Default display properiies e.g. Representation, color, texture, etc.
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "Default appearance for this Geometry");

    enum DisplayPreference {
        Hide = 0,       ///<Hide geometry from display
        DrawPoints = 1, ///< Use a cloud of points.
        DrawWireframe = 2, ///< Use a line drawing.
        DrawSurface = 3, ///< Use a shaded surface.

        DrawDefault = -1  ///< Let someone else decide.
    };
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    /// Default constructor, does nothing
    Geometry()
    {
        setNull();
        constructInfrastructure();

        constructProperty_scale_factors(SimTK::Vec3(1));
        constructProperty_Appearance(Appearance());
    }
    /// Convenience constructor that takes a Frame
    Geometry(const Frame& frame)
    {
        setNull();
        constructInfrastructure();

        constructProperty_scale_factors(SimTK::Vec3(1));
        constructProperty_Appearance(Appearance());

        setFrame(frame);
    }

    /// Default destructor
    virtual ~Geometry() {}
    /** Interface methods to handle the Frame which the Geometry is attached to.
    **/
    /** Set the name of the Frame of attachment **/
    void setFrameName(const std::string& name);
    /** Set the Frame of attachment **/
    void setFrame(const Frame& frame);
    /** Return a reference to the name of the Frame to which
    this Geometry is attached (using a Connector). **/
    const std::string& getFrameName() const;
    /** Return a reference to the actual Frame to which this Geometry
    is attached. */
    const Frame& getFrame() const;
    //==========================================================================
    // METHODS
    //==========================================================================
    /// Compute Transform of this geometry relative to its base frame, utilizing 
    /// passed in state. Both transform and body_id are set in the passed-in 
    /// decorations as a side effect.
    void setDecorativeGeometryTransform(
                        SimTK::Array_<SimTK::DecorativeGeometry>& decorations, 
                        const SimTK::State& state) const;

    /// Manage Appearance (how the Geometry is rendered) by applying Appearance 
    /// from Geometry to DecorativeGeometry.
    void setDecorativeGeometryAppearance(
        SimTK::DecorativeGeometry& decoration) const {
            decoration.setColor(get_Appearance().get_color());
            decoration.setOpacity(get_Appearance().get_opacity());
            decoration.setRepresentation(
                (SimTK::DecorativeGeometry::Representation)
                get_Appearance().get_representation());
    };
    /// Convenient access to set Appearance/Color
    void setColor(const SimTK::Vec3& color) { 
        upd_Appearance().set_color(color); 
    };
    /// Convenient access to get Appearance/Color
    const SimTK::Vec3& getColor() const { 
        return get_Appearance().get_color(); 
    };

    /// Convenient access to set Appearance/Opacity
    void setOpacity(const double opacity) { 
        upd_Appearance().set_opacity(opacity); 
    };
    /// Convenient access to get Appearance/Opacity
    const double getOpacity() { 
        return get_Appearance().get_opacity(); 
    };

    /// Convenient access to set Appearance/representation
    void setRepresentation(const DisplayPreference& rep) { 
        upd_Appearance().set_representation(rep); 
    };
    /// Convenient access to get Appearance/representation
    const DisplayPreference& getRepresentation() { return 
        (const DisplayPreference&)get_Appearance().get_representation(); 
    };

    /// Map this Geometry into a list of primitives aka SimTK::DecorativeGeometry 
    /// and return it in the passed in Array.
    virtual void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>&) const {};
    /// Implement method from Component interface. Subclasses only need to 
    /// implement createDecorativeGeometry to generate an Array of 
    /// SimTK::DecorativeGeometry. From then on, setting Transforms & Appearance 
    /// is handled by the base class Geometry to avoid duplication. 
    void generateDecorations
        (bool                                       fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const override final;

    /// Methods to support frame as a connection, implement Component interface
    void constructConnectors();

private:
    
    void setNull()
    {
        setAuthors("Ayman Habib");
    }    //=====================================================================
};  // END of class Geometry

/**
 * LineGeometry is a utility class used to abstract a line segment.
 * It is used by muscle segments so that it's as small and useful as possible.
 */
class OSIMSIMULATION_API LineGeometry : public Geometry
{   
    OpenSim_DECLARE_CONCRETE_OBJECT(LineGeometry, Geometry);
public:
    // Property start_point
    OpenSim_DECLARE_PROPERTY(start_point, SimTK::Vec3,
        "Line start point.");
    // Property end_point
    OpenSim_DECLARE_PROPERTY(end_point, SimTK::Vec3,
        "Line end point.");
    /// Convenience constructor that takes two end points
    LineGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2):
      Geometry()
    {
        constructProperties();
        setPoints(aPoint1, aPoint2);
        std::string gnd("ground");
        setFrameName(gnd);
    }
    /// default constructor, creates line (0,0,0)-(1,1,1)
    LineGeometry():
      Geometry()
    {
        constructProperties();
    }
    /// destructor
    virtual ~LineGeometry() {}
    /// Get end points as Vec3 in passed in arguments
    void getPoints(SimTK::Vec3& rPoint1, SimTK::Vec3& rPoint2) const 
    {
        rPoint1 = get_start_point();
        rPoint2 = get_end_point();
    }
    /// Set end points from passed in arguments
    void setPoints(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2)
    {
        upd_start_point() = aPoint1;
        upd_end_point() = aPoint2;
    }

    /** Virtual method to map LineGeometry to SimTK::Array of
    SimTK::DecorativeGeometry.  Appearance, Transforms are handled by base
    Geometry class. */
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties(){
        constructProperty_start_point(SimTK::Vec3(0));
        constructProperty_end_point(SimTK::Vec3(1));
    }
};
/**
* Arrow is a Geometry subclass used to represent an arrow. The arrow goes from
* start_point (Property) and has direction (Property) and length (Property)
* 
*/
class OSIMSIMULATION_API Arrow : public Geometry
{   
    OpenSim_DECLARE_CONCRETE_OBJECT(Arrow, Geometry);
public:
    // Property start_point
    OpenSim_DECLARE_PROPERTY(start_point, SimTK::Vec3,
        "Arrow start point.");
    // Property direction 
    OpenSim_DECLARE_PROPERTY(direction, SimTK::Vec3, 
        "direction of Arrow");

    OpenSim_DECLARE_PROPERTY(length, double, "length of Arrow");
    /// constructor that takes startPoint, direction vector and length
    Arrow(SimTK::Vec3& aPoint1, SimTK::UnitVec3& aUnitDir, double aLength) 
    {
        constructProperties();
        updProperty_start_point() = aPoint1;
        updProperty_direction() = aUnitDir;
        updProperty_length() = aLength;
    }
    /// Default constructor that creates Arrow of length 1 starting at origin 
    /// in the direction (1,1,1)
    Arrow() 
    {
        constructProperties();
    }
    /// destructor
    virtual ~Arrow() {}

    /** Virtual method to map Arrow to SimTK::Array of SimTK::DecorativeGeometry.
    Appearance, Transforms are handled by base Geometry class. */
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties(){
        constructProperty_start_point(SimTK::Vec3(0));
        constructProperty_direction(SimTK::Vec3(1));
        constructProperty_length(1.0);
    }
};


/**
 * Utility class used to abstract anayltic geometry. This will need to be 
 * revisited when wrapping is re-done to handle quadrants or analytic shapes
 * that were supported in earlier releases before 4.0. 
 *
 * TODO: using start/end angle may be a better choice.
 */
class OSIMSIMULATION_API AnalyticGeometry : public Geometry
{    
    OpenSim_DECLARE_ABSTRACT_OBJECT(AnalyticGeometry, Geometry);
    // Amended with a quadrants array to support pieces of analytic geometry (for wrapping)
    OpenSim_DECLARE_LIST_PROPERTY(quadrants, std::string,
        "Quadrants to use: combination of +X -X +Y -Y +Z -Z space separated."); 
private:
    bool                    _bounds[6];     //-X, +X, -Y, +Y, -Z, +Z
    bool                    _piece;
public:
    AnalyticGeometry():
        _piece(false)
    {
        constructProperties();
        for(int i=0; i<6; i++) _bounds[i]=true;
    }
    virtual ~AnalyticGeometry() {}
    void setQuadrants(const bool quadrants[6])
    {
        _piece=false;
        for(int i=0; i<6; i++){
            _bounds[i]=quadrants[i];
            _piece = _piece || (!quadrants[i]);
        }
    }
    void getQuadrants(bool quadrants[6])
    {
        for(int i=0; i<6; i++){
            quadrants[i]=_bounds[i];
        }
    }
    const bool isPiece() const
    {
        return _piece;
    }
private:
    void constructProperties() {
        constructProperty_quadrants();
    }
};
/**
 * A class to represent Sphere geometry. 
 */
class OSIMSIMULATION_API Sphere : public AnalyticGeometry
{
OpenSim_DECLARE_CONCRETE_OBJECT(Sphere, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(radius, double,
        "Radius of sphere, defaults to 1.0"); 
    /// Default constructor, creates a sphere of radius 1.0
    Sphere() :
    AnalyticGeometry()
    {
        constructProperties();
    }
    /// Another constructor that takes in a specified radius
    Sphere(double radius):
        AnalyticGeometry()
        {
        constructProperties();
        upd_radius() = radius;
        }
    /// destructor
    ~Sphere() {}
    /// Convenience method to set radius
    void setSphereRadius(double radius)
    {
        upd_radius() = radius;
    }
    /// Virtual method to map Sphere to Array of SimTK::DecorativeGeometry.
    /// Appearance, Transforms are handled by base Geometry class
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties(){
        constructProperty_radius(1.0);
    }
};  // Sphere

/**
* A class to represent an Ellipsoid geometry.
*/
class OSIMSIMULATION_API Ellipsoid : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Ellipsoid, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(radii, SimTK::Vec3, "Radii of Ellipsoid."); 
    /// Default constructor, creates an Ellipsoid of radii 0.5, 1., 2.
    Ellipsoid() :
    AnalyticGeometry()
    {
        constructProperties();
    }
    /// Constructor that takes in three radii
    Ellipsoid(double radius1, double radius2, double radius3):
        AnalyticGeometry(){
        constructProperties();
        setEllipsoidParams(radius1, radius2, radius3);
    }
    /// destructor
    ~Ellipsoid() {}
    /// Convenience interface to set radii
    void setEllipsoidParams(double radius1, double radius2, double radius3)
    {
        upd_radii()[0] = radius1;
        upd_radii()[1] = radius2;
        upd_radii()[2] = radius3;
    } 
    /** Virtual method to map Ellipsoid to SimTK::Array_ of
    SimTK::DecorativeGeometry.  Appearance, Transforms are handled by base
    Geometry class. */
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties() {
        constructProperty_radii(SimTK::Vec3(0.5, 1., 2.));
    }
};


/**
* A class to represent a Cylinder geometry.
*/

class OSIMSIMULATION_API Cylinder : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Cylinder, AnalyticGeometry);
public:
    // Property radius
    OpenSim_DECLARE_PROPERTY(radius, double, "Radius of cylinder.");
    // Property half_height
    OpenSim_DECLARE_PROPERTY(half_height, double, "Half-Height of cylinder.");
    /// Default constructor
    Cylinder() :
    AnalyticGeometry()
    {
        constructProperties();
    }
    /// Convenience constructor that takes radius and half-height
    Cylinder(const double radius, const double hheight):
        AnalyticGeometry()
        {
        constructProperties();
        upd_radius() = radius;
        upd_half_height() = hheight;
        }
    /// destructor
    ~Cylinder() {}
    /// Convenient way to get the two parameters that define the cylinder
    void getCylinderParams(SimTK::Vec2& params) const
    {
        params[0] = get_radius();
        params[1] = get_half_height();
    }
    /** Virtual method to map Cylinder to SimTK::Array_ of SimTK::DecorativeGeometry.
    Appearance, Transforms are handled by base Geometry class. */
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties() {
        constructProperty_radius(0.5);
        constructProperty_half_height(0.5);
    }
};



/**
* A class to represent a Cone geometry.
*/

class OSIMSIMULATION_API Cone : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Cone, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(origin, SimTK::Vec3, "origin of cone base");
    OpenSim_DECLARE_PROPERTY(direction, SimTK::Vec3, "direction of cone axis.");
    OpenSim_DECLARE_PROPERTY(base_radius, double, "Base radius of cone.");
    OpenSim_DECLARE_PROPERTY(height, double, "Height of cone.");
    /// Default constructor
    Cone() :
        AnalyticGeometry()
    {
        constructProperties();
    }
    /// Convenience constructor that takes radius and half-height
    Cone(const SimTK::Vec3& o, const SimTK::UnitVec3& dir,
        double height, double base) :
        AnalyticGeometry()
    {
        constructProperties();
        upd_origin() = o;
        upd_direction() = SimTK::Vec3(dir);
        upd_base_radius() = base;
        upd_height() = height;
    }
    /// destructor
    ~Cone() {}
    /** Method to map Cone to SimTK::Array of SimTK::DecorativeGeometry.
    Appearance, Transforms are handled by base Geometry class. */
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties() {
        constructProperty_origin(SimTK::Vec3(0));
        constructProperty_direction(SimTK::UnitVec3(1));
        constructProperty_base_radius(0.5);
        constructProperty_height(1.0);
    }
};

/**
* A class to represent Torus geometry. The torus is centered at the
origin with the axial direction aligned to the z-axis. It is defined by
a ring_radius (radius of the circular centerline of the torus, measured
from the origin), and a cross_section (radius of the torus cross-section:
perpendicular distance from the circular centerline to the surface).
*/
class OSIMSIMULATION_API Torus : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Torus, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(cross_section, double, "Cross section radius");
    OpenSim_DECLARE_PROPERTY(ring_radius, double, "Radius of the torus ring.");
public:
    /// Default constructor
    Torus():
    AnalyticGeometry()
    {}
    /// Constructor that takes in two radii
    Torus(const double ringRadius, const double crossSectionRadius):
    AnalyticGeometry()
    {
        upd_ring_radius()=ringRadius;
        upd_cross_section()=crossSectionRadius;
    }
    virtual ~Torus() {}

    /** Method to map Cone to Array of SimTK::DecorativeGeometry.
    Appearance, Transforms are handled by base Geometry class. */
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override {};
};

/**
* A class to represent Brick geometry. Brick is specified by three half_lengths
*/
class OSIMSIMULATION_API Brick : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Brick, Geometry);
public:
    OpenSim_DECLARE_PROPERTY(half_lengths, SimTK::Vec3, 
        "Half lengths in X, Y, Z respectively.");

public:
    /// Default constructor, makes a Brick with half-length 0.1,0.2,0.3
    Brick() :
    Geometry()
    {
        constructProperty_half_lengths(SimTK::Vec3(0.1, 0.2, 0.3));
    }
    /// Convenience constructor with specified half-lengths
    Brick(const SimTK::Vec3& halfLengths) :
        Geometry()
    {
        constructProperty_half_lengths(SimTK::Vec3(0.1, 0.2, 0.3));
        upd_half_lengths() = halfLengths;
    }
    /// Destructor
    ~Brick() {}
    /// Method to map Brick to Array of SimTK::DecorativeGeometry.
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;

};
/**
* A class to represent Mesh geometry that comes from a file.
* Supported file formats .vtp, .stl, .obj but will grow over time
*/
class OSIMSIMULATION_API Mesh : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Mesh, Geometry);
public:
    OpenSim_DECLARE_PROPERTY(mesh_file, std::string,
        "Name of geometry file.");

public:
    /// Default constructor
    Mesh() :
        Geometry()
    {
        constructProperty_mesh_file("");
    }
    /// Constructor that takes a mesh file name
    Mesh(const std::string& geomFile) :
        Geometry()
    {
        constructProperty_mesh_file("");
        upd_mesh_file() = geomFile;
    }
    /// destructor
    virtual ~Mesh() {};
    /// Retrieve file name
    const std::string&  getGeometryFilename() const
    {
        return get_mesh_file();
    };
    /// Method to map Mesh to Array of SimTK::DecorativeGeometry.
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;

};

/**
* A class to represent Frame geometry. Knobs that can be changed
* are in Appearance::Representation, size, thickness.
*/
class OSIMSIMULATION_API FrameGeometry : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(FrameGeometry, Geometry);
public:
    OpenSim_DECLARE_PROPERTY(display_radius, double,
        "The radius of the arrow-shaft used to display the frame.");
    /// Default constructor
    FrameGeometry(double scale=1.0) :
        Geometry()
    {
       constructInfrastructure();
       set_scale_factors(SimTK::Vec3(scale));
    }
    /// destructor
    virtual ~FrameGeometry() {};
    /// Method to map FrameGeometry to Array of SimTK::DecorativeGeometry.
    void createDecorativeGeometry(
        SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructInfrastructure() {
        constructProperty_display_radius(.005);
    }
};
}; //namespace
//=============================================================================
//=============================================================================

#endif //__OPENSIM_GEOMETRY_H__
