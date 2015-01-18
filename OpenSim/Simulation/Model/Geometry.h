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
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

// Geometry.h
// Authors: Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* -------------------------------------------------------------------------- *
*                              OpenSim:  Geometry.h                          *
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


/*  
 * Author:  Ayman Habib
 */

namespace OpenSim { 

class RigidFrame;
class Model;
class Frame;
class ModelComponent;
//=============================================================================
//=============================================================================
/**
 * Class Geometry is intended to be used as the base class for all
 * geometry that needs to be represented in the system, both as mesh files, 
 * or built in analytic shapes. Any ModelComponent can specify a list of
 * Geometry items to represent itself in graphics window. The association between
 * a ModelComponent and specific Geometry is made using the subcomponent paradigm.
 *
 * @author Ayman Habib
 */

// Base Geometry
class OSIMSIMULATION_API Geometry : public Component {
    OpenSim_DECLARE_ABSTRACT_OBJECT(Geometry, Component);
public:
    // Properties common to all Geometry types are included as Properties
    // Scale factors
    OpenSim_DECLARE_PROPERTY(scale_factors, SimTK::Vec3,
        "Scale factors in X, Y, Z directions respectively.");
    // Optional frame_name (optional since some objects already have default
    // frames, e.g. Body, Frame
    OpenSim_DECLARE_OPTIONAL_PROPERTY(frame_name, std::string,
        "Name of the Frame that this Geometry is attached to.");
    // Default display properiies e.g. Representation, color, texture, etc.
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance,
        "Default appearance for this Geometry");

    enum Representation {
        Hide = 0,
        DrawPoints = 1, ///< Use a cloud of points.
        DrawWireframe = 2, ///< Use a line drawing.
        DrawSurface = 3, ///< Use a shaded surface.

        DrawDefault = -1  ///< Let someone else decide.
    };
    //--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
    // Default constructor, does nothing
	Geometry()
	{
        constructProperty_scale_factors(SimTK::Vec3(1));
        constructProperty_frame_name();
        constructProperty_Appearance(Appearance());
    }
    // Default destructor
	virtual ~Geometry() {}

	//=============================================================================
	// METHODS
	//=============================================================================
    // Get Transform of this geometry relative to passed in frame, utilizing passed 
    // in state.
    SimTK::Transform getTransform(const SimTK::State& state, 
        const OpenSim::RigidFrame& frame) const;
    // Manage Appearance or how the Geometry is rendered by moving Appearance down
    // from Geometry to DecorativeGeometry.
    void setDecorativeGeometryAppearance(SimTK::DecorativeGeometry& decoration) const {
        decoration.setColor(get_Appearance().get_color());
        decoration.setOpacity(get_Appearance().get_opacity());
        decoration.setRepresentation((SimTK::DecorativeGeometry::Representation)get_Appearance().get_representation());
    };
    // Convenient access to Appearance constituents: Color
    void setColor(const SimTK::Vec3& color) { upd_Appearance().set_color(color); };
    const SimTK::Vec3& getColor() const { return get_Appearance().get_color(); };

    // Convenient access to Appearance constituents: Opacity
    void setOpacity(const double opacity) { upd_Appearance().set_opacity(opacity); };
    const double getOpacity() { return get_Appearance().get_opacity(); };

    // Convenient access to Appearance constituents: /representation
    void setRepresentation(const Representation& rep) { upd_Appearance().set_representation(rep); };
    const Representation& getRepresentation() { return (const Representation&)get_Appearance().get_representation(); };

    // set/get ModelComponent that owns this Geometry.
    void setOwnerModelComponent(const OpenSim::ModelComponent& mc) {
        _owner = mc;
    }
    const ModelComponent& getOwnerModelComponent() const {
        return _owner.getRef();
    }
    // Get name of Frame that this Geometry is attached to. This could 
    // either be a property, or owner ModelComponent if it's a type of frame
    std::string getFrameName() const;

    // Map this Geometry into a list of primitives aka SimTK::DecorativeGeometry 
    // and return it in the passed in Array.
    virtual void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>&) const {};

private:
    SimTK::ReferencePtr<const OpenSim::ModelComponent> _owner;
    //=========================================================================
};	// END of class Geometry

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
    // Convenience constructor that takes two end points
	LineGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2):
	  Geometry()
	{
        constructProperties();
        setPoints(aPoint1, aPoint2);
	}
    // default constructor, creates line (0,0,0)-(1,1,1)
	LineGeometry():
	  Geometry()
	{
        constructProperties();
	}
    // destructor
	virtual ~LineGeometry() {}
	// Get & Set end points
	void getPoints(SimTK::Vec3& rPoint1, SimTK::Vec3& rPoint2) const 
	{
		rPoint1 = get_start_point();
		rPoint2 = get_end_point();
	}
	void getPoints(double rPoint1[], double rPoint2[]) const // A variant that uses basic types for use by GUI
	{
		getPoints(SimTK::Vec3::updAs(rPoint1), SimTK::Vec3::updAs(rPoint2));
	}
    // Scripting friendly setter methods
	void setPoints(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2)
	{
        upd_start_point() = aPoint1;
        upd_end_point() = aPoint2;
    }
	void setPoints(double aPoint1[], double aPoint2[])	// A variant that uses basic types for use by GUI
	{
		setPoints(SimTK::Vec3::updAs(aPoint1), SimTK::Vec3::updAs(aPoint2));
	}

    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties(){
        constructProperty_start_point(SimTK::Vec3(0));
        constructProperty_end_point(SimTK::Vec3(1));
    }
};
/**
* ArrowGeometry is a class used to abstract an arrow.
*/

// untested yet
class OSIMSIMULATION_API ArrowGeometry : public LineGeometry
{	
    OpenSim_DECLARE_CONCRETE_OBJECT(ArrowGeometry, LineGeometry);
public:
    // constructor that takes startPoint, direction vector and length
	ArrowGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aUnitDirTo, double aLength):
	  LineGeometry()
	{
        SimTK::Vec3 point2 = aPoint1 + aLength* aUnitDirTo;
        setPoints(aPoint1, point2);
	}
    // destructor
	virtual ~ArrowGeometry() {}

    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override {};
};


/**
 * Utility class used to abstract anayltic geometry. This will need to be 
 * revisited when wrapping is re-done.
 */
class OSIMSIMULATION_API AnalyticGeometry : public Geometry
{	 
    OpenSim_DECLARE_ABSTRACT_OBJECT(AnalyticGeometry, Geometry);
	// Amended with a quadrants array to support pieces of analytic geometry (for wrapping)
    OpenSim_DECLARE_LIST_PROPERTY(quadrants, std::string,
        "Quadrants to use: combination of +X -X +Y -Y +Z -Z space separated."); 
private:
	bool					_bounds[6];		//-X, +X, -Y, +Y, -Z, +Z
	bool					_piece;
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
    // Default constructor, creates a sphere of radius 1.0
    Sphere() :
	AnalyticGeometry()
	{
        constructProperties();
    }
    // Another constructor that takes in a specified radius
	Sphere(double radius):
		AnalyticGeometry()
		{
        constructProperties();
        upd_radius() = radius;
		}
    // destructor
	~Sphere() {}
    // Scripting friendly method to set radius
	void setSphereRadius(double radius)
	{
        upd_radius() = radius;
	}
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties(){
        constructProperty_radius(1.0);
    }
};	// Sphere

/**
* A class to represent an Ellipsoid geometry.
*/
class OSIMSIMULATION_API Ellipsoid : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Ellipsoid, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(radii, SimTK::Vec3, "Radii of Ellipsoid."); 
    // Default constructor, creates an Ellipsoid of radii 0.5, 1., 2.
    Ellipsoid() :
	AnalyticGeometry()
	{
        constructProperties();
    }
    // Constructor that takes in three radii
	Ellipsoid(double radius1, double radius2, double radius3):
		AnalyticGeometry(){
        constructProperties();
        setEllipsoidParams(radius1, radius2, radius3);
	}
    // destructor
	~Ellipsoid() {}
    // Scripting friendly interface to set radii
	void setEllipsoidParams(double radius1, double radius2, double radius3)
	{
		//assert(_analyticType==Sphere);
        upd_radii()[0] = radius1;
        upd_radii()[1] = radius2;
        upd_radii()[2] = radius3;
    } 
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
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
    OpenSim_DECLARE_PROPERTY(radius, double, "Radius of cylinder.");
    OpenSim_DECLARE_PROPERTY(half_height, double, "Half-Height of cylinder.");
    // Default constructor
    Cylinder() :
	AnalyticGeometry()
	{
        constructProperties();
    }
    // Convenience constructor that takes radius and half-height
	Cylinder(const double radius, const double hheight):
		AnalyticGeometry()
		{
        constructProperties();
        upd_radius() = radius;
        upd_half_height() = hheight;
		}
    // destructor
	~Cylinder() {}
    // Convenient way to get the two parameters that define the cylinder
	void getCylinderParams(SimTK::Vec2& params) const
	{
        params[0] = get_radius();
        params[1] = get_half_height();
	}
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties() {
        constructProperty_radius(0.5);
        constructProperty_half_height(0.5);
    }
};
/**
* A class to represent Torus geometry.
*/
// unimplemented need Properties too
class OSIMSIMULATION_API Torus : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Torus, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(cross_section, double, "Cross section radius");
    OpenSim_DECLARE_PROPERTY(ring_radius, double, "Radius of the torus ring.");
public:
	Torus():
		AnalyticGeometry()
		{}
	Torus(const double ringRadius, const double crossSectionRadius):
		AnalyticGeometry()
		{
			upd_ring_radius()=ringRadius;
			upd_cross_section()=crossSectionRadius;
		}
	virtual ~Torus() {}
	void getTorusParams(double params[]) const
	{
		//assert(_analyticType==Torus);
        params[0] = get_ring_radius();
		params[1] = get_cross_section();
	}
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override {};
};
/**
* A class to represent Brick geometry.
*/
class OSIMSIMULATION_API Brick : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Brick, Geometry);
public:
    // Half lengths in X,Y,Z directions respectively
    OpenSim_DECLARE_PROPERTY(half_lengths, SimTK::Vec3, "Half lengths in X, Y, Z respectively.");

public:
    // Default constructor, makes a Brick with half-length 0.1,0.2,0.3
    Brick() :
    Geometry()
    {
        constructProperty_half_lengths(SimTK::Vec3(0.1, 0.2, 0.3));
    }
    // Convenience constructor with specified half-lengths
    Brick(const SimTK::Vec3& halfLengths) :
        Geometry()
    {
        constructProperty_half_lengths(SimTK::Vec3(0.1, 0.2, 0.3));
        upd_half_lengths() = halfLengths;
    }
    // Destructor
    ~Brick() {}
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;

};
/**
* A class to represent Mesh geometry that comes from a file.
* Supported file formats .vtp, .stl, .obj but will change over time
*/
class OSIMSIMULATION_API Mesh : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Mesh, Geometry);
public:
    OpenSim_DECLARE_PROPERTY(mesh_file, std::string,
        "Name of geometry file.");

public:
    // Default constructor
    Mesh() :
        Geometry()
    {
        constructProperty_mesh_file("");
    }
    // Constructor that takes a mesh file name
    Mesh(const std::string& geomFile) :
        Geometry()
    {
        constructProperty_mesh_file("");
        upd_mesh_file() = geomFile;
    }
    // destructor
    virtual ~Mesh() {};
    const std::string&  getGeometryFilename() const
	{
		return get_mesh_file();
	};
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;

};
}; //namespace
//=============================================================================
//=============================================================================

#endif //__OPENSIM_GEOMETRY_H__
