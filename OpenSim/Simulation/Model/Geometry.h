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

// Geometry.h
// Authors: Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*  
 * Author:  Ayman Habib
 */

namespace OpenSim { 

class RigidFrame;
class Model;
class Frame;
//=============================================================================
//=============================================================================
/**
 * Class Geometry is intended to be used as the base class for all
 * geometry that needs to be represented in the system, both as vtk files, or analytic
 *
 * @version 1.0
 * @author Ayman Habib
 */

// Base Geometry
class OSIMSIMULATION_API Geometry : public Component {
    OpenSim_DECLARE_ABSTRACT_OBJECT(Geometry, Component);
public:
    // Scale factors
    OpenSim_DECLARE_PROPERTY(scale_factors, SimTK::Vec3,
        "Scale factors in X, Y, Z directotions respectively.");
    OpenSim_DECLARE_PROPERTY(frame_name, std::string,
        "Name of the Frame that this Geometry is attached to.");
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Geometry()
	{
        constructProperty_scale_factors(SimTK::Vec3(1));
        constructProperty_frame_name("ground");
    }
	virtual ~Geometry() {}

	//=============================================================================
	// METHODS
	//=============================================================================
    SimTK::Transform getTransform(const SimTK::State& state, const OpenSim::RigidFrame& frame) const;

    virtual void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& ) const {};
    //=============================================================================
};	// END of class Geometry

/***
 * LineGeometry is a utility class used to abstract a line segment.
 * It is used by muscle segments so that it's as small and useful as possiblethe 
 * For muscle segments, GUI is free to display it as a line, cylinder or ellipsoid
 */
class OSIMSIMULATION_API LineGeometry : public Geometry
{	
    OpenSim_DECLARE_CONCRETE_OBJECT(LineGeometry, Geometry);
    OpenSim_DECLARE_PROPERTY(start_point, SimTK::Vec3,
        "Line start point.");
    OpenSim_DECLARE_PROPERTY(end_point, SimTK::Vec3,
        "Line end point.");
public:
	LineGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2):
	  Geometry()
	{
        constructProperties();
        setPoints(aPoint1, aPoint2);
	}
	LineGeometry():
	  Geometry()
	{
        constructProperties();
	}
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

// unimplemented yet, make Properties
class OSIMSIMULATION_API ArrowGeometry : public LineGeometry
{	
    OpenSim_DECLARE_CONCRETE_OBJECT(ArrowGeometry, Geometry);
public:
	ArrowGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aUnitDirTo, double aLength):
	  LineGeometry(aPoint1, /* aPoint1+aLength* */aUnitDirTo)
	{
	}
	virtual ~ArrowGeometry() {}

    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override {};
};


/**
 * Utility class used to abstract anayltic geometry. 
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

class OSIMSIMULATION_API Sphere : public AnalyticGeometry
{
OpenSim_DECLARE_CONCRETE_OBJECT(Sphere, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(radius, double,
        "Radius of sphere."); 
    Sphere() :
	AnalyticGeometry()
	{
        constructProperties();
    }
	Sphere(double radius):
		AnalyticGeometry()
		{
        constructProperties();
        upd_radius() = radius;
		}
	virtual ~Sphere() {}
	void setSphereRadius(double radius)
	{
		//assert(_analyticType==Sphere);
        upd_radius() = radius;
	}
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties(){
        constructProperty_radius(1.0);
    }
};	// Sphere

class OSIMSIMULATION_API Ellipsoid : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Ellipsoid, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(radii, SimTK::Vec3, "Radii of Ellipsoid."); 
    Ellipsoid() :
	AnalyticGeometry()
	{
        constructProperties();
    }
	Ellipsoid(double radius1, double radius2, double radius3):
		AnalyticGeometry(){
        constructProperties();
        setEllipsoidParams(radius1, radius2, radius3);
	}
	virtual ~Ellipsoid() {}
	void setEllipsoidParams(double radius1, double radius2, double radius3)
	{
		//assert(_analyticType==Sphere);
        upd_radii()[0] = radius1;
        upd_radii()[1] = radius2;
        upd_radii()[2] = radius3;
    }
	void getEllipsoidParams(SimTK::Vec3& params)
	{
        params = get_radii();
	} 
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties() {
        constructProperty_radii(SimTK::Vec3(0.5, 1., 2.));
    }
};

class OSIMSIMULATION_API Cylinder : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Cylinder, AnalyticGeometry);
public:
    OpenSim_DECLARE_PROPERTY(radius, double, "Radius of cylinder.");
    OpenSim_DECLARE_PROPERTY(height, double, "Height of cylinder.");
    Cylinder() :
	AnalyticGeometry()
	{
        constructProperties();
    }
	Cylinder(const double radius, const double height):
		AnalyticGeometry()
		{
        constructProperties();
        upd_radius() = radius;
        upd_height() = height;
		}
	virtual ~Cylinder() {}
	void getCylinderParams(SimTK::Vec2& params) const
	{
		//assert(_analyticType==Cylinder);
        params[0] = get_radius();
        params[1] = get_height();
	}
    void createDecorativeGeometry(SimTK::Array_<SimTK::DecorativeGeometry>& decoGeoms) const override;
private:
    void constructProperties() {
        constructProperty_radius(1.0);
        constructProperty_height(0.5);
    }
};
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

class OSIMSIMULATION_API Mesh : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(Mesh, Geometry);
public:
    OpenSim_DECLARE_PROPERTY(mesh_file, std::string,
        "Name of geometry file.");

public:
    Mesh(const std::string& geomFile) :
	Geometry()
	{
    constructProperty_mesh_file("");
    upd_mesh_file() = geomFile;
	}
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
