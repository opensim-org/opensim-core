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
protected:
	SimTK::Vec3 _point1;
	SimTK::Vec3 _point2;
public:
	LineGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2):
	  Geometry()
	{
		_point1 = aPoint1;
		_point2 = aPoint2;
	}
	LineGeometry():
	  Geometry(),
	  _point1(0.0),
	  _point2(0.0)
	{
	}
	virtual ~LineGeometry() {}
	// Get & Set end points
	void getPoints(SimTK::Vec3& rPoint1, SimTK::Vec3& rPoint2) const 
	{
		rPoint1 = _point1;
		rPoint2 = _point2;
	}
	void getPoints(double rPoint1[], double rPoint2[]) const // A variant that uses basic types for use by GUI
	{
		getPoints(SimTK::Vec3::updAs(rPoint1), SimTK::Vec3::updAs(rPoint2));
	}
	void setPoints(SimTK::Vec3& aPoint1, SimTK::Vec3& aPoint2)
	{
		_point1 = aPoint1;
		_point2 = aPoint2;
	}
	void setPoints(double aPoint1[], double aPoint2[])	// A variant that uses basic types for use by GUI
	{
		setPoints(SimTK::Vec3::updAs(aPoint1), SimTK::Vec3::updAs(aPoint2));
	}
};

class OSIMSIMULATION_API ArrowGeometry : public LineGeometry
{	
    OpenSim_DECLARE_CONCRETE_OBJECT(ArrowGeometry, Geometry);
public:
	ArrowGeometry(SimTK::Vec3& aPoint1, SimTK::Vec3& aUnitDirTo, double aLength):
	  LineGeometry(aPoint1, /* aPoint1+aLength* */aUnitDirTo)
	{
	}
	virtual ~ArrowGeometry() {}
};


/**
 * Utility class used to abstract anayltic geometry. 
 * If the differentiation between types becomes an issue, or more info need
 * to be stuffed in this class, it may be worth it to separate it into a bunch,
 * one per shape.
 */
class OSIMSIMULATION_API AnalyticGeometry : public Geometry
{	 
    OpenSim_DECLARE_ABSTRACT_OBJECT(AnalyticGeometry, Geometry);
	// Common array of attributes for analytic geometry we can represent
	// Meaning depends on _analyticType as follows.
	// Amended with a quadrants array to support pieces of analytic geometry (for wrapping)
	// Sphere	[Radius, unused, unused, unused, unused, unused]
	// Cylinder  [Radius, Height, unused, ...]
	// Cone		 [BaseRadius, TopRadius, Height, unused, ..]
	// Ellipsoid [AxisLength1, AxisLength2, AxisLength3, unused, ..]
private:
	bool					_bounds[6];		//-X, +X, -Y, +Y, -Z, +Z
	bool					_piece;
public:
	AnalyticGeometry():
		_piece(false)
	{
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
};

class OSIMSIMULATION_API AnalyticSphere : public AnalyticGeometry
{
OpenSim_DECLARE_CONCRETE_OBJECT(AnalyticSphere, AnalyticGeometry);
protected:
	double				_attributes[1];
public:
	AnalyticSphere():
		AnalyticGeometry()
		{}
	AnalyticSphere(double radius):
		AnalyticGeometry()
		{
			_attributes[0] = radius;
		}
	virtual ~AnalyticSphere() {}
	const double& getRadius() const
	{
		//assert(_analyticType==Sphere);
		return _attributes[0];
	}
	void setSphereRadius(double radius)
	{
		//assert(_analyticType==Sphere);
			_attributes[0] = radius;		
	}
	static AnalyticGeometry* createSphere(double radius)
	{
		AnalyticSphere* sphere = new AnalyticSphere();
		sphere->setSphereRadius(radius);
		return sphere;
	}
};	// AnalyticSphere

class OSIMSIMULATION_API AnalyticEllipsoid : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(AnalyticEllipsoid, AnalyticGeometry);
protected:
	double				_attributes[3];
public:
	AnalyticEllipsoid():
		AnalyticGeometry()
		{}
	AnalyticEllipsoid(double radius1, double radius2, double radius3):
		AnalyticGeometry(){
			_attributes[0] = radius1;
			_attributes[1] = radius2;
			_attributes[2] = radius3;
	}
	virtual ~AnalyticEllipsoid() {}
	void setEllipsoidParams(double radius1, double radius2, double radius3)
	{
		//assert(_analyticType==Sphere);
		_attributes[0] = radius1;
		_attributes[1] = radius2;
		_attributes[2] = radius3;
	}
	void getEllipsoidParams(double params[])
	{
		//assert(_analyticType==Ellipsoid);
		for(int i=0;i<3; i++)
			params[i] = _attributes[i];
	}
};

class OSIMSIMULATION_API AnalyticCylinder : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(AnalyticCylinder, AnalyticGeometry);
protected:
	double				_attributes[2];
public:
	AnalyticCylinder():
		AnalyticGeometry()
		{}
	AnalyticCylinder(const double radius, const double height):
		AnalyticGeometry()
		{
			_attributes[0]=radius;
			_attributes[1]=height;
		}
	virtual ~AnalyticCylinder() {}
	void getCylinderParams(double params[]) const
	{
		//assert(_analyticType==Cylinder);
		params[0] = _attributes[0];
		params[1] = _attributes[1];
	}
};

class OSIMSIMULATION_API AnalyticTorus : public AnalyticGeometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(AnalyticTorus, AnalyticGeometry);
protected:
	double				_attributes[2];
public:
	AnalyticTorus():
		AnalyticGeometry()
		{}
	AnalyticTorus(const double ringRadius, const double crossSectionRadius):
		AnalyticGeometry()
		{
			_attributes[0]=ringRadius;
			_attributes[1]=crossSectionRadius;
		}
	virtual ~AnalyticTorus() {}
	void getTorusParams(double params[]) const
	{
		//assert(_analyticType==Torus);
		params[0] = _attributes[0];
		params[1] = _attributes[1];
	}
};

class OSIMSIMULATION_API MeshGeometry : public Geometry
{
    OpenSim_DECLARE_CONCRETE_OBJECT(MeshGeometry, Geometry);
public:
	std::string _geometryFile;
    OpenSim_DECLARE_PROPERTY(mesh_file, std::string,
        "Name of geometry file.");

public:
	MeshGeometry(const std::string& geomFile):
	Geometry()
	{
    constructProperty_mesh_file("");
    upd_mesh_file() = geomFile;
	}
	const std::string&  getGeometryFilename() const
	{
		return get_mesh_file();
	};

};

typedef OpenSim::Set<Geometry> GeometrySet;
}; //namespace
//=============================================================================
//=============================================================================

#endif //__OPENSIM_GEOMETRY_H__
