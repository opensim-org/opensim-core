#ifndef _Geometry_h_
#define _Geometry_h_
// Geometry.h
// Authors: Ayman Habib
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/*  
 * Author:  Ayman Habib
 */

namespace OpenSim { 

// CONSTANTS

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
class RDTOOLS_API Geometry
{
public:
	// Basically subtypes so that we can do dynamic casting safely on GUI side, based on type
	enum GeometryType{
		None, Sphere,  Cylinder, Cone, Ellipsoid, Torus, Line, Arrow	
	};
private:
	bool			_fixed; /** to indicate if the geometry is fixed vs deformable */
protected:
	GeometryType	_analyticType;
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Geometry():
	_fixed(true),
	_analyticType(None)
	{}
	virtual ~Geometry() {}
	//=============================================================================
	// METHODS
	//=============================================================================
	// Retrieve analytic type
	const GeometryType getShape() const
	{
		return _analyticType;
	}
	// Check if the geometry corresponds to an analytic geometry
	virtual bool isAnalytic() const
	{
		return _analyticType!=None;
	}
	// Mark geometry as Fixed (so that vtk resources are allocated once,
	// potentially use the same vtkGeometry more than once.)
	virtual void setFixed(bool aFixed)
	{
		_fixed = aFixed;
	}
	virtual bool getFixed() const
	{
		return _fixed;
	}

//=============================================================================
};	// END of class Geometry

/***
 * LineGeometry is a utility class used to abstract a line segment.
 * It is used by muscle segments so that it's as small and useful as possiblethe 
 * For muscle segments, GUI is free to display it as a line, cylinder or ellipsoid
 */
class RDTOOLS_API LineGeometry : public Geometry
{	
protected:
	double _point1[3];
	double _point2[3];
public:
	LineGeometry(double aPoint1[3], double aPoint2[3]):
	  Geometry()
	{
		
		_analyticType=Line;
		for(int i=0; i<3; i++){
			_point1[i] = aPoint1[i];
			_point2[i] = aPoint2[i];
		}
	}
	LineGeometry():
	  Geometry()
	{
		_analyticType=Line;
		for(int i=0; i<3; i++){
			_point1[i] = _point2[i] =0.0;
		}
	}
	virtual ~LineGeometry() {}
	// Get & Set end points
	void getPoints(double rPoint1[3], double rPoint2[3]) const
	{
		for(int i=0; i<3; i++){
			rPoint1[i] = _point1[i];
			rPoint2[i] = _point2[i];
		}
	}
	void setPoints(double aPoint1[3], double aPoint2[3])
	{
		for(int i=0; i<3; i++){
			_point1[i] = aPoint1[i];
			_point2[i] = aPoint2[i];
		}
	}
};

class RDTOOLS_API ArrowGeometry : public LineGeometry
{	
public:
	ArrowGeometry(double aPoint1[3], double aUnitDirTo[3], double aLength):
	  LineGeometry(aPoint1, /* aPoint1+aLength* */aUnitDirTo)
	{
		_analyticType = Arrow;
	}
	virtual ~ArrowGeometry() {}
};


/**
 * Utility class used to abstract anayltic geometry. 
 * If the differentiation between types becomes an issue, or more info need
 * to be stuffed in this class, it may be worth it to separate it into a bunch,
 * one per shape.
 */
class RDTOOLS_API AnalyticGeometry : public Geometry
{	 

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
	AnalyticGeometry(GeometryType aGeometricType):
		_piece(false)
	{
		_analyticType=aGeometricType;
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

class RDTOOLS_API AnalyticSphere : public AnalyticGeometry
{
protected:
	double				_attributes[1];
public:
	AnalyticSphere():
		AnalyticGeometry(Sphere)
		{}
	AnalyticSphere(double radius):
		AnalyticGeometry(Sphere)
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

class RDTOOLS_API AnalyticEllipsoid : public AnalyticGeometry
{
protected:
	double				_attributes[3];
public:
	AnalyticEllipsoid():
		AnalyticGeometry(Ellipsoid)
		{}
	AnalyticEllipsoid(double radius1, double radius2, double radius3):
		AnalyticGeometry(Ellipsoid){
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

class RDTOOLS_API AnalyticCylinder : public AnalyticGeometry
{
protected:
	double				_attributes[2];
public:
	AnalyticCylinder():
		AnalyticGeometry(Cylinder)
		{}
	AnalyticCylinder(const double radius, const double height):
		AnalyticGeometry(Cylinder)
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

class RDTOOLS_API AnalyticTorus : public AnalyticGeometry
{
protected:
	double				_attributes[2];
public:
	AnalyticTorus():
		AnalyticGeometry(Torus)
		{}
	AnalyticTorus(const double ringRadius, const double crossSectionRadius):
		AnalyticGeometry(Torus)
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

class RDTOOLS_API PolyhedralGeometry : public Geometry
{
protected:
	std::string _geometryFile;
public:
	PolyhedralGeometry(const std::string& geomFile):
		Geometry()
		{
			_geometryFile = geomFile;
		}
	const std::string&  getGeometryFilename() const
	{
		return _geometryFile;
	};

};

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Geometry_h__
