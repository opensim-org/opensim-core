#ifndef _Transform_h_
#define _Transform_h_

// Transform.h
// Authors: Ayman Habib, Peter Loan
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

#include "osimCommonDLL.h"
#include "Object.h"
#include "SimTKcommon.h"

namespace OpenSim { 

class Mtx;
// CONSTANTS

//=============================================================================
//=============================================================================
/**
 * Class Transform is intended to represent transformations for rd platform
 *
 * @authors Ayman Habib, Peter Loan
 * @version 1.0
 *
 * Removed inheritance from Object since none of this is used and transforms are 
 * created a ton of times during display and simulation
 */
class OSIMCOMMON_API Transform /* : public Object*/
{
public:
	enum AxisName {
		X=0, Y=1, Z=2, W=3, NoAxis=-1
	};
	// Translation is not accounted for here in RotationOrder
	// In all cases I've seen so far it's done either first or last
	// the caller routine is reponsible for making up this part of ordering
	// by calling functions in proper order.
	enum RotationOrder {	
		XYZ=0, XZY=1, YXZ=2, YZX=3, ZXY=4, ZYX=5
	};
	enum AnglePreference {
		Radians, Degrees
	};

//=============================================================================
// DATA
//=============================================================================
private:
	/** A 4x4 matrix representing the homogenized transformation */
	double	_matrix4[4][4];
	/** Addede a flag to avoid doing matrix multiplication when 3 additions suffice -Ayman 7/06*/
	bool _translationOnly;
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	// Default constructor returns identity transform
	Transform();
	// Copy constructor
	Transform(const Transform &aTransform);
	// Transform from a 4x4 Matrix
	Transform(const double aMat44[4][4]);
	// Construct a transform to rotate around an arbitrary axis with specified angle
	Transform(const double r, const AnglePreference preference, const SimTK::Vec3& axis);
	virtual ~Transform();
	Transform* copy() const;

private:
	void setNull();
	AxisName getRotationAxis(const int i, RotationOrder order);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Transform& operator=(const Transform &aTransform);
#endif
	/** Debugging */
	void printMatrix();
	//--------------------------------------------------------------------------
	// GET AND SET  
	//--------------------------------------------------------------------------
	void getPosition(SimTK::Vec3& pos) const;
	void getPosition(double pos[3]) const
	{
		getPosition(SimTK::Vec3::updAs(pos));
	}
	void setPosition(const SimTK::Vec3& pos);
	void getOrientation(double rOrientation[3][3]) const;
	void setOrientation(const double aOrientation[3][3]);
	void setIdentity();

	void rotate(const double r[3], const AnglePreference preference, const RotationOrder order);
	void rotateX(double r, const AnglePreference preference);
	void rotateY(double r, const AnglePreference preference);
	void rotateZ(double r, const AnglePreference preference);
	void rotateAxis(double r, const AnglePreference preference, const SimTK::Vec3& axis);
	void rotateXBodyFixed(double r, const AnglePreference preference);
	void rotateYBodyFixed(double r, const AnglePreference preference);
	void rotateZBodyFixed(double r, const AnglePreference preference);
	void translateX(const double t);
	void translateY(const double t);
	void translateZ(const double t);
	void translate(const SimTK::Vec3& t);

	void transformPoint(double pt[3]) const;
	void transformPoint(SimTK::Vec3& pt) const;
	void transformVector(double vec[3]) const;
	void transformVector(SimTK::Vec3& vec) const;

	double* getMatrix() { return &_matrix4[0][0]; } // Pete
	void getMatrix(double aMat[]) const;
	void setRotationSubmatrix(double rDirCos[3][3]);
	//--------------------------------------------------------------------------
	// XML
	//--------------------------------------------------------------------------
	//virtual void setupSerializedMembers();

//=============================================================================
};	// END of class Transform

}; //namespace
//=============================================================================
//=============================================================================

#endif //__Transform_h__
