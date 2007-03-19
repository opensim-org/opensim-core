#ifndef _Transform_h_
#define _Transform_h_

// Transform.h
// Authors: Ayman Habib, Peter Loan
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

#include "osimCommon.h"
#include "Object.h"

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
	// Construct a transform to rotate around an arbitrary axis with specified angle
	Transform(const double r, const AnglePreference preference, const double axis[3]);
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
	void getPosition(double pos[3]) const;
	void setPosition(const double pos[3]);
	void getOrientation(double rOrientation[3][3]) const;
	void setOrientation(const double aOrientation[3][3]);
	void setIdentity();

	void rotate(const double r[3], const AnglePreference preference, const RotationOrder order);
	void rotateX(double r, const AnglePreference preference);
	void rotateY(double r, const AnglePreference preference);
	void rotateZ(double r, const AnglePreference preference);
	void rotateAxis(double r, const AnglePreference preference, const double axis[3]);
	void rotateXBodyFixed(double r, const AnglePreference preference);
	void rotateYBodyFixed(double r, const AnglePreference preference);
	void rotateZBodyFixed(double r, const AnglePreference preference);
	void translateX(const double t);
	void translateY(const double t);
	void translateZ(const double t);
	void translate(const double t[3]);

	void transformPoint(double pt[3]) const;
	void transformPoint(Array<double>& pt) const;
	void transformVector(double vec[3]) const;
	void transformVector(Array<double>& vec) const;

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
