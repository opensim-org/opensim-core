// Transform.cpp
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


#include "rdMath.h"
#include "Mtx.h"
#include "Transform.h"
#include "PropertyDblArray.h"

//=============================================================================
// STATIC VARIABLES
//=============================================================================


using namespace OpenSim;
using namespace std;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
Transform::~Transform()
{
}

//_____________________________________________________________________________
/**
 * Construct an identity Transform.
 *
 */
Transform::Transform()
{
	setNull();

	// TYPE
	setType("Transform");

	for(int i=0; i < 4; i++)
		for(int j=0; j < 4; j++)
			_matrix4[i][j] = (i==j)?1.0:0.0;

	_angles[0] = _angles[1] = _angles[2] = 0.0;
}

//_____________________________________________________________________________
/**
 * Copy constructor
 *
 */
Transform::Transform(const Transform &aTransform):
Object(aTransform)
{
	setNull();

	//Assignment
	(*this) = aTransform;

	for(int i=0; i<3; i++)
		_angles[i] = aTransform._angles[i];
}

//_____________________________________________________________________________
/**
 * Construct a copy of this Transform.
 */
Object* Transform::
copy() const
{
	return(new Transform(*this));
}
// Construct a transform to rotate around an arbitrary axis with specified angle
Transform::Transform(const double r, const AnglePreference preference, const double axis[3])
{
	double aa = axis[X] * axis[X];
	double bb = axis[Y] * axis[Y];
	double cc = axis[Z] * axis[Z];

	double rInRadians = (preference==Radians)? r : (r * rdMath::DTR);
	double sinTheta = sin(rInRadians);
	double cosTheta = cos(rInRadians);
	double omCos = 1.0 - cosTheta;

	_matrix4[X][X] = aa + (1.0 - aa) * cosTheta;
	_matrix4[Y][Y] = bb + (1.0 - bb) * cosTheta;
	_matrix4[Z][Z] = cc + (1.0 - cc) * cosTheta;
	_matrix4[X][Y] = axis[X] * axis[Y] * omCos + axis[Z] * sinTheta;
	_matrix4[X][Z] = axis[X] * axis[Z] * omCos - axis[Y] * sinTheta;
	_matrix4[Y][X] = axis[X] * axis[Y] * omCos - axis[Z] * sinTheta;
	_matrix4[Y][Z] = axis[Y] * axis[Z] * omCos + axis[X] * sinTheta;
	_matrix4[Z][X] = axis[X] * axis[Z] * omCos + axis[Y] * sinTheta;
	_matrix4[Z][Y] = axis[Y] * axis[Z] * omCos - axis[X] * sinTheta;

	_matrix4[X][W] = _matrix4[Y][W] = _matrix4[Z][W] = 0.0;
	_matrix4[W][X] = _matrix4[W][Y] = _matrix4[W][Z] = 0.0;
	_matrix4[W][W] = 1.0;
}

//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void Transform::
setNull()
{
	setupSerializedMembers();

}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Transform::
setupSerializedMembers()
{
}
//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
Transform& Transform::operator=(const Transform &aTransform)
{
	setNull();
	int i;
	for(i=0; i < 4; i++)
		for(int j=0; j < 4; j++)
			_matrix4[i][j]= aTransform._matrix4[i][j];

	for(i=0; i<3; i++)
		_angles[i] = aTransform._angles[i];
	return (*this);

}
//=============================================================================
// SET / GET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the position vector.
 *
 */
void Transform::
getPosition(double pos[3]) const
{
	for(int i=0; i<3; i++)
		pos[i] = _matrix4[3][i];  //JPL 9/15/05: moved translation from last column to last row
}
//_____________________________________________________________________________
/**
 * Set the position vector.
 *
 */
void Transform::
setPosition(const double pos[3])
{
	for(int i=0; i<3; i++)
		_matrix4[3][i]=pos[i];  //JPL 9/15/05: moved translation from last column to last row

}

//_____________________________________________________________________________
/**
 * Get orientation vector from matrix
 *
 * Actually vtk specifies transform using translations and rotations
 * so it's more efficient to store them in this format.
 *
 * @todo check if it's more efficient to make and maintain the matrix and 
 *  if there's an API to set the transformation matrix directly in vtk.
 */
void Transform::
getOrientation(double orientation[3]) const
{
  int i;

  // first rotate about y axis
/*  double x2 = _matrix4[2][0];
  double y2 = _matrix4[2][1];
  double z2 = _matrix4[2][2];

  double x3 = _matrix4[1][0];
  double y3 = _matrix4[1][1];
  double z3 = _matrix4[1][2];

  double d1 = sqrt(x2*x2 + z2*z2);

  double cosTheta, sinTheta;
  if (d1 < rdMath::SMALL) 
    {
    cosTheta = 1.0;
    sinTheta = 0.0;
    }
  else 
    {
    cosTheta = z2/d1;
    sinTheta = x2/d1;
    }

  double theta = atan2(sinTheta, cosTheta);
  orientation[1] = -theta/rdMath::DTR;

  // now rotate about x axis
  double d = sqrt(x2*x2 + y2*y2 + z2*z2);

  double sinPhi, cosPhi;
  if (d < rdMath::SMALL) 
    {
    sinPhi = 0.0;
    cosPhi = 1.0;
    }
  else if (d1 < rdMath::SMALL) 
    {
    sinPhi = y2/d;
    cosPhi = z2/d;
    }
  else 
    {
    sinPhi = y2/d;
    cosPhi = (x2*x2 + z2*z2)/(d1*d);
    }

  double phi = atan2(sinPhi, cosPhi);
  orientation[0] = phi/rdMath::DTR;

  // finally, rotate about z
  double x3p = x3*cosTheta - z3*sinTheta;
  double y3p = - sinPhi*sinTheta*x3 + cosPhi*y3 - sinPhi*cosTheta*z3;
  double d2 = sqrt(x3p*x3p + y3p*y3p);

  double cosAlpha, sinAlpha;
  if (d2 < rdMath::SMALL) 
    {
    cosAlpha = 1.0;
    sinAlpha = 0.0;
    }
  else 
    {
    cosAlpha = y3p/d2;
    sinAlpha = x3p/d2;
    }

  double alpha = atan2(sinAlpha, cosAlpha);
  orientation[2] = alpha/rdMath::DTR;*/
	
	for(i=0; i<3; i++)
		orientation[i]=_angles[i]/rdMath::DTR;


}
// Do we need to pass in Order? This may need to be removed in favor of actual
// translate/rotate combinations
void Transform::
setOrientation(const double orientation[3])
{
	for(int i=0; i<3; i++)
		_angles[i]=orientation[i];

}
//_____________________________________________________________________________
/**
 * Set transform matrix to identity
 *
 */
void Transform::
setIdentity()
{
	Mtx::Identity(4, (double *)_matrix4);
}
//_____________________________________________________________________________
/**
 * Set transform matrix based on angles, rotation order and preference
 *
 */
void Transform::
rotate(const double r[3], const AnglePreference preference, const RotationOrder order)
{
	// Convert angle to radians as this's what low level functions use
	for(int rotAxisIndex=0; rotAxisIndex<3; rotAxisIndex++){
		Transform::AxisName nextAxis = getRotationAxis(rotAxisIndex, order);
		switch(nextAxis){
			case X:
				rotateX(r[rotAxisIndex], preference);
				break;
			case Y:
				rotateY(r[rotAxisIndex], preference);
				break;
			case Z:
				rotateZ(r[rotAxisIndex], preference);
				break;
			case NoAxis:
				break;

		}
	}
}
//_____________________________________________________________________________
/**
 * Rotate by r degrees or radians around X axis
 *
 */
void Transform::
rotateX(double r, const AnglePreference preference)
{
	// Convert angle to radians as this's what low level functions use
	double rInRadians = (preference==Radians)? r : (r * rdMath::DTR);
	double RotationMatrix[4][4];
	Mtx::Identity(4, (double *)RotationMatrix);
		// COMPUTE SIN AND COS
	double c = cos(rInRadians);
	double s = sin(rInRadians);
	RotationMatrix[1][1] = RotationMatrix[2][2] = c;
	RotationMatrix[1][2] = s;
	RotationMatrix[2][1] = -s;
	Mtx::Multiply(4, 4, 4, (double *)_matrix4, (double *)RotationMatrix, (double *)_matrix4);

}
//_____________________________________________________________________________
/**
 * Rotate by r degrees or radians around Y axis
 *
 */
void Transform::
rotateY(double r, const AnglePreference preference)
{
	// Convert angle to radians as this's what low level functions use
	double rInRadians = (preference==Radians)? r : (r * rdMath::DTR);
	double RotationMatrix[4][4];
	Mtx::Identity(4, (double *)RotationMatrix);
		// COMPUTE SIN AND COS
	double c = cos(rInRadians);
	double s = sin(rInRadians);
	RotationMatrix[0][0] = RotationMatrix[2][2] = c;
	RotationMatrix[2][0] = s;
	RotationMatrix[0][2] = -s;
	Mtx::Multiply(4, 4, 4, (double *)_matrix4, (double *)RotationMatrix, (double *)_matrix4);
}
//_____________________________________________________________________________
/**
 * Rotate by r degrees or radians around Z axis
 *
 */
void Transform::
rotateZ(double r, const AnglePreference preference)
{
	// Convert angle to radians as this's what low level functions use
	double rInRadians = (preference==Radians)? r : (r * rdMath::DTR);
	double RotationMatrix[4][4];
	Mtx::Identity(4, (double *)RotationMatrix);
		// COMPUTE SIN AND COS
	double c = cos(rInRadians);
	double s = sin(rInRadians);
	RotationMatrix[0][0] = RotationMatrix[1][1] = c;
	RotationMatrix[0][1] = s;
	RotationMatrix[1][0] = -s;
	Mtx::Multiply(4, 4, 4, (double *)_matrix4, (double *)RotationMatrix, (double *)_matrix4);
}
//_____________________________________________________________________________
/**
 * Rotate by r degrees or radians around arbitrary axis. The axis must be of
 * unit length.
 *
 */
void Transform::
rotateAxis(double r, const AnglePreference preference, const double axis[3])
{
	Transform RotationMatrix(r, preference, axis);
	Mtx::Multiply(4, 4, 4, (double *)_matrix4, RotationMatrix.getMatrix(), (double *)_matrix4);
}
//_____________________________________________________________________________
/**
 * Translate by double in X direction
 *
 */
void Transform::
translateX(const double tX)
{
		_matrix4[3][0] += tX;  //JPL 9/15/05: moved translation from last column to last row
}
//_____________________________________________________________________________
/**
 * Translate by double in Y direction
 *
 */
void Transform::
translateY(const double tY)
{
		_matrix4[3][1] += tY;  //JPL 9/15/05: moved translation from last column to last row
}
//_____________________________________________________________________________
/**
 * Translate by double in Z direction
 *
 */
void Transform::
translateZ(const double tZ)
{
		_matrix4[3][2] += tZ;  //JPL 9/15/05: moved translation from last column to last row
}

//_____________________________________________________________________________
/**
 * Translate by vector t
 *
 */
void Transform::
translate(const double t[3])
{
	for (int i=0; i < 3; i++)
		_matrix4[3][i] += t[i];  //JPL 9/15/05: moved translation from last column to last row
}
/**
 * Return 0 for X, 1 for Y 2 for Z index i starts at 0
 * A careful assignment of numbers to enums will make this much smaller 
 * but much harder to debug or understand!
 * where it can be achieved by bitwise operations
 * eg. XYZ=00.01.10 to indicate X then Y then Z
 * XZY=00.10.01 to indicate X then Z then Y
 * ....
 * then we can return ((order << (2*i))& 03)
 */
Transform::AxisName Transform::
getRotationAxis(const int i, RotationOrder order)
{
	if (i==0){
		switch(order){
			case XYZ:
			case XZY:
				return X;
			case YXZ:
			case YZX:
				return Y;
			case ZXY:
			case ZYX:
				return Z;
			default:
				return NoAxis;
		}
	} 
	else if (i==1) {
		switch(order){
			case YXZ:
			case ZXY:
				return X;
			case XYZ:
			case ZYX:
				return Y;
			case YZX:
			case XZY:
				return Z;
			default:
				return NoAxis;
		}

	} 
	else if (i==2) {
		switch(order){
			case ZYX:
			case YZX:
				return X;
			case XZY:
			case ZXY:
				return Y;
			case XYZ:
			case YXZ:
				return Z;
			default:
				return NoAxis;
		}

	} 
	else 
		return NoAxis;
}

void Transform::transformPoint(double pt[3]) const
{
	double tx = pt[X] * _matrix4[0][0] + pt[Y] * _matrix4[1][0] + pt[Z] * _matrix4[2][0] + _matrix4[3][0];
	double ty = pt[X] * _matrix4[0][1] + pt[Y] * _matrix4[1][1] + pt[Z] * _matrix4[2][1] + _matrix4[3][1];

	pt[Z] = pt[X] * _matrix4[0][2] + pt[Y] * _matrix4[1][2] + pt[Z] * _matrix4[2][2] + _matrix4[3][2];
	pt[X] = tx;
	pt[Y] = ty;
}

void Transform::transformPoint(Array<double>& pt) const
{
	double tx = pt[X] * _matrix4[0][0] + pt[Y] * _matrix4[1][0] + pt[Z] * _matrix4[2][0] + _matrix4[3][0];
	double ty = pt[X] * _matrix4[0][1] + pt[Y] * _matrix4[1][1] + pt[Z] * _matrix4[2][1] + _matrix4[3][1];

	pt[Z] = pt[X] * _matrix4[0][2] + pt[Y] * _matrix4[1][2] + pt[Z] * _matrix4[2][2] + _matrix4[3][2];
	pt[X] = tx;
	pt[Y] = ty;
}

void Transform::transformVector(double vec[3]) const
{
	double tx = vec[X] * _matrix4[0][0] + vec[Y] * _matrix4[1][0] + vec[Z] * _matrix4[2][0];
	double ty = vec[X] * _matrix4[0][1] + vec[Y] * _matrix4[1][1] + vec[Z] * _matrix4[2][1];

	vec[Z] = vec[X] * _matrix4[0][2] + vec[Y] * _matrix4[1][2] + vec[Z] * _matrix4[2][2];
	vec[X] = tx;
	vec[Y] = ty;
}

/**
 * Debugging 
 */
void Transform::printMatrix()
{
	cout << "Xform: " << endl;
	for(int i=0; i < 4; i++){
		for(int j=0; j < 4; j++){
			cout << _matrix4[i][j]<< " ";
		}
		cout << endl;
	}
	cout << endl << endl;
}