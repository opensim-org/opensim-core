#ifndef _VectorGCVSplineR1R3_h_
#define _VectorGCVSplineR1R3_h_
// VectorGCVSplineR1R3.h
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
 * Author:  Frank C. Anderson
 
 */



// INCLUDES
#include "osimCommon.h"
#include "Array.h"
#include "VectorFunction.h"
#include "GCVSpline.h"


//template class OSIMCOMMON_API Array<double>;


//=============================================================================
//=============================================================================
/**
 * A class for representing a smooth function with a generalized
 * cross-validation spline.  Linear, cubic, qunitic, and heptic splines
 * are supported:
 *
 *    m (half-order)     order         degree         description
 *    1                    2             1              linear
 *    2                    4             3              cubic
 *    3                    6             5              quintic
 *    4                    8             7              heptic
 *
 * This class wraps the gcvspl.c source code written by D. Twisk in 1994,
 * which is based on the GCVSPL code written in Fortran by Woltring
 * in 1985_07_04.  This class was initially based on a spline class
 * authored by Darryl Thelen and Victor Ng; it has been rewritten to fit
 * into the Realistic Dynamics, Inc. software framework.
 *
 * See the following source for details on how the GCV spline is fit:
 * Woltring, H.J. (1986).  A Fortran package for generalized,
 * cross-validatory spline smoothing and differentiation.  Advances in
 * Engineering Software, Vol. 8, No. 2, 104-113.
 *
 * This class inherits from VectorFunction and so can be used as input to
 * any class requiring an rdVectorFuction as input.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API VectorGCVSplineR1R3 : public VectorFunction
{
//=============================================================================
// DATA
//=============================================================================
private:
	/** Holds return value for evaluate */
	Array<double> _value;
protected:
	/** Spline containing 1st dependent variable data. */
	GCVSpline* _splineY0;
	/** Spline containing 2nd dependent variable data. */
	GCVSpline* _splineY1;
	/** Spline containing 3rd dependent variable data. */
	GCVSpline* _splineY2;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	VectorGCVSplineR1R3();
	VectorGCVSplineR1R3(int aDegree,int aN,const double *aTimes,double *aY0Values,
		double *aY1Values,double *aY2Values,const char *aName=NULL,
		double aErrorVariance=0.0);
	VectorGCVSplineR1R3(const VectorGCVSplineR1R3 &aVectorSpline);
	virtual ~VectorGCVSplineR1R3();
	virtual Object* copy() const;
private:
	void setNull();
	void setEqual(const VectorGCVSplineR1R3 &aSpline);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	VectorGCVSplineR1R3& operator=(const VectorGCVSplineR1R3 &aVectorSpline);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
	void setDegree(int aDegree);
public:
	GCVSpline* getSplineY0() const;
	GCVSpline* getSplineY1() const;
	GCVSpline* getSplineY2() const;

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual void evaluate(const double *aX,double *rY);
	virtual void evaluate(const Array<double> &aX,Array<double> &rY);
	virtual void evaluate(const Array<double> &aX,Array<double> &rY,
		const Array<int> &aDerivWRT);

//=============================================================================
};	// END class VectorGCVSplineR1R3

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __VectorGCVSplineR1R3_h__
