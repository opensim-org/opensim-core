#ifndef _Function_h_
#define _Function_h_
// Function.cpp
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include "osimCommonDLL.h"
#include "Object.h"
#include "PropertyDbl.h"
#include "SimmPoint.h"


//=============================================================================
//=============================================================================
/**
 * An abstract class for representing a function.
 *
 * A function is a relation between independent variables and a dependent
 * value such that for any particular set of independent variables there is
 * only one unique dependent value.  Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or diferentiable; the evaluate method returns values between
 * rdMath::MINUS_INFINITY and rdMath::PLUS_INFINITY, or it returns rdMath::NAN
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API XYPoint
{
public:
	double _x;
	double _y;
	XYPoint() { _x = _y = 0.0; }
	XYPoint(double aX, double aY) { _x = aX; _y = aY; }
	bool operator==(const XYPoint &aXYPoint) const { return false; }
	bool operator<(const XYPoint &aXYPoint) const { return false; }

};

class OSIMCOMMON_API Function : public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	// PROPERTIES
	/** Minimum value of the x independent variable. */
	PropertyDbl _propMinX;
	/** Maximum value of the x independent variable. */
	PropertyDbl _propMaxX;
	/** Minimum value of the y independent variable. */
	PropertyDbl _propMinY;
	/** Maximum value of the y independent variable. */
	PropertyDbl _propMaxY;
	/** Minimum value of the z independent variable. */
	PropertyDbl _propMinZ;
	/** Maximum value of the z independent variable. */
	PropertyDbl _propMaxZ;

	// REFERENCES
	double &_minX;
	double &_maxX;
	double &_minY;
	double &_maxY;
	double &_minZ;
	double &_maxZ;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	Function();
	Function(const Function &aFunction);
	virtual ~Function();
	virtual Object* copy() const = 0;
	virtual void init(int aN, const double *aXValues, const double *aYValues) { }

private:
	void setNull();
	void setupProperties();
	void setEqual(const Function &aFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Function& operator=(const Function &aFunction);
#endif
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	void setMinX(double aMinX);
	double getMinX() const;
	void setMaxX(double aMaxX);
	double getMaxX() const;
	void setMinY(double aMinY);
	double getMinY() const;
	void setMaxY(double aMaxY);
	double getMaxY() const;
	void setMinZ(double aMinZ);
	double getMinZ() const;
	void setMaxZ(double aMaxZ);
	double getMaxZ() const;
	virtual int getNumberOfPoints() const = 0;
	virtual const double* getXValues() const { return NULL; }
	virtual const double* getYValues() const { return NULL; }
	virtual double getX(int aIndex) const = 0;
	virtual double getY(int aIndex) const = 0;
	virtual double getZ(int aIndex) const = 0;
	virtual void setX(int aIndex, double aValue) { }
	virtual void setY(int aIndex, double aValue) { }
	virtual void setZ(int aIndex, double aValue) { }
	virtual bool deletePoint(int aIndex) = 0;
	virtual void addPoint(double aX, double aY) = 0;
	virtual Array<XYPoint>* renderAsLineSegments(double aStart, double aEnd) { return NULL; }
	virtual Array<XYPoint>* renderAsLineSegments(int aIndex) { return NULL; }
	static void deleteXYPointArray(Array<XYPoint>* aArray) { if (aArray) delete aArray; }

	//--------------------------------------------------------------------------
	// UTILITY
	//--------------------------------------------------------------------------
	virtual void
	isLinear(double aTol,
				double aMinX,double aMaxX,double &rMX,
				double aMinY,double aMaxY,double &rMY,
				double aMinZ,double aMaxZ,double &rMZ);
	static Function* makeFunctionOfType(Function* aFunction, const std::string& aNewTypeName);

	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox() = 0;
	virtual double
		evaluate(int aDerivOrder,double aX=0.0,double aY=0.0,double aZ=0.0) = 0;
	virtual double
		evaluateTotalFirstDerivative(double aX,double aDxdt);
	virtual double
		evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2);
	virtual void scaleY(double aScaleFactor) = 0;

	OPENSIM_DECLARE_DERIVED(Function, Object);

//=============================================================================
};	// END class Function

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __Function_h__
