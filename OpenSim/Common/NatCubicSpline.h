#ifndef _NatCubicSpline_h_
#define _NatCubicSpline_h_

// NatCubicSpline.h
// Author: Peter Loan
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


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "Function.h"


//=============================================================================
//=============================================================================
/**
 * A class implementing a smooth function with a natural cubic spline.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring an rdFuction as input.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API NatCubicSpline : public Function
{
//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	// PROPERTIES
	/** Array of values for the independent variables (i.e., the spline knot
	sequence).  This array must be monotonically increasing. */
	PropertyDblArray _propX;
	Array<double> &_x;

	/** Y values. */
	PropertyDblArray _propY;
	Array<double> &_y;

private:
	Array<double> _b;
	Array<double> _c;
	Array<double> _d;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	NatCubicSpline();
	NatCubicSpline(int aN,const double *aTimes,const double *aValues,
		const std::string &aName="");
	NatCubicSpline(const NatCubicSpline &aSpline);
	virtual ~NatCubicSpline();
	virtual Object* copy() const;
	virtual void init(Function* aFunction);

private:
	void setNull();
	void setupProperties();
	void setEqual(const NatCubicSpline &aSpline);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	NatCubicSpline& operator=(const NatCubicSpline &aSpline);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	int getSize() const;
	double getMinX() const;
	double getMaxX() const;
	const Array<double>& getX() const;
	const Array<double>& getY() const;
	virtual const double* getXValues() const;
	virtual const double* getYValues() const;
	virtual int getNumberOfPoints() const { return _x.getSize(); }
	virtual double getX(int aIndex) const;
	virtual double getY(int aIndex) const;
	virtual double getZ(int aIndex) const { return 0.0; }
	virtual void setX(int aIndex, double aValue);
	virtual void setY(int aIndex, double aValue);
	virtual void scaleY(double aScaleFactor);
	virtual bool deletePoint(int aIndex);
	virtual bool deletePoints(const Array<int>& indices);
	virtual void addPoint(double aX, double aY);
	virtual Array<XYPoint>* renderAsLineSegments(int aIndex);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual double	evaluate(int aDerivOrder, double aX=0.0, double aY=0.0, double aZ=0.0);
	virtual double evaluateTotalFirstDerivative(double aX,double aDxdt);
	virtual double evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2);

	virtual void updateFromXMLNode();

private:
	double evaluate(double aX, double velocity, double acceleration, int aDerivOrder);
	void calcCoefficients();

	OPENSIM_DECLARE_DERIVED(NatCubicSpline, Function)

//=============================================================================
};	// END class NatCubicSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __NatCubicSpline_h__
