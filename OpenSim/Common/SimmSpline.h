#ifndef OPENSIM_SIMMSPLINE_H_
#define OPENSIM_SIMMSPLINE_H_

// SimmSpline.h
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

class OSIMCOMMON_API SimmSpline : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(SimmSpline, Function);

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
	SimmSpline();
	SimmSpline(int aN,const double *aTimes,const double *aValues,
		const std::string &aName="");
	SimmSpline(const SimmSpline &aSpline);
	virtual ~SimmSpline();

	virtual void init(Function* aFunction);

private:
	void setNull();
	void setupProperties();
	void setEqual(const SimmSpline &aSpline);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	SimmSpline& operator=(const SimmSpline &aSpline);
#endif
	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
public:
	int getSize() const;
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
	virtual bool deletePoint(int aIndex);
	virtual bool deletePoints(const Array<int>& indices);
	virtual int addPoint(double aX, double aY);

	//--------------------------------------------------------------------------
	// EVALUATION
	//--------------------------------------------------------------------------
    double calcValue(const SimTK::Vector& x) const;
    double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const;
    int getArgumentSize() const;
    int getMaxDerivativeOrder() const;
    SimTK::Function* createSimTKFunction() const;

	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

private:
	void calcCoefficients();
//=============================================================================
};	// END class SimmSpline

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_SIMMSPLINE_H_
