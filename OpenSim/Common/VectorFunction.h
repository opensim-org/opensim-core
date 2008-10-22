#ifndef _VectorFunction_h_
#define _VectorFunction_h_
// VectorFunction.cpp
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
#include "osimCommonDLL.h"
#include "Object.h"
#include "Array.h"



//=============================================================================
//=============================================================================
/**
 * An abstract class for representing a vector function.
 *
 * A vector function is a relation between some number of independent variables 
 * and some number of dependent values such that for any particular set of
 * independent variables the correct number of dependent variables is returned.
 * Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or diferentiable; the evaluate method returns values between
 * rdMath::MINUS_INFINITY and rdMath::PLUS_INFINITY, or it returns rdMath::getNAN()
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson and Saryn R. Goldberg
 */
namespace OpenSim { 

class OSIMCOMMON_API VectorFunction : public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Number of independent variables */
	int _nX;
	/** Number of dependant variables */
	int _nY;
	/** Array containing minimum allowed values of the independent variables. */
	Array<double> _minX;
	/** Array containing maximum allowed values of the independent variables. */
	Array<double> _maxX;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	VectorFunction();
	VectorFunction(int aNX, int aNY);
	VectorFunction(const VectorFunction &aVectorFunction);
	virtual ~VectorFunction();
	virtual Object* copy() const = 0;
private:
	void setNull();
	void setEqual(const VectorFunction &aVectorFunction);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	VectorFunction& operator=(const VectorFunction &aVectorFunction);

	//--------------------------------------------------------------------------
	// SET AND GET
	//--------------------------------------------------------------------------
private:
	void setNX(int aNX);
	void setNY(int aNY);

public:
	int getNX() const;

	int getNY() const;

	void setMinX(const Array<double> &aMinX);
	const Array<double>& getMinX() const;
	void setMinX(int aXIndex, double aMinX);
	double getMinX(int aXIndex) const;

	void setMaxX(const Array<double> &aMaxX);
	const Array<double>& getMaxX() const;
	void setMaxX(int aXIndex, double aMaxX);
	double getMaxX(int aXIndex) const;
	
	//--------------------------------------------------------------------------
	// EVALUATE
	//--------------------------------------------------------------------------
	virtual void updateBoundingBox();
	virtual void evaluate(const double *aX,double *rY)=0;
	virtual void evaluate(const Array<double> &aX,Array<double> &rY)=0;
	virtual void evaluate(const Array<double> &aX,Array<double> &rY,
		const Array<int> &aDerivWRT)=0;

//=============================================================================
};	// END class VectorFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __VectorFunction_h__
