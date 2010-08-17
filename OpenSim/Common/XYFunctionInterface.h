#ifndef __XYFunctionInterfaceInterface_h__
#define __XYFunctionInterfaceInterface_h__

// XYFunctionInterfaceInterface.h
// Authors: Peter Loan
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
 * Use of the OpenSim software in source form is permitted provided that the following
 * conditions are met:
 *   1. The software is used only for non-commercial research and education. It may not
 *     be used in relation to any commercial activity.
 *   2. The software is not distributed or redistributed.  Software distribution is allowed 
 *     only through https://simtk.org/home/opensim.
 *   3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
 *      presentations, or documents describing work in which OpenSim or derivatives are used.
 *   4. Credits to developers may not be removed from executables
 *     created from modifications of the source.
 *   5. Modifications of source code must retain the above copyright notice, this list of
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

#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/StepFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/BlockFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/NaturalCubicSpline.h>
#include <OpenSim/Common/MultiplierFunction.h>

namespace OpenSim {

#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
        #define OSIMCOMMON_API
    #endif
#endif

#if 1
class XYPoint {
public:
	double _x;
	double _y;
	XYPoint() { _x = _y = 0.0; }
	XYPoint(double aX, double aY) { _x = aX; _y = aY; }

#ifndef SWIG
	bool operator==(const XYPoint &aXYPoint) const { return false; }
	bool operator<(const XYPoint &aXYPoint) const { return false; }
#endif

};
#endif
// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond
class OSIMCOMMON_API XYFunctionInterface : public Object {
public:
	enum FunctionType
	{
		typeConstant,
		typeStepFunction,
		typePiecewiseLinearFunction,
		typeLinearFunction,
		typeNatCubicSpline,
		typeGCVSpline,
		typeUndefined
	};

//=============================================================================
// DATA
//=============================================================================

private:
	FunctionType _functionType;

	Constant* _constant;
	StepFunction* _stepFunction;
	PiecewiseLinearFunction* _piecewiseLinearFunction;
	LinearFunction* _linearFunction;
	NaturalCubicSpline* _natCubicSpline;
	GCVSpline* _gcvSpline;

	double _scaleFactor;  // = 1.0 unless function is a MultiplierFunction

public:
   static bool isXYFunction(Function* f);
	XYFunctionInterface(Function* f);
	int getNumberOfPoints() const;
	const double* getXValues() const;
	const double* getYValues() const;
	double getX(int aIndex) const;
	double getY(int aIndex) const;
	void setX(int aIndex, double aValue);
	void setY(int aIndex, double aValue);
	bool deletePoint(int aIndex);
	bool deletePoints(const Array<int>& indices);
	int addPoint(double aX, double aY);
	Array<XYPoint>* renderAsLineSegments(int aIndex);
	static void deleteXYPointArray(Array<XYPoint>* aArray) { if (aArray) delete aArray; }
	FunctionType getFunctionType() const { return _functionType; }
	double getScale() const { return _scaleFactor; }
	// Utility methods for getting the function as each of the supported types
	Constant* getConstant() const { return _constant; }
	StepFunction* getStepFunction() const { return _stepFunction; }
	PiecewiseLinearFunction* getPiecewiseLinearFunction() const { return _piecewiseLinearFunction; }
	LinearFunction* getLinearFunction() const { return _linearFunction; }
	NaturalCubicSpline* getNaturalCubicSpline() const { return _natCubicSpline; }
	GCVSpline* getGCVSpline() const { return _gcvSpline; }

}; // class XYFunctionInterface
/// @endcond
}  // namespace OpenSim

#endif // __XYFunctionInterface_h__

