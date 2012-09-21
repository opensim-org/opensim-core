#ifndef __XYFunctionInterfaceInterface_h__
#define __XYFunctionInterfaceInterface_h__
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  XYFunctionInterface.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/osimCommonDLL.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/StepFunction.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/MultiplierFunction.h>

namespace OpenSim {

#ifdef SWIG
    #ifdef OSIMCOMMON_API
        #undef OSIMCOMMON_API
        #define OSIMCOMMON_API
    #endif
#endif

// Excluding this from Doxygen until it has better documentation! -Sam Hamner
    /// @cond

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

class OSIMCOMMON_API XYFunctionInterface : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(XYFunctionInterface, Object);

public:
	enum FunctionType
	{
		typeConstant,
		typeStepFunction,
		typePiecewiseConstantFunction,
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
	SimmSpline* _natCubicSpline;
	GCVSpline* _gcvSpline;
	PiecewiseConstantFunction* _mStepFunction;
    Function* _genericFunction;

	double _scaleFactor;  // = 1.0 unless function is a MultiplierFunction


public:
    static bool isXYFunction(Function* f);
	XYFunctionInterface(Function* f);

    bool isSpecifiedByControlPoints() const; // Flag to indicate whether function can be edited by changing control points

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
	PiecewiseConstantFunction* getMultiStepFunction() const { return _mStepFunction; }
	PiecewiseLinearFunction* getPiecewiseLinearFunction() const { return _piecewiseLinearFunction; }
	LinearFunction* getLinearFunction() const { return _linearFunction; }
	SimmSpline* getSimmSpline() const { return _natCubicSpline; }
	GCVSpline* getGCVSpline() const { return _gcvSpline; }

}; // class XYFunctionInterface
/// @endcond
}  // namespace OpenSim

#endif // __XYFunctionInterface_h__

