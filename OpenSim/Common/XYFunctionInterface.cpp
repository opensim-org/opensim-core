// XYFunctionInterface.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2009, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "XYFunctionInterface.h"


namespace OpenSim {

bool XYFunctionInterface::isXYFunction(Function* f)
{
	Function* func = f;
	MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(f);
	if (mf)
		func = mf->getFunction();

	if (dynamic_cast<Constant*>(func) ||
		dynamic_cast<StepFunction*>(func) ||
		dynamic_cast<PiecewiseLinearFunction*>(func) ||
		dynamic_cast<LinearFunction*>(func) ||
		dynamic_cast<NaturalCubicSpline*>(func) ||
		dynamic_cast<GCVSpline*>(func))
		return true;

	return false;
}

XYFunctionInterface::XYFunctionInterface(Function* f) :
   _functionType(typeUndefined),
	_constant(0),
   _stepFunction(0),
   _linearFunction(0),
   _natCubicSpline(0),
   _gcvSpline(0)
{
	Function* func;
	MultiplierFunction* mf = dynamic_cast<MultiplierFunction*>(f);
	if (mf) {
		func = mf->getFunction();
		_scaleFactor = mf->getScale();
	} else {
		func = f;
		_scaleFactor = 1.0;
	}

	_constant = dynamic_cast<Constant*>(func);
	if (_constant) {
		_functionType = typeConstant;
		return;
	}
	_stepFunction = dynamic_cast<StepFunction*>(func);
	if (_stepFunction) {
		_functionType = typeStepFunction;
		return;
	}
	_piecewiseLinearFunction = dynamic_cast<PiecewiseLinearFunction*>(func);
	if (_piecewiseLinearFunction) {
		_functionType = typePiecewiseLinearFunction;
		return;
	}
	_linearFunction = dynamic_cast<LinearFunction*>(func);
	if (_linearFunction) {
		_functionType = typeLinearFunction;
		return;
	}
	_natCubicSpline = dynamic_cast<NaturalCubicSpline*>(func);
	if (_natCubicSpline) {
		_functionType = typeNatCubicSpline;
		return;
	}
	_gcvSpline = dynamic_cast<GCVSpline*>(func);
	if (_gcvSpline) {
		_functionType = typeGCVSpline;
		return;
	}

	throw Exception("Object " + getName() + " of type " + getType() + " is not an XYFunction.");
}

int XYFunctionInterface::getNumberOfPoints() const
{
	switch (_functionType)
	{
		case typeConstant:
			return 0;
	   case typeStepFunction:
			return _stepFunction->getNumberOfPoints();
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->getNumberOfPoints();
	   case typeLinearFunction:
		   return 2;
	    case typeNatCubicSpline:
		   return _natCubicSpline->getNumberOfPoints();
	   case typeGCVSpline:
		   return _gcvSpline->getNumberOfPoints();
		default:
			return 0;
	}
}

const double* XYFunctionInterface::getXValues() const
{
	switch (_functionType)
	{
		case typeConstant:
			return NULL;
	   case typeStepFunction:
			return _stepFunction->getXValues();
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->getXValues();
	   case typeLinearFunction:
			{
				double* xValues = new double[2];
				xValues[0] = -1.0;
				xValues[1] = 1.0;
				return xValues; // possible memory leak
			}
		case typeNatCubicSpline:
		   return _natCubicSpline->getXValues();
	   case typeGCVSpline:
		   return _gcvSpline->getXValues();
		default:
			return 0;
	}
}

const double* XYFunctionInterface::getYValues() const
{
	const double* yValues = NULL;
	double* tmp = NULL;
	int numPoints = getNumberOfPoints();

	switch (_functionType)
	{
		case typeConstant:
			return NULL;
	   case typeStepFunction:
			yValues = _stepFunction->getYValues();
			break;
	   case typePiecewiseLinearFunction:
		   yValues = _piecewiseLinearFunction->getYValues();
			break;
	   case typeLinearFunction:
		   tmp = new double[2];
			tmp[0] = _linearFunction->getCoefficients()[1] - _linearFunction->getCoefficients()[0];
			tmp[1] = _linearFunction->getCoefficients()[1] + _linearFunction->getCoefficients()[0];
			break;
	    case typeNatCubicSpline:
		   yValues = _natCubicSpline->getYValues();
			break;
	   case typeGCVSpline:
		   yValues = _gcvSpline->getYValues();
			break;
		default:
			return NULL;
	}

	double* scaledY = new double[numPoints];
	memcpy(scaledY, yValues, numPoints*sizeof(double));
	for (int i=0; i<numPoints; i++)
		scaledY[i] *= _scaleFactor;

	if (tmp)
		delete tmp;

	// possible memory leak
	return scaledY;
}

double XYFunctionInterface::getX(int aIndex) const
{
	switch (_functionType)
	{
		case typeConstant:
			return 0; //_constant->getX(aIndex);
	   case typeStepFunction:
			return _stepFunction->getX(aIndex);
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->getX(aIndex);
	   case typeLinearFunction:
		   if (aIndex == 0)
				return -1.0;
			else if (aIndex == 1)
				return 1.0;
			else
				return 0.0;
		case typeNatCubicSpline:
		   return _natCubicSpline->getX(aIndex);
	   case typeGCVSpline:
		   return _gcvSpline->getX(aIndex);
		default:
			return 0.0;
	}
}

double XYFunctionInterface::getY(int aIndex) const
{
	switch (_functionType)
	{
		case typeConstant:
			return _constant->getValue() * _scaleFactor;
	   case typeStepFunction:
			return _stepFunction->getY(aIndex) * _scaleFactor;
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->getY(aIndex) * _scaleFactor;
	   case typeLinearFunction:
		   if (aIndex == 0)
				return (_linearFunction->getCoefficients()[1] - _linearFunction->getCoefficients()[0]) * _scaleFactor;
			else if (aIndex == 1)
				return (_linearFunction->getCoefficients()[1] + _linearFunction->getCoefficients()[0]) * _scaleFactor;
			else
				return 0.0;
	    case typeNatCubicSpline:
		   return _natCubicSpline->getY(aIndex) * _scaleFactor;
	   case typeGCVSpline:
		   return _gcvSpline->getY(aIndex) * _scaleFactor;
		default:
			return 0.0;
	}
}

void XYFunctionInterface::setX(int aIndex, double aValue)
{
	switch (_functionType)
	{
		case typeConstant:
			break;
	   case typeStepFunction:
		   _stepFunction->setX(aIndex, aValue);
		   break;
	   case typePiecewiseLinearFunction:
		   _piecewiseLinearFunction->setX(aIndex, aValue);
		   break;
	   case typeLinearFunction:
		   break;
	    case typeNatCubicSpline:
		   _natCubicSpline->setX(aIndex, aValue);
		   break;
	   case typeGCVSpline:
		   _gcvSpline->setX(aIndex, aValue);
		   break;
	}
}

void XYFunctionInterface::setY(int aIndex, double aValue)
{
	aValue /= _scaleFactor;

	switch (_functionType)
	{
		case typeConstant:
			break;
	   case typeStepFunction:
		   _stepFunction->setY(aIndex, aValue);
		   break;
	   case typePiecewiseLinearFunction:
		   _piecewiseLinearFunction->setY(aIndex, aValue);
		   break;
		case typeLinearFunction:
			break;
		case typeNatCubicSpline:
		   _natCubicSpline->setY(aIndex, aValue);
		   break;
	   case typeGCVSpline:
		   _gcvSpline->setY(aIndex, aValue);
		   break;
	}
}

bool XYFunctionInterface::deletePoint(int aIndex)
{
	switch (_functionType)
	{
		case typeConstant:
			return true;
	   case typeStepFunction:
		   return _stepFunction->deletePoint(aIndex);
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->deletePoint(aIndex);
	   case typeLinearFunction:
		   return false;
		case typeNatCubicSpline:
		   return _natCubicSpline->deletePoint(aIndex);
	   case typeGCVSpline:
		   return _gcvSpline->deletePoint(aIndex);
		default:
			return true;
	}
}

bool XYFunctionInterface::deletePoints(const Array<int>& indices)
{
	switch (_functionType)
	{
		case typeConstant:
			return true;
	   case typeStepFunction:
		   return _stepFunction->deletePoints(indices);
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->deletePoints(indices);
	   case typeLinearFunction:
		   return false;
		case typeNatCubicSpline:
		   return _natCubicSpline->deletePoints(indices);
	   case typeGCVSpline:
		   return _gcvSpline->deletePoints(indices);
		default:
			return true;
	}
}

int XYFunctionInterface::addPoint(double aX, double aY)
{
	aY /= _scaleFactor;

	switch (_functionType)
	{
		case typeConstant:
			return 1;
	   case typeStepFunction:
		   return _stepFunction->addPoint(aX, aY);
	   case typePiecewiseLinearFunction:
		   return _piecewiseLinearFunction->addPoint(aX, aY);
	   case typeLinearFunction:
		   return 0;
		case typeNatCubicSpline:
		   return _natCubicSpline->addPoint(aX, aY);
	   case typeGCVSpline:
		   return _gcvSpline->addPoint(aX, aY);
		default:
			return 1;
	}
}

Array<XYPoint>* XYFunctionInterface::renderAsLineSegments(int aIndex)
{
	if (_functionType == typeUndefined || _functionType == typeConstant ||
		_functionType == typeLinearFunction || aIndex < 0 || aIndex >= getNumberOfPoints() - 1)
		return NULL;

	Array<XYPoint>* xyPts = new Array<XYPoint>(XYPoint());
	const double* x = getXValues();
	const double* y = getYValues();

	if (_functionType == typeStepFunction)	{
		xyPts->append(XYPoint(x[aIndex], y[aIndex]));
		xyPts->append(XYPoint(x[aIndex+1], y[aIndex]));
		xyPts->append(XYPoint(x[aIndex+1], y[aIndex+1]));
	} else if (_functionType == typePiecewiseLinearFunction) {
		xyPts->append(XYPoint(x[aIndex], y[aIndex]));
		xyPts->append(XYPoint(x[aIndex+1], y[aIndex+1]));
	} else if (_functionType == typeNatCubicSpline) {
		// X sometimes goes slightly beyond the range due to roundoff error,
		// so do the last point separately.
		int numSegs = 20;
		for (int i=0; i<numSegs-1; i++) {
			double xValue = x[aIndex] + (double)i * (x[aIndex + 1] - x[aIndex]) / ((double)numSegs - 1.0);
			xyPts->append(XYPoint(xValue, _natCubicSpline->calcValue(SimTK::Vector(1,xValue)) * _scaleFactor));
		}
		xyPts->append(XYPoint(x[aIndex + 1], _natCubicSpline->calcValue(SimTK::Vector(1,x[aIndex+1])) * _scaleFactor));
	} else if (_functionType == typeGCVSpline) {
		// X sometimes goes slightly beyond the range due to roundoff error,
		// so do the last point separately.
		int numSegs = 20;
		for (int i=0; i<numSegs-1; i++) {
			double xValue = x[aIndex] + (double)i * (x[aIndex + 1] - x[aIndex]) / ((double)numSegs - 1.0);
			xyPts->append(XYPoint(xValue, _gcvSpline->calcValue(SimTK::Vector(1,xValue)) * _scaleFactor));
		}
		xyPts->append(XYPoint(x[aIndex + 1], _gcvSpline->calcValue(SimTK::Vector(1,x[aIndex + 1])) * _scaleFactor));
	}

	return xyPts;
}

}	// namespace
