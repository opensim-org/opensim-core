/* -------------------------------------------------------------------------- *
 *                     OpenSim:  XYFunctionInterface.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "XYFunctionInterface.h"
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/LinearFunction.h>
#include <OpenSim/Common/MultiplierFunction.h>
#include <OpenSim/Common/PiecewiseConstantFunction.h>
#include <OpenSim/Common/PiecewiseLinearFunction.h>
#include <OpenSim/Common/SimmSpline.h>
#include <OpenSim/Common/StepFunction.h>


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
        dynamic_cast<SimmSpline*>(func) ||
        dynamic_cast<GCVSpline*>(func)||
        dynamic_cast<PiecewiseConstantFunction*>(func))
        return true;

    return false;
}

XYFunctionInterface::XYFunctionInterface(Function* f) :
   _functionType(typeUndefined),
    _constant(0),
   _stepFunction(0),
   _linearFunction(0),
   _natCubicSpline(0),
   _gcvSpline(0),
   _mStepFunction(0),
   _genericFunction(0)
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
    _mStepFunction = dynamic_cast<PiecewiseConstantFunction*>(func);
    if (_mStepFunction) {
        _functionType = typePiecewiseConstantFunction;
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
    _natCubicSpline = dynamic_cast<SimmSpline*>(func);
    if (_natCubicSpline) {
        _functionType = typeNatCubicSpline;
        return;
    }
    _gcvSpline = dynamic_cast<GCVSpline*>(func);
    if (_gcvSpline) {
        _functionType = typeGCVSpline;
        return;
    }
    _genericFunction = func;
    
}

bool XYFunctionInterface::isSpecifiedByControlPoints() const 
{
    switch (_functionType)
    {
       case typeGCVSpline:
       case typeNatCubicSpline:
           return true;
       case typeConstant:
       case typePiecewiseConstantFunction:
       case typePiecewiseLinearFunction:
       case typeLinearFunction:
       default:
            return false;
    }
}
int XYFunctionInterface::getNumberOfPoints() const
{
    switch (_functionType)
    {
        case typeConstant:
            return 0;
       case typePiecewiseConstantFunction:
            return _mStepFunction->getNumberOfPoints();
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
            return NULL;
       case typePiecewiseConstantFunction:
            return _mStepFunction->getXValues();
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
            return NULL;;
            break;
       case typePiecewiseConstantFunction:
            yValues = _mStepFunction->getYValues();
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
    if (tmp){
        scaledY[0] = tmp[0];
        scaledY[1] = tmp[1];
    }
    else
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
            return 0; //_stepFunction->getX(aIndex);
        case typePiecewiseConstantFunction:
            return _mStepFunction->getX(aIndex);
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
           return SimTK::NaN;
       case typePiecewiseConstantFunction:
           return _mStepFunction->getY(aIndex) * _scaleFactor;
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
        case typePiecewiseConstantFunction:
           _mStepFunction->setX(aIndex, aValue);
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
        default:
            return;
    }
}

void XYFunctionInterface::setY(int aIndex, double aValue)
{
    aValue /= _scaleFactor;

    switch (_functionType)
    {
        case typeConstant:
            break;
       case typePiecewiseConstantFunction:
           _mStepFunction->setY(aIndex, aValue);
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
        default:
            return;
    }
}

bool XYFunctionInterface::deletePoint(int aIndex)
{
    switch (_functionType)
    {
        case typeConstant:
            return true;
       case typeStepFunction:
           return false;
       case typePiecewiseConstantFunction:
           return _mStepFunction->deletePoint(aIndex);
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
           return false;
       case typePiecewiseConstantFunction:
           return _mStepFunction->deletePoints(indices);
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
           return 0;
       case typePiecewiseConstantFunction:
           return _mStepFunction->addPoint(aX, aY);
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
        aIndex < 0 || aIndex >= getNumberOfPoints() - 1)
        return NULL;

    Array<XYPoint>* xyPts = new Array<XYPoint>(XYPoint());
    const double* x = getXValues();
    const double* y = getYValues();

    if (_functionType == typeStepFunction)  {
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
    } else if (_functionType == typePiecewiseConstantFunction)  {
        xyPts->append(XYPoint(x[aIndex], y[aIndex]));
        xyPts->append(XYPoint(x[aIndex], y[aIndex+1]));
        xyPts->append(XYPoint(x[aIndex+1], y[aIndex+1]));
    } else if (_functionType == typeLinearFunction) {
        xyPts->append(XYPoint(x[aIndex], y[aIndex]));
        xyPts->append(XYPoint(x[aIndex+1], y[aIndex+1]));
    }

    return xyPts;
}

}   // namespace
