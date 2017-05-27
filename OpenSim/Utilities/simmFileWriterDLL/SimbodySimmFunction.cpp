/* -------------------------------------------------------------------------- *
 *                          SimbodySimmFunction.cpp                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
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
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <time.h>

#include "SimbodySimmFunction.h"
#include <OpenSim/Common/XYFunctionInterface.h>


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using namespace SimTK;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SimbodySimmFunction::~SimbodySimmFunction()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SimbodySimmFunction::SimbodySimmFunction()
{
   _function = NULL;
   _userNumber = -1;
   _XType = Coordinate::Translational;
   _YType = Coordinate::Translational;
}

//_____________________________________________________________________________
/**
 * Constructor from a Function.
 *
 * @param aFunction Function that this object is based on.
 * @param aUserNumber User-defined number of the function.
 * @param aXType Type (translational, rotational) of the function's X axis
 * @param aYType Type (translational, rotational) of the function's Y axis
 */
SimbodySimmFunction::SimbodySimmFunction(const OpenSim::Function* aFunction, int aUserNumber,
                                         Coordinate::MotionType aXType,
                                         Coordinate::MotionType aYType)
{
   _function = aFunction;
   _userNumber = aUserNumber;
   _XType = aXType;
   _YType = aYType;
}

//_____________________________________________________________________________
/**
 * Write the Function to a [SIMM joint] file.
 *
 * @param aStream The file to write to.
 */
void SimbodySimmFunction::write(ofstream& aStream)
{
    XYFunctionInterface xyFunc((OpenSim::Function*)_function);

    if (xyFunc.getFunctionType() == XYFunctionInterface::typeGCVSpline) {
        aStream << "begingcvspline f" << _userNumber << endl;
        for (int i=0; i<xyFunc.getNumberOfPoints(); i++) {
            double x = xyFunc.getX(i);
            if (_XType == Coordinate::Rotational)
                x *= 180.0 / SimTK::Pi;
            double y = xyFunc.getY(i);
            if (_YType == Coordinate::Rotational)
                y *= 180.0 / SimTK::Pi;
            aStream << "(" << x << "," << y << ")" << endl;
        }
        aStream << "endgcvspline" << endl << endl;
    } else if (xyFunc.getFunctionType() == XYFunctionInterface::typeNatCubicSpline) {
        aStream << "beginnaturalcubicspline f" << _userNumber << endl;
        for (int i=0; i<xyFunc.getNumberOfPoints(); i++) {
            double x = xyFunc.getX(i);
            if (_XType == Coordinate::Rotational)
                x *= 180.0 / SimTK::Pi;
            double y = xyFunc.getY(i);
            if (_YType == Coordinate::Rotational)
                y *= 180.0 / SimTK::Pi;
            aStream << "(" << x << "," << y << ")" << endl;
        }
        aStream << "endnaturalcubicspline" << endl << endl;
    } else if (xyFunc.getFunctionType() == XYFunctionInterface::typePiecewiseLinearFunction ||
                xyFunc.getFunctionType() == XYFunctionInterface::typeLinearFunction) {
        aStream << "beginlinearfunction f" << _userNumber << endl;
        for (int i=0; i<xyFunc.getNumberOfPoints(); i++) {
            double x = xyFunc.getX(i);
            if (_XType == Coordinate::Rotational)
                x *= 180.0 / SimTK::Pi;
            double y = xyFunc.getY(i);
            if (_YType == Coordinate::Rotational)
                y *= 180.0 / SimTK::Pi;
            aStream << "(" << x << "," << y << ")" << endl;
        }
        aStream << "endlinearfunction" << endl << endl;
    } else if (xyFunc.getFunctionType() == XYFunctionInterface::typeStepFunction) {
        aStream << "beginstepfunction f" << _userNumber << endl;
        for (int i=0; i<xyFunc.getNumberOfPoints(); i++) {
            double x = xyFunc.getX(i);
            if (_XType == Coordinate::Rotational)
                x *= 180.0 / SimTK::Pi;
            double y = xyFunc.getY(i);
            if (_YType == Coordinate::Rotational)
                y *= 180.0 / SimTK::Pi;
            aStream << "(" << x << "," << y << ")" << endl;
        }
        aStream << "endstepfunction" << endl << endl;
    }
}
