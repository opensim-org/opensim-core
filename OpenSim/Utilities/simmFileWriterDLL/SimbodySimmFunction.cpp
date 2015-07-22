// SimbodySimmFunction.cpp
// Authors: Peter Loan
/*
 * Copyright (c)  2008, Stanford University. All rights reserved.
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

//=============================================================================
// INCLUDES
//=============================================================================
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
