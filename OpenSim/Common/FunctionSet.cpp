/* -------------------------------------------------------------------------- *
 *                         OpenSim:  FunctionSet.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

// INCLUDES
#include "FunctionSet.h"



using namespace OpenSim;
using namespace std;

template class OSIMCOMMON_API OpenSim::Set<OpenSim::Function>;

//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// DESTRUCTOR AND CONSTRUCTORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
FunctionSet::~FunctionSet()
{
}

//_____________________________________________________________________________
/**
 * Default constructor.
 *
 * @param aName Name of the function set.
 */
FunctionSet::FunctionSet() :
    Set<Function>()
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Construct a function set from file.
 *
 * @param aFileName Name of the file.
 */
FunctionSet::FunctionSet(const string &aFileName) :
    Set<Function>(aFileName, false)
{
    setNull();
    updateFromXMLDocument();
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set all member variables to NULL values.
 */
void FunctionSet::
setNull()
{
}

//=============================================================================
// EVALUATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate a function or one of its derivatives.
 *
 * @param aIndex Index of the function to evaluate.
 * @param aDerivOrder Order of the derivative to evaluate.
 * @param aX Value of the x independent variable.
 * @param aY Value of the y independent variable.
 * @param aZ Value of the z independent variable.
 * @return Value of the function.  If the function is NULL or undefined,
 * SimTK::NaN is returned.
 * @see Function
 */
double FunctionSet::
evaluate(int aIndex,int aDerivOrder,double aX) const
{
    Function& func = get(aIndex);

    SimTK::Vector arg = SimTK::Vector(1, aX);
    if (aDerivOrder==0)
        return (func.calcValue(arg));

    std::vector<int> derivComponents;
    for(int i=0; i<aDerivOrder; i++)
        derivComponents.push_back(0);

    return( func.calcDerivative(derivComponents, arg) );
}

//_____________________________________________________________________________
/**
 * Evaluate all the functions in the function set or their derivatives.
 *
 * @param rValues Array containing the values of the functions.
 * @param aDerivOrder Order of the derivative to evaluate.
 * @param aX Value of the x independent variable.
 * @param aY Value of the y independent variable.
 * @param aZ Value of the z independent variable.
 * @return Value of the function.  If the function is NULL or undefined,
 * SimTK::NaN is returned.
 * @see Function
 */
void FunctionSet::
evaluate(Array<double> &rValues,int aDerivOrder,double aX) const
{
    int size = getSize();
    rValues.setSize(size);

    int i;
    for(i=0;i<size;i++) {
        Function& func = get(i);
        if (aDerivOrder==0)
            rValues[i] = func.calcValue(SimTK::Vector(1,aX));
        else {
            std::vector<int> derivComponents;
            for(int j=0; j<aDerivOrder; j++)
                derivComponents.push_back(0);
            rValues[i] = func.calcDerivative(derivComponents, SimTK::Vector(1,aX));

        }
    }
}
