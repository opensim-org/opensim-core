#ifndef OPENSIM_VECTOR_FUNCTION_H_
#define OPENSIM_VECTOR_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  VectorFunction.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Saryn R. Goldberg                            *
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
#include "osimCommonDLL.h"
#include "Object.h"
#include "Array.h"



//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * An abstract class for representing a vector function.
 *
 * A vector function is a relation between some number of independent variables 
 * and some number of dependent values such that for any particular set of
 * independent variables the correct number of dependent variables is returned.
 * Values of the function and its derivatives
 * are obtained by calling the evaluate() method.  The curve may or may not
 * be finite or differentiable; the evaluate method returns values between
 * -`SimTK::Infinity` and `SimTK::Infinity`, or it returns `SimTK::NaN`
 * (not a number) if the curve is not defined.
 * Currently, functions of up to 3 variables (x,y,z) are supported.
 *
 * @author Frank C. Anderson and Saryn R. Goldberg
 */
class OSIMCOMMON_API VectorFunction : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(VectorFunction, Object);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Number of independent variables */
    int _nX;
    /** Number of dependent variables */
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
    virtual void calcValue(const double *aX,double *rY, int aSize)=0;
    virtual void calcValue(const Array<double> &aX,Array<double> &rY)=0;
    virtual void calcDerivative(const Array<double> &aX,Array<double> &rY,
        const Array<int> &aDerivWRT)=0;

//=============================================================================
};  // END class VectorFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // OPENSIM_VECTOR_FUNCTION_H_
