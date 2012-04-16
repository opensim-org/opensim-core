#ifndef _PiecewiseConstantFunction_h_
#define _PiecewiseConstantFunction_h_

// PiecewiseConstantFunction.h
// Author: Peter Loan
/*
 * Copyright (c) 2007, Stanford University. All rights reserved. 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions
 * are met: 
 *  - Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer. 
 *  - Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  - Neither the name of the Stanford University nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission. 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE. 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Array.h"
#include "PropertyInt.h"
#include "PropertyDbl.h"
#include "PropertyDblArray.h"
#include "Function.h"
#include "FunctionAdapter.h"


//=============================================================================
//=============================================================================
/**
 * A class implementing a step function.
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Function as input.
 *
 * @author Peter Loan
 * @version 1.0
 */
namespace OpenSim { 

class OSIMCOMMON_API PiecewiseConstantFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(PiecewiseConstantFunction, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
       // PROPERTIES
       /** Array of values for the independent variables (i.e., the knot
       sequence).  This array must be monotonically increasing. */
       PropertyDblArray _propX;
       Array<double> &_x;

       /** Y values. */
       PropertyDblArray _propY;
       Array<double> &_y;

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    PiecewiseConstantFunction();
    PiecewiseConstantFunction(int aN,const double *aTimes,const double *aValues,
            const std::string &aName="");
    PiecewiseConstantFunction(const PiecewiseConstantFunction &aFunction);
    virtual ~PiecewiseConstantFunction();

    virtual void init(Function* aFunction);

private:
    void setNull();
    void setupProperties();
    void setEqual(const PiecewiseConstantFunction &aFunction);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    PiecewiseConstantFunction& operator=(const PiecewiseConstantFunction &aFunction);
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
    virtual double evaluateTotalFirstDerivative(double aX,double aDxdt) const;
    virtual double evaluateTotalSecondDerivative(double aX,double aDxdt,double aD2xdt2) const;
    double calcValue(const SimTK::Vector& x) const;
    double calcDerivative(const std::vector<int>& derivComponents, const SimTK::Vector& x) const;
    int getArgumentSize() const;
    int getMaxDerivativeOrder() const;
    SimTK::Function* createSimTKFunction() const;

//=============================================================================
};     // END class PiecewiseConstantFunction

}; //namespace
//=============================================================================
//=============================================================================

#endif  // __PiecewiseConstantFunction_h__

