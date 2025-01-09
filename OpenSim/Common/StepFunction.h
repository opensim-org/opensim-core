#ifndef OPENSIM_STEP_FUNCTION_H_
#define OPENSIM_STEP_FUNCTION_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  StepFunction.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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


// INCLUDES
#include "Function.h"
#include "PropertyDbl.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for representing a StepFunction.
 *
 * Defines a piecewise function \f(f: \mathbb{R}\mapsto\mathbb{R}\f), with a
 * smooth "S"-shaped quintic polynomial \f$S(t)\f$ from
 * (\c startTime, \c startValue) to (\c endTime, \c endValue).
 * The function is defined as follows:
 * \f[f(t) =
 * \begin{cases}
 * startValue, & t\leq startTime\\
 * S(t), & startTime < t \leq endTime\\
 * endValue, & endTime < t
 * \end{cases}\f]
 *
 * The function has the following shape:
 * 
 * \image html fig_StepFunction.png
 *
 * This class inherits from Function and so can be used as input to
 * any class requiring a Function as input.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMCOMMON_API StepFunction : public Function {
OpenSim_DECLARE_CONCRETE_OBJECT(StepFunction, Function);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
    PropertyDbl _startTimeProp;
    double &_startTime;

    PropertyDbl _endTimeProp;
    double &_endTime;

    PropertyDbl _startValueProp;
    double &_startValue;

    PropertyDbl _endValueProp;
    double &_endValue;

//=============================================================================
// METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    StepFunction();
    StepFunction(double startTime, double endTime, double startValue=0.0, double endValue=1.0);
    StepFunction(const StepFunction &aSpline);
    virtual ~StepFunction();

private:
    void setNull();
    void setupProperties();
    void copyData(const StepFunction &aStepFunction);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    StepFunction& operator=(const StepFunction &aStepFunction);
#endif

    //--------------------------------------------------------------------------
    // SET AND GET Coefficients
    //--------------------------------------------------------------------------
public:
    /** %Set step transition start time */
    void setStartTime(double time)
        { _startTime = time; };
    /** Get step transition time */
    double getStartTime() const
        { return _startTime; };

    /** %Set step transition end time */
    void setEndTime(double time)
        { _endTime = time; };
    /** Get step transition time */
    double getEndTime() const
        { return _endTime; };

    /** %Set start value before step */
    void setStartValue(double start)
        { _startValue = start; };
    /** Get start value before step */
    double getStartValue() const
        { return _startValue; };

    /** %Set end value before step */
    void setEndValue(double end)
        { _endValue = end; };
    /** Get end value before step */
    double getEndValue() const
        { return _endValue; };

    //--------------------------------------------------------------------------
    // EVALUATION
    //--------------------------------------------------------------------------
    SimTK::Function* createSimTKFunction() const override;

//=============================================================================
};  // END class StepFunction
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif  // OPENSIM_STEP_FUNCTION_H_
