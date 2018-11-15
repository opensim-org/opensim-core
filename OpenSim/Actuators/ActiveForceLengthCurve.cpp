/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ActiveForceLengthCurve.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
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
#include "ActiveForceLengthCurve.h"
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//==============================================================================
// CONSTRUCTION
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment.

// Default constructor.
ActiveForceLengthCurve::ActiveForceLengthCurve()
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());
    ensureCurveUpToDate();
}

// Constructor with enough information to build the curve.
ActiveForceLengthCurve::ActiveForceLengthCurve(double minActiveNormFiberLength,
                                               double transitionNormFiberLength,
                                               double maxActiveNormFiberLength,
                                               double shallowAscendingSlope,
                                               double minimumValue)
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());

    set_min_norm_active_fiber_length(minActiveNormFiberLength);
    set_transition_norm_fiber_length(transitionNormFiberLength);
    set_max_norm_active_fiber_length(maxActiveNormFiberLength);
    set_shallow_ascending_slope(shallowAscendingSlope);
    set_minimum_value(minimumValue);

    ensureCurveUpToDate();
}

void ActiveForceLengthCurve::setNull()
{
    setAuthors("Matthew Millard");
}

void ActiveForceLengthCurve::constructProperties()
{
    // The actual value is 0.47, but 0.0259 is added due to the corner in the
    // curve. See SmoothSegmentedFunctionFactory::createActiveForceLengthCurve
    // for details.
    constructProperty_min_norm_active_fiber_length(0.47-0.0259);
    constructProperty_transition_norm_fiber_length(0.73); //was 0.6259

    // Cross-bridge maximum fiber length: 1.57+0.0259
    // Gollapudi and Lin maximum fiber length: 1.8123
    constructProperty_max_norm_active_fiber_length(1.8123);
    constructProperty_shallow_ascending_slope(0.8616);
    constructProperty_minimum_value(0.1);
}

void ActiveForceLengthCurve::buildCurve()
{
    SimTK::Function* f = createSimTKFunction();
    m_curve = *(static_cast<SmoothSegmentedFunction*>(f));
    delete f;
    setObjectIsUpToDateWithProperties();
}

void ActiveForceLengthCurve::ensureCurveUpToDate()
{
    if(!isObjectUpToDateWithProperties()) {
        buildCurve();
    }
}

//==============================================================================
// OpenSim::Function Interface
//==============================================================================
SimTK::Function* ActiveForceLengthCurve::createSimTKFunction() const
{
    // Back the OpenSim::Function with this SimTK::Function.
    return SmoothSegmentedFunctionFactory::createFiberActiveForceLengthCurve(
                                    get_min_norm_active_fiber_length(),
                                    get_transition_norm_fiber_length(),
                                    1.0,
                                    get_max_norm_active_fiber_length(),
                                    get_minimum_value(),
                                    get_shallow_ascending_slope(),
                                    1.0,
                                    false,
                                    getName());
}

//==============================================================================
// GET AND SET METHODS
//==============================================================================
double ActiveForceLengthCurve::getMinActiveFiberLength() const
{   return get_min_norm_active_fiber_length(); }
double ActiveForceLengthCurve::getTransitionFiberLength() const
{   return get_transition_norm_fiber_length(); }
double ActiveForceLengthCurve::getMaxActiveFiberLength() const
{   return get_max_norm_active_fiber_length(); }
double ActiveForceLengthCurve::getShallowAscendingSlope() const
{   return get_shallow_ascending_slope(); }
double ActiveForceLengthCurve::getMinValue() const
{   return get_minimum_value(); }

void ActiveForceLengthCurve::setActiveFiberLengths(
                                    double minActiveNormFiberLength,
                                    double transitionNormFiberLength,
                                    double maxActiveNormFiberLength,
                                    double shallowAscendingSlope)
{
    set_min_norm_active_fiber_length(minActiveNormFiberLength);
    set_transition_norm_fiber_length(transitionNormFiberLength);
    set_max_norm_active_fiber_length(maxActiveNormFiberLength);
    set_shallow_ascending_slope(shallowAscendingSlope);
    ensureCurveUpToDate();
}

void ActiveForceLengthCurve::setMinValue(double minimumValue)
{
    set_minimum_value(minimumValue);
    ensureCurveUpToDate();
}

//==============================================================================
// SERVICES
//==============================================================================
double ActiveForceLengthCurve::calcValue(double normFiberLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "ActiveForceLengthCurve: Curve is not up-to-date with its properties");
    return m_curve.calcValue(normFiberLength);
}

double ActiveForceLengthCurve::calcDerivative(double normFiberLength,
                                              int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "ActiveForceLengthCurve: Curve is not up-to-date with its properties");
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2,
        "ActiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);

    return m_curve.calcDerivative(normFiberLength,order);
}

double ActiveForceLengthCurve::
    calcDerivative(const std::vector<int>& derivComponents,
                   const SimTK::Vector& x) const
{
    return m_curve.calcDerivative(derivComponents, x);
}

SimTK::Vec2 ActiveForceLengthCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "ActiveForceLengthCurve: Curve is not up-to-date with its properties");

    return m_curve.getCurveDomain();
}

void ActiveForceLengthCurve::printMuscleCurveToCSVFile(const std::string& path)
{
    ensureCurveUpToDate();

    double xmin = min(0.0, get_min_norm_active_fiber_length());
    double xmax = max(2.0, get_max_norm_active_fiber_length());

    m_curve.printMuscleCurveToCSVFile(path,xmin,xmax);
}
