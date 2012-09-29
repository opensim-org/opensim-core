/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ActiveForceLengthCurve.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// CONSTRUCTION
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, 
// copy assignment.

// Default constructor.
ActiveForceLengthCurve::ActiveForceLengthCurve()
{
    setNull();
    constructProperties();
    setName("default_ActiveForceLengthCurve");

    ensureCurveUpToDate();
}

// Constructor with enough info to maek the curve.
ActiveForceLengthCurve::ActiveForceLengthCurve(double minActiveNormFiberLength, 
                                               double transitionNormFiberLength,
                                               double maxActiveNormFiberLength,
                                               double shallowAscendingSlope,
                                               double minValue,
                                               const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_ActiveForceLengthCurve");

    set_min_norm_active_fiber_length(minActiveNormFiberLength);
    set_transition_norm_fiber_length(transitionNormFiberLength);
    set_max_norm_active_fiber_length(maxActiveNormFiberLength);
    set_shallow_ascending_slope(shallowAscendingSlope);
    set_minimum_value(minValue);

    ensureCurveUpToDate();
}

void ActiveForceLengthCurve::setNull()
{
    
    setAuthors("Matthew Millard");
}

void ActiveForceLengthCurve::constructProperties()
{
    constructProperty_min_norm_active_fiber_length(0.47-0.0259);//actual: 0.47
        //but added width for the corner is 0.0259. See
        //SmoothSegmentedFunctionFactory createActiveForceLengthCurve 
        //implementation for details
    constructProperty_transition_norm_fiber_length(0.6259); 

    //X Bridge Maximum Fiber Length: 1.57 + 0.0259
    //Gollapudi & Lin Max Fiber Length: 1.8123
    constructProperty_max_norm_active_fiber_length(1.8123);
    constructProperty_shallow_ascending_slope(0.8616);
    constructProperty_minimum_value(0.1); 
}

void ActiveForceLengthCurve::buildCurve()
{
    
        double lce0 = get_min_norm_active_fiber_length();
        double lce1 = get_transition_norm_fiber_length();
        double lce2 = 1.0;
        double lce3 = get_max_norm_active_fiber_length();

        double dydx = get_shallow_ascending_slope();
        double minVal=get_minimum_value();

        double curviness = 1;

        //Here's where you call the SmoothSegmentedFunctionFactory
        SmoothSegmentedFunction tmp = 
            SmoothSegmentedFunctionFactory::
            createFiberActiveForceLengthCurve(  lce0, lce1,
                                                lce2, lce3,
                                              minVal, dydx,
                                           curviness, false, 
                                            getName());
        this->m_curve = tmp;
        setObjectIsUpToDateWithProperties();
    
}

void ActiveForceLengthCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties()==false){
        buildCurve();
    }
}

//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void ActiveForceLengthCurve::connectToModel(Model& aModel)
{
    Super::connectToModel(aModel);
    ensureCurveUpToDate();
}

void ActiveForceLengthCurve::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void ActiveForceLengthCurve::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "ActiveForceLengthCurve: Curve is not"
        " to date with its properties");
    
}



//=============================================================================
// GET & SET METHODS
//=============================================================================


double ActiveForceLengthCurve::getMinActiveFiberLength() const
{ 
    return get_min_norm_active_fiber_length();
}

double ActiveForceLengthCurve::getTransitionFiberLength() const
{
    return get_transition_norm_fiber_length();
}

double ActiveForceLengthCurve::getMaxActiveFiberLength() const
{
    return get_max_norm_active_fiber_length();
}

double ActiveForceLengthCurve::getShallowAscendingSlope() const
{
    return get_shallow_ascending_slope();
}

double ActiveForceLengthCurve::getMinValue() const
{
    return get_minimum_value();
}



void ActiveForceLengthCurve::
    setActiveFiberLengths(  double minActiveNormFiberLength,
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


//=============================================================================
// SERVICES
//=============================================================================

double ActiveForceLengthCurve::calcValue(double normFiberLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "ActiveForceLengthCurve: Curve is not"
        " to date with its properties");

    return m_curve.calcValue(normFiberLength);
}

double ActiveForceLengthCurve::calcDerivative(double normFiberLength, 
                                                 int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "ActiveForceLengthCurve: Curve is not"
        " to date with its properties");

    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ActiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    return m_curve.calcDerivative(normFiberLength,order);
}

SimTK::Vec2 ActiveForceLengthCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "ActiveForceLengthCurve: Curve is not"
        " to date with its properties");  
    
    return m_curve.getCurveDomain();
}

void ActiveForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path)
{   
    ensureCurveUpToDate();    
    m_curve.printMuscleCurveToCSVFile(path);
}
