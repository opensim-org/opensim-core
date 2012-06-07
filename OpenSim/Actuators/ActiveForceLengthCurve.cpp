/* Author: Matthew Millard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a    *
 * copy of this software and associated documentation files (the "Software"), *
 * to deal in the Software without restriction, including without limitation  *
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,   *
 * and/or sell copies of the Software, and to permit persons to whom the      *
 * Software is furnished to do so, subject to the following conditions:       *
 *                                                                            *
 * The above copyright notice and this permission notice shall be included in *
 * all copies or substantial portions of the Software.                        *
 *                                                                            *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    *
 * THE AUTHORS, CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,    *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR      *
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE  *
 * USE OR OTHER DEALINGS IN THE SOFTWARE.                                     *
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
    
}

void ActiveForceLengthCurve::constructProperties()
{
    constructProperty_min_norm_active_fiber_length(0.4);
    constructProperty_transition_norm_fiber_length(0.75);
    constructProperty_max_norm_active_fiber_length(1.6);
    constructProperty_shallow_ascending_slope(0.75);
    constructProperty_minimum_value(0.01); 
}

void ActiveForceLengthCurve::buildCurve()
{
    
        double lce0 = get_min_norm_active_fiber_length();
        double lce1 = get_transition_norm_fiber_length();
        double lce2 = 1.0;
        double lce3 = get_max_norm_active_fiber_length();

        double dydx = get_shallow_ascending_slope();
        double minVal=get_minimum_value();

        double curviness = 0.75;

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
}

void ActiveForceLengthCurve::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void ActiveForceLengthCurve::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================


double ActiveForceLengthCurve::getMinActiveFiberLength() const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();  
    return get_min_norm_active_fiber_length();
}

double ActiveForceLengthCurve::getTransitionFiberLength() const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();  
    return get_transition_norm_fiber_length();
}

double ActiveForceLengthCurve::getMaxActiveFiberLength() const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();  
    return get_max_norm_active_fiber_length();
}

double ActiveForceLengthCurve::getShallowAscendingSlope() const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();  
    return get_shallow_ascending_slope();
}

double ActiveForceLengthCurve::getMinValue() const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();  
    return get_minimum_value();
}



void ActiveForceLengthCurve::
    setMinActiveFiberLength(double minActiveNormFiberLength)
{   
    set_min_norm_active_fiber_length(minActiveNormFiberLength);   
}

void ActiveForceLengthCurve::
    setTransitionFiberLength(double transitionNormFiberLength)
{    
    set_transition_norm_fiber_length(transitionNormFiberLength);
}

void ActiveForceLengthCurve::
    setMaxActiveFiberLength(double maxActiveNormFiberLength)
{
    set_max_norm_active_fiber_length(maxActiveNormFiberLength);
}

void ActiveForceLengthCurve::
    setShallowAscendingSlope(double aSlopeValue)
{
    set_shallow_ascending_slope(aSlopeValue);
}

void ActiveForceLengthCurve::setMinValue(double minimumValue)
{    
    set_minimum_value(minimumValue);    
}


//=============================================================================
// SERVICES
//=============================================================================

double ActiveForceLengthCurve::calcValue(double normFiberLength) const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    return m_curve.calcValue(normFiberLength);
}

double ActiveForceLengthCurve::calcDerivative(double normFiberLength, 
                                                 int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ActiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.calcDerivative(normFiberLength,order);
}

SimTK::Vec2 ActiveForceLengthCurve::getCurveDomain() const
{
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.getCurveDomain();
}

void ActiveForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{   
    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    m_curve.printMuscleCurveToCSVFile(path);
}
