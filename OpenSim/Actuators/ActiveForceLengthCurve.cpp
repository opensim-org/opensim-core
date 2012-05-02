/* Author: Matthew Millard
/*
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

    setMinActiveFiberLength(minActiveNormFiberLength);
    setTransitionFiberLength(transitionNormFiberLength);
    setMaxActiveFiberLength(maxActiveNormFiberLength);
    setShallowAscendingSlope(shallowAscendingSlope);
    setMinValue(minValue);

    buildCurve();
}

void ActiveForceLengthCurve::setNull()
{
    m_curveUpToDate = false;
}

void ActiveForceLengthCurve::constructProperties()
{
    constructProperty_min_norm_active_fiber_length(0.4);
    constructProperty_transition_norm_fiber_length(0.75);
    constructProperty_max_norm_active_fiber_length(1.6);
    constructProperty_shallow_ascending_slope(0.75);
    constructProperty_minimum_value(0.05); 
}

void ActiveForceLengthCurve::buildCurve()
{
    if(m_curveUpToDate == false){
        double lce0 = getMinActiveFiberLength();
        double lce1 = getTransitionFiberLength();
        double lce2 = 1.0;
        double lce3 = getMaxActiveFiberLength();

        double dydx = getShallowAscendingSlope();
        double minVal=getMinValue();

        double curviness = 0.75;

        //Here's where you call the MuscleCurveFunctionFactory
        MuscleCurveFunction tmp = 
            MuscleCurveFunctionFactory::
            createFiberActiveForceLengthCurve(  lce0, lce1,
                                                lce2, lce3,
                                              minVal, dydx,
                                           curviness, false, 
                                            getName());
        this->m_curve = tmp;
    }
    m_curveUpToDate = true;
}


//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void ActiveForceLengthCurve::setup(Model& aModel)
{
    Super::setup(aModel);
}

void ActiveForceLengthCurve::initState(SimTK::State& s) const
{
    Super::initState(s);
}

void ActiveForceLengthCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    ActiveForceLengthCurve* mthis = const_cast<ActiveForceLengthCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================


double ActiveForceLengthCurve::getMinActiveFiberLength() const
{
    return getProperty_min_norm_active_fiber_length();
}

double ActiveForceLengthCurve::getTransitionFiberLength() const
{
    return getProperty_transition_norm_fiber_length();
}

double ActiveForceLengthCurve::getMaxActiveFiberLength() const
{
    return getProperty_max_norm_active_fiber_length();
}

double ActiveForceLengthCurve::getShallowAscendingSlope() const
{
    return getProperty_shallow_ascending_slope();
}

double ActiveForceLengthCurve::getMinValue() const
{
    return getProperty_minimum_value();
}



void ActiveForceLengthCurve::
    setMinActiveFiberLength(double minActiveNormFiberLength)
{
    if(minActiveNormFiberLength != getMinActiveFiberLength())
    {
        setProperty_min_norm_active_fiber_length(minActiveNormFiberLength);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::
    setTransitionFiberLength(double transitionNormFiberLength)
{    
    if(transitionNormFiberLength != getTransitionFiberLength())
    {
        setProperty_transition_norm_fiber_length(transitionNormFiberLength);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::
    setMaxActiveFiberLength(double maxActiveNormFiberLength)
{
    if(maxActiveNormFiberLength != getMaxActiveFiberLength()){
        setProperty_max_norm_active_fiber_length(maxActiveNormFiberLength);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::
    setShallowAscendingSlope(double aSlopeValue)
{
    if(aSlopeValue != getShallowAscendingSlope()){
        setProperty_shallow_ascending_slope(aSlopeValue);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::setMinValue(double minimumValue)
{    
    if(minimumValue != getMinValue()){
        setProperty_minimum_value(minimumValue);
        m_curveUpToDate = false;
    }
}


//=============================================================================
// SERVICES
//=============================================================================

double ActiveForceLengthCurve::calcValue(double normFiberLength) const
{
    if(m_curveUpToDate == false){
        ActiveForceLengthCurve* mthis = 
            const_cast<ActiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(normFiberLength);
}

double ActiveForceLengthCurve::calcDerivative(double normFiberLength, 
                                                 int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ActiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(m_curveUpToDate == false){
        ActiveForceLengthCurve* mthis = 
            const_cast<ActiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(normFiberLength,order);
}

SimTK::Vec2 ActiveForceLengthCurve::getCurveDomain() const
{
    if(m_curveUpToDate == false){
        ActiveForceLengthCurve* mthis = 
            const_cast<ActiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void ActiveForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(m_curveUpToDate == false){
        ActiveForceLengthCurve* mthis = 
            const_cast<ActiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}