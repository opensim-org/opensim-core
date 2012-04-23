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

static const char* MinActiveNormFiberLengthName ="min_norm_active_fiber_length";
static const char* TransitionNormFiberLengthName="transition_norm_fiber_length";
static const char* MaxActiveNormFiberLengthName ="max_norm_active_fiber_length";
static const char* ShallowAscendingSlopeName    ="shallow_ascending_slope";
static const char* MinimumValueName             ="minimum_value";

//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================

ActiveForceLengthCurve::ActiveForceLengthCurve():m_curveUpToDate(false)
{
        setNull();
        addProperties();
}

ActiveForceLengthCurve::~ActiveForceLengthCurve()
{
}

ActiveForceLengthCurve::ActiveForceLengthCurve(
    const ActiveForceLengthCurve& source)                                   
{
        setNull();
        addProperties();
        copyData(source);   
}

ActiveForceLengthCurve& ActiveForceLengthCurve::
                                operator=(const ActiveForceLengthCurve &source)
{
    if(&source != this){
        ModelComponent::operator=(source);
        copyData(source);        
    }
    return(*this);
}

ActiveForceLengthCurve::ActiveForceLengthCurve(double minActiveNormFiberLength, 
                                               double transitionNormFiberLength,
                                               double maxActiveNormFiberLength,
                                               double shallowAscendingSlope,
                                               double minValue,
                                               const std::string muscleName)
                                               :m_curveUpToDate(false)
{
    std::string curveName = muscleName;
    curveName.append("_ActiveForceLengthCurve");

    setNull();
    addProperties();

    setMinActiveFiberLength(minActiveNormFiberLength);
    setTransitionFiberLength(transitionNormFiberLength);
    setMaxActiveFiberLength(maxActiveNormFiberLength);
    setShallowAscendingSlope(shallowAscendingSlope);
    setMinValue(minValue);

    buildCurve();
}


void ActiveForceLengthCurve::setNull()
{
    m_curveUpToDate =false;
}

void ActiveForceLengthCurve::addProperties()
{

    setName("default_ActiveForceLengthCurve");

    addProperty<double>(MinActiveNormFiberLengthName, 
        "normalized fiber length which the steep ascending limb starts",0.4);

    addProperty<double>(TransitionNormFiberLengthName, 
        "normalized fiber length which the steep ascending limb transitions"
        " to the shallow ascending limb",0.75);

    addProperty<double>(MaxActiveNormFiberLengthName, 
        "normalized fiber length which the descending limb ends",1.6);

    addProperty<double>(ShallowAscendingSlopeName, 
        "slope of the shallow ascending limb",0.75);

    addProperty<double>(MinimumValueName, 
        "minimum value of the active force length curve",0.05);
    
}

void ActiveForceLengthCurve::copyData(const ActiveForceLengthCurve &source)
{
    if(&source != this){
        setPropertyValue(MinActiveNormFiberLengthName,
            source.getPropertyValue<double>(MinActiveNormFiberLengthName));

        setPropertyValue(TransitionNormFiberLengthName,
            source.getPropertyValue<double>(TransitionNormFiberLengthName));

        setPropertyValue(MaxActiveNormFiberLengthName,
            source.getPropertyValue<double>(MaxActiveNormFiberLengthName));

        setPropertyValue(ShallowAscendingSlopeName,
            source.getPropertyValue<double>(ShallowAscendingSlopeName));

        setPropertyValue(MinimumValueName,
            source.getPropertyValue<double>(MinimumValueName));

        m_curveUpToDate = source.m_curveUpToDate;
        
        m_curve = source.m_curve;
    }
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
    ModelComponent::setup(aModel);
}

void ActiveForceLengthCurve::initState(SimTK::State& s) const
{
    ModelComponent::initState(s);
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
    return getPropertyValue<double>(MinActiveNormFiberLengthName);
}

double ActiveForceLengthCurve::getTransitionFiberLength() const
{
    return getPropertyValue<double>(TransitionNormFiberLengthName);
}

double ActiveForceLengthCurve::getMaxActiveFiberLength() const
{
    return getPropertyValue<double>(MaxActiveNormFiberLengthName);
}

double ActiveForceLengthCurve::getShallowAscendingSlope() const
{
    return getPropertyValue<double>(ShallowAscendingSlopeName);
}

double ActiveForceLengthCurve::getMinValue() const
{
    return getPropertyValue<double>(MinimumValueName);
}



void ActiveForceLengthCurve::
    setMinActiveFiberLength(double minActiveNormFiberLength)
{
    if(minActiveNormFiberLength != getMinActiveFiberLength())
    {
        setPropertyValue(MinActiveNormFiberLengthName,
                         minActiveNormFiberLength);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::
    setTransitionFiberLength(double transitionNormFiberLength)
{    
    if(transitionNormFiberLength != getTransitionFiberLength())
    {
        setPropertyValue(TransitionNormFiberLengthName,
                          transitionNormFiberLength);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::
    setMaxActiveFiberLength(double maxActiveNormFiberLength)
{
    if(maxActiveNormFiberLength != getMaxActiveFiberLength()){
        setPropertyValue(MaxActiveNormFiberLengthName,maxActiveNormFiberLength);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::
    setShallowAscendingSlope(double aSlopeValue)
{
    if(aSlopeValue != getShallowAscendingSlope()){
        setPropertyValue(ShallowAscendingSlopeName, aSlopeValue);
        m_curveUpToDate = false;
    }
}

void ActiveForceLengthCurve::setMinValue(double minimumValue)
{    
    if(minimumValue != getMinValue()){
        setPropertyValue(MinimumValueName, minimumValue);
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