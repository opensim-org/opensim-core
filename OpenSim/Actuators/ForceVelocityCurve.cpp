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
#include "ForceVelocityCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// CONSTRUCTION
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

ForceVelocityCurve::ForceVelocityCurve()
{
    setNull();
    constructProperties();
    setName("default_ForceVelocityCurve");
}

ForceVelocityCurve::ForceVelocityCurve
   (double concentricMinSlope, 
    double isometricMaxSlope,        
    double eccentricMinSlope,
    double maxEccentricVelocityForceMultiplier,
    double concentricCurviness,
    double eccentricCurviness,
    const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_ForceVelocityCurve");

    setConcentricMinSlope(concentricMinSlope);
    setIsometricMaxSlope(isometricMaxSlope);
    setEccentricMinSlope(eccentricMinSlope);
    setMaxEccentricVelocityForceMultiplier(maxEccentricVelocityForceMultiplier);
    setConcentricCurviness(concentricCurviness);
    setEccentricCurviness(eccentricCurviness);
    
    buildCurve();
}


void ForceVelocityCurve::setNull()
{
    m_curveUpToDate = false;
}

void ForceVelocityCurve::constructProperties()
{
    constructProperty_min_concentric_slope(0.1);
    constructProperty_isometric_slope(5);
    constructProperty_min_eccentric_slope(0.1);
    constructProperty_max_eccentric_velocity_force_multiplier(1.8);
    constructProperty_concentric_curviness(0.1);
    constructProperty_eccentric_curviness(0.75);
}

void ForceVelocityCurve::buildCurve()
{
    if(m_curveUpToDate == false){
        
        double dydxC =  getConcentricMinSlope();
        double dydxIso= getIsometricMaxSlope();
        double dydxE =  getEccentricMinSlope();
        double fmax  =  getMaxEccentricVelocityForceMultiplier();
        double ccurv =  getConcentricCurviness();
        double ecurv =  getEccentricCurviness();

        //Here's where you call the MuscleCurveFunctionFactory
        MuscleCurveFunction tmp = 
            MuscleCurveFunctionFactory::
            createFiberForceVelocityCurve(  fmax,
                                            dydxC,
                                            dydxIso,
                                            dydxE,
                                            ccurv,
                                            ecurv,
                                            false,
                                            getName());
        this->m_curve = tmp;
          
    }
    m_curveUpToDate = true;
}


//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void ForceVelocityCurve::setup(Model& aModel)
{
    Super::setup(aModel);
}

void ForceVelocityCurve::initState(SimTK::State& s) const
{
    Super::initState(s);
}

void ForceVelocityCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double ForceVelocityCurve::getConcentricMinSlope()
{
    return getProperty_min_concentric_slope();
}

double ForceVelocityCurve::getIsometricMaxSlope()
{
    return getProperty_isometric_slope();
}

double ForceVelocityCurve::getEccentricMinSlope()
{
    return getProperty_min_eccentric_slope();
}

double ForceVelocityCurve::getMaxEccentricVelocityForceMultiplier()
{
    return getProperty_max_eccentric_velocity_force_multiplier();
}
    
double ForceVelocityCurve::getConcentricCurviness()
{
    return getProperty_concentric_curviness();
}
    
double ForceVelocityCurve::getEccentricCurviness()
{
    return getProperty_eccentric_curviness();
}

void ForceVelocityCurve::setConcentricMinSlope(double aConcentricMinSlope)
{
    if(aConcentricMinSlope != getConcentricMinSlope() )
    {
        setProperty_min_concentric_slope(aConcentricMinSlope);
        m_curveUpToDate = false;
    }

}

void ForceVelocityCurve::setIsometricMaxSlope(double aIsometricMaxSlope)
{
    if(aIsometricMaxSlope != getIsometricMaxSlope() )
    {
        setProperty_isometric_slope(aIsometricMaxSlope);
        m_curveUpToDate = false;
    }
}

void ForceVelocityCurve::setEccentricMinSlope(double aEccentricMinSlope)
{
    if(aEccentricMinSlope != getEccentricMinSlope() )
    {
        setProperty_min_eccentric_slope(aEccentricMinSlope);
        m_curveUpToDate = false;
    }
}


void ForceVelocityCurve::
    setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier)
{
    if(aMaxForceMultiplier != getMaxEccentricVelocityForceMultiplier() )
    {
        setProperty_max_eccentric_velocity_force_multiplier
            (aMaxForceMultiplier);
        m_curveUpToDate = false;
    }
}


void ForceVelocityCurve::setConcentricCurviness(double aConcentricCurviness)
{
    if(aConcentricCurviness != getConcentricCurviness() )
    {
        setProperty_concentric_curviness(aConcentricCurviness);
        m_curveUpToDate = false;
    }
}


void ForceVelocityCurve::setEccentricCurviness(double aEccentricCurviness)
{
    if(aEccentricCurviness != getEccentricCurviness() )
    {
        setProperty_eccentric_curviness(aEccentricCurviness);
        m_curveUpToDate = false;
    }
}


//=============================================================================
// SERVICES
//=============================================================================

double ForceVelocityCurve::calcValue(double normFiberVelocity) const
{
    if(m_curveUpToDate == false){
        ForceVelocityCurve* mthis = 
            const_cast<ForceVelocityCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(normFiberVelocity);
}

double ForceVelocityCurve::calcDerivative(double normFiberVelocity, 
                                                 int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ForceVelocityCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(m_curveUpToDate == false){
        ForceVelocityCurve* mthis = 
            const_cast<ForceVelocityCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(normFiberVelocity,order);
}

SimTK::Vec2 ForceVelocityCurve::getCurveDomain() const
{
    if(m_curveUpToDate == false){
        ForceVelocityCurve* mthis = 
            const_cast<ForceVelocityCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void ForceVelocityCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(m_curveUpToDate == false){
        ForceVelocityCurve* mthis = 
            const_cast<ForceVelocityCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}