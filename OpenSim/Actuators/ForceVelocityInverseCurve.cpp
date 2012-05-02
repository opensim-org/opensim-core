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
#include "ForceVelocityInverseCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// CONSTRUCTION
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

ForceVelocityInverseCurve::ForceVelocityInverseCurve()
{
    setNull();
    constructProperties();
    setName("default_ForceVelocityInverseCurve");
}

ForceVelocityInverseCurve::ForceVelocityInverseCurve
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
    setName(muscleName + "_ForceVelocityInverseCurve");

    setConcentricMinSlope(concentricMinSlope);
    setIsometricMaxSlope(isometricMaxSlope);
    setEccentricMinSlope(eccentricMinSlope);
    setMaxEccentricVelocityForceMultiplier(maxEccentricVelocityForceMultiplier);
    setConcentricCurviness(concentricCurviness);
    setEccentricCurviness(eccentricCurviness);
    
    buildCurve();
}


void ForceVelocityInverseCurve::setNull()
{
    m_curveUpToDate =false;
}

void ForceVelocityInverseCurve::constructProperties()
{
    constructProperty_min_concentric_slope(0.1);
    constructProperty_isometric_slope(5);
    constructProperty_min_eccentric_slope(0.1);
    constructProperty_max_eccentric_velocity_force_multiplier(1.8);
    constructProperty_concentric_curviness(0.1);
    constructProperty_eccentric_curviness(0.75);
}

void ForceVelocityInverseCurve::buildCurve()
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
            createFiberForceVelocityInverseCurve(  fmax,
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
void ForceVelocityInverseCurve::setup(Model& aModel)
{
    ModelComponent::setup(aModel);
}

void ForceVelocityInverseCurve::initState(SimTK::State& s) const
{
    ModelComponent::initState(s);
}

void ForceVelocityInverseCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    ForceVelocityInverseCurve* mthis = const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double ForceVelocityInverseCurve::getConcentricMinSlope()
{
    return getProperty_min_concentric_slope();
}

double ForceVelocityInverseCurve::getIsometricMaxSlope()
{
    return getProperty_isometric_slope();
}

double ForceVelocityInverseCurve::getEccentricMinSlope()
{
    return getProperty_min_eccentric_slope();
}

double ForceVelocityInverseCurve::getMaxEccentricVelocityForceMultiplier()
{
    return getProperty_max_eccentric_velocity_force_multiplier();
}
    
double ForceVelocityInverseCurve::getConcentricCurviness()
{
    return getProperty_concentric_curviness();
}
    
double ForceVelocityInverseCurve::getEccentricCurviness()
{
    return getProperty_eccentric_curviness();
}


void ForceVelocityInverseCurve::setConcentricMinSlope(double aConcentricMinSlope)
{
    if(aConcentricMinSlope != getConcentricMinSlope() )
    {
        setProperty_min_concentric_slope(aConcentricMinSlope);
        m_curveUpToDate = false;
    }

}

void ForceVelocityInverseCurve::setIsometricMaxSlope(double aIsometricMaxSlope)
{
    if(aIsometricMaxSlope != getIsometricMaxSlope() )
    {
        setProperty_isometric_slope(aIsometricMaxSlope);
        m_curveUpToDate = false;
    }
}

void ForceVelocityInverseCurve::setEccentricMinSlope(double aEccentricMinSlope)
{
    if(aEccentricMinSlope != getEccentricMinSlope() )
    {
        setProperty_min_eccentric_slope(aEccentricMinSlope);
        m_curveUpToDate = false;
    }
}


void ForceVelocityInverseCurve::
    setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier)
{
    if(aMaxForceMultiplier != getMaxEccentricVelocityForceMultiplier() )
    {
        setProperty_max_eccentric_velocity_force_multiplier
            (aMaxForceMultiplier);
        m_curveUpToDate = false;
    }
}


void ForceVelocityInverseCurve::setConcentricCurviness(double aConcentricCurviness)
{
    if(aConcentricCurviness != getConcentricCurviness() )
    {
        setProperty_concentric_curviness(aConcentricCurviness);
        m_curveUpToDate = false;
    }
}


void ForceVelocityInverseCurve::setEccentricCurviness(double aEccentricCurviness)
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

double ForceVelocityInverseCurve::
    calcValue(double aForceVelocityMultiplier) const
{
    if(m_curveUpToDate == false){
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(aForceVelocityMultiplier);
}

double ForceVelocityInverseCurve::
    calcDerivative(double aForceVelocityMultiplier, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ForceVelocityInverseCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(m_curveUpToDate == false){
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(aForceVelocityMultiplier,order);
}

SimTK::Vec2 ForceVelocityInverseCurve::getCurveDomain() const
{
    if(m_curveUpToDate == false){
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void ForceVelocityInverseCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(m_curveUpToDate == false){
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}