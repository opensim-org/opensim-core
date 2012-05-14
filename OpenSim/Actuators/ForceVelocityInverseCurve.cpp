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

    
    setProperty_min_concentric_slope(concentricMinSlope);
    setProperty_isometric_slope(isometricMaxSlope);
    setProperty_min_eccentric_slope(eccentricMinSlope);
    setProperty_max_eccentric_velocity_force_multiplier(
        maxEccentricVelocityForceMultiplier);
    setProperty_concentric_curviness(concentricCurviness);
    setProperty_eccentric_curviness(eccentricCurviness);

    ensureCurveUpToDate();
}


void ForceVelocityInverseCurve::setNull()
{
    
}

void ForceVelocityInverseCurve::constructProperties()
{
    constructProperty_min_concentric_slope(0.1);
    constructProperty_isometric_slope(5);
    constructProperty_min_eccentric_slope(0.1);
    constructProperty_max_eccentric_velocity_force_multiplier(1.8);
    constructProperty_concentric_curviness(0.4);
    constructProperty_eccentric_curviness(0.9);
}

void ForceVelocityInverseCurve::buildCurve()
{      
        double dydxC =  getProperty_min_concentric_slope();
        double dydxIso= getProperty_isometric_slope();
        double dydxE =  getProperty_min_eccentric_slope();
        double fmax  =  getProperty_max_eccentric_velocity_force_multiplier();
        double ccurv =  getProperty_concentric_curviness();
        double ecurv =  getProperty_eccentric_curviness();

        //Here's where you call the SmoothSegmentedFunctionFactory
        SmoothSegmentedFunction tmp = 
            SmoothSegmentedFunctionFactory::
            createFiberForceVelocityInverseCurve(  fmax,
                                            dydxC,
                                            dydxIso,
                                            dydxE,
                                            ccurv,
                                            ecurv,
                                            false,
                                            getName());

        this->m_curve = tmp;
        setObjectIsUpToDateWithProperties();      
    
}

void ForceVelocityInverseCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties() == false){
        buildCurve();
    }
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

void ForceVelocityInverseCurve::
    createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    ForceVelocityInverseCurve* mthis 
        = const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double ForceVelocityInverseCurve::getConcentricMinSlope() const
{
    ForceVelocityInverseCurve* mthis = 
        const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return getProperty_min_concentric_slope();
}

double ForceVelocityInverseCurve::getIsometricMaxSlope() const
{
    ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_isometric_slope();
}

double ForceVelocityInverseCurve::getEccentricMinSlope() const
{
    ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_min_eccentric_slope();
}

double ForceVelocityInverseCurve::getMaxEccentricVelocityForceMultiplier() const
{
    ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_max_eccentric_velocity_force_multiplier();
}
    
double ForceVelocityInverseCurve::getConcentricCurviness() const
{
    ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_concentric_curviness();
}
    
double ForceVelocityInverseCurve::getEccentricCurviness() const
{
    ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_eccentric_curviness();
}


void ForceVelocityInverseCurve::setConcentricMinSlope(double aConcentricMinSlope)
{   
    setProperty_min_concentric_slope(aConcentricMinSlope);    
}

void ForceVelocityInverseCurve::setIsometricMaxSlope(double aIsometricMaxSlope)
{
    setProperty_isometric_slope(aIsometricMaxSlope);
}

void ForceVelocityInverseCurve::setEccentricMinSlope(double aEccentricMinSlope)
{
    setProperty_min_eccentric_slope(aEccentricMinSlope);
}


void ForceVelocityInverseCurve::
    setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier)
{
        setProperty_max_eccentric_velocity_force_multiplier
            (aMaxForceMultiplier);
}


void ForceVelocityInverseCurve::setConcentricCurviness(double aConcentricCurviness)
{
        setProperty_concentric_curviness(aConcentricCurviness);
}


void ForceVelocityInverseCurve::setEccentricCurviness(double aEccentricCurviness)
{
        setProperty_eccentric_curviness(aEccentricCurviness);
}


//=============================================================================
// SERVICES
//=============================================================================

double ForceVelocityInverseCurve::
    calcValue(double aForceVelocityMultiplier) const
{
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->ensureCurveUpToDate();    

    return m_curve.calcValue(aForceVelocityMultiplier);
}

double ForceVelocityInverseCurve::
    calcDerivative(double aForceVelocityMultiplier, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ForceVelocityInverseCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->ensureCurveUpToDate();
    
    return m_curve.calcDerivative(aForceVelocityMultiplier,order);
}

SimTK::Vec2 ForceVelocityInverseCurve::getCurveDomain() const
{
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->ensureCurveUpToDate();    

    return m_curve.getCurveDomain();
}

void ForceVelocityInverseCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
        ForceVelocityInverseCurve* mthis = 
            const_cast<ForceVelocityInverseCurve*>(this);    
        mthis->ensureCurveUpToDate();    

    m_curve.printMuscleCurveToCSVFile(path);
}