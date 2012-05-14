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

    setProperty_min_concentric_slope(concentricMinSlope);
    setProperty_isometric_slope(isometricMaxSlope);
    setProperty_min_eccentric_slope(eccentricMinSlope);

    setProperty_max_eccentric_velocity_force_multiplier(
                       maxEccentricVelocityForceMultiplier);

    setProperty_concentric_curviness(concentricCurviness);
    setProperty_eccentric_curviness(eccentricCurviness);


    ensureCurveUpToDate();
}


void ForceVelocityCurve::setNull()
{
   
}

void ForceVelocityCurve::constructProperties()
{
    constructProperty_min_concentric_slope(0.0);
    constructProperty_isometric_slope(5);
    constructProperty_min_eccentric_slope(0.0);
    constructProperty_max_eccentric_velocity_force_multiplier(1.8);
    constructProperty_concentric_curviness(0.4);
    constructProperty_eccentric_curviness(0.9);
}


void ForceVelocityCurve::buildCurve()
{
        
        double dydxC =  getProperty_min_concentric_slope();;
        double dydxIso= getProperty_isometric_slope();
        double dydxE =  getProperty_min_eccentric_slope();
        double fmax  =  getProperty_max_eccentric_velocity_force_multiplier();
        double ccurv =  getProperty_concentric_curviness();
        double ecurv =  getProperty_eccentric_curviness();
                                
        //Here's where you call the SmoothSegmentedFunctionFactory
        SmoothSegmentedFunction tmp = 
            SmoothSegmentedFunctionFactory::
            createFiberForceVelocityCurve(  fmax,
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

void ForceVelocityCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties() == false){
        buildCurve();
    }
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
    mthis->ensureCurveUpToDate();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double ForceVelocityCurve::getConcentricMinSlope() const
{
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_min_concentric_slope();
}

double ForceVelocityCurve::getIsometricMaxSlope() const
{
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_isometric_slope();
}

double ForceVelocityCurve::getEccentricMinSlope() const
{
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_min_eccentric_slope();
}

double ForceVelocityCurve::getMaxEccentricVelocityForceMultiplier() const
{
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_max_eccentric_velocity_force_multiplier();
}
    
double ForceVelocityCurve::getConcentricCurviness() const
{
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_concentric_curviness();
}
    
double ForceVelocityCurve::getEccentricCurviness() const
{
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return getProperty_eccentric_curviness();
}

void ForceVelocityCurve::setConcentricMinSlope(double aConcentricMinSlope)
{   
    setProperty_min_concentric_slope(aConcentricMinSlope);   
}

void ForceVelocityCurve::setIsometricMaxSlope(double aIsometricMaxSlope)
{
    setProperty_isometric_slope(aIsometricMaxSlope);
}

void ForceVelocityCurve::setEccentricMinSlope(double aEccentricMinSlope)
{
    setProperty_min_eccentric_slope(aEccentricMinSlope);
}


void ForceVelocityCurve::
    setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier)
{
    setProperty_max_eccentric_velocity_force_multiplier(aMaxForceMultiplier);
}


void ForceVelocityCurve::setConcentricCurviness(double aConcentricCurviness)
{
    setProperty_concentric_curviness(aConcentricCurviness);
}


void ForceVelocityCurve::setEccentricCurviness(double aEccentricCurviness)
{
    setProperty_eccentric_curviness(aEccentricCurviness);
}


//=============================================================================
// SERVICES
//=============================================================================

double ForceVelocityCurve::calcValue(double normFiberVelocity) const
{   
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate(); 

    return m_curve.calcValue(normFiberVelocity);
}

double ForceVelocityCurve::calcDerivative(double normFiberVelocity, 
                                                 int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "ForceVelocityCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
  
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();        

    return m_curve.calcDerivative(normFiberVelocity,order);
}

SimTK::Vec2 ForceVelocityCurve::getCurveDomain() const
{
    
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.getCurveDomain();
}

void ForceVelocityCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    
    ForceVelocityCurve* mthis = const_cast<ForceVelocityCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    m_curve.printMuscleCurveToCSVFile(path);
}