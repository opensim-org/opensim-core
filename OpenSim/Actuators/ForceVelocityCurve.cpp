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

static const char* ConcentricMinSlopeName   ="min_concentric_slope";
static const char* IsometricMaxSlopeName    ="isometric_slope";
static const char* EccentricMinSlopeName    ="max_eccentric_slope";

static const char* MaxEccentricVelocityForceMultiplierName 
                                    = "max_eccentric_velocity_force_multiplier";

static const char* ConcentricCurvinessName  ="concentric_curviness";
static const char* EccentricCurvinessName   ="eccentric_curviness";

//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================

ForceVelocityCurve::ForceVelocityCurve():m_curveUpToDate(false)
{
        setNull();
        addProperties();
}

ForceVelocityCurve::~ForceVelocityCurve()
{
}

ForceVelocityCurve::ForceVelocityCurve(
    const ForceVelocityCurve& source)                                   
{
        setNull();
        addProperties();
        copyData(source);   
}

ForceVelocityCurve& ForceVelocityCurve::
                                operator=(const ForceVelocityCurve &source)
{
    if(&source != this){
        ModelComponent::operator=(source);
        copyData(source);        
    }
    return(*this);
}

ForceVelocityCurve::ForceVelocityCurve(double concentricMinSlope, 
                                       double isometricMaxSlope,        
                                       double eccentricMinSlope,
                                     double maxEccentricVelocityForceMultiplier,
                                       double concentricCurviness,
                                       double eccentricCurviness,
                                      const std::string muscleName)
                                      :m_curveUpToDate(false)
{


    std::string curveName = muscleName;
    curveName.append("_ForceVelocityCurve");

    setNull();
    addProperties();

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
    m_curveUpToDate =false;
}

void ForceVelocityCurve::addProperties()
{

    setName("default_ForceVelocityCurve");

    addProperty<double>(ConcentricMinSlopeName, 
        "curve slope at the maximum normalized "
        "concentric contraction velocity (-1)",0.1);

    addProperty<double>(IsometricMaxSlopeName, 
        "curve slope at isometric (normalized fiber velocity of 0)",5);

    addProperty<double>(EccentricMinSlopeName, 
        "curve slope at the maximum normalized "
        "eccentric contraction velocity (1)",0.1);

    addProperty<double>(MaxEccentricVelocityForceMultiplierName, 
        "curve value at the maximum normalized "
        "eccentric contraction velocity",1.8);

    addProperty<double>(ConcentricCurvinessName, 
        "concentric curve bend, from "
        "linear to maximum bend  (0-1)",0.1);
    
    addProperty<double>(EccentricCurvinessName, 
        "eccentric curve bend, from "
        "linear to maximum bend  (0-1)",0.75);
}

void ForceVelocityCurve::copyData(const ForceVelocityCurve &source)
{

    if(&source != this){
        setPropertyValue(ConcentricMinSlopeName,
            source.getPropertyValue<double>(ConcentricMinSlopeName));

        setPropertyValue(IsometricMaxSlopeName,
            source.getPropertyValue<double>(IsometricMaxSlopeName));

        setPropertyValue(EccentricMinSlopeName,
            source.getPropertyValue<double>(EccentricMinSlopeName));

        setPropertyValue(MaxEccentricVelocityForceMultiplierName,
          source.getPropertyValue<double>(
            MaxEccentricVelocityForceMultiplierName));

        setPropertyValue(ConcentricCurvinessName,
            source.getPropertyValue<double>(ConcentricCurvinessName));

        setPropertyValue(EccentricCurvinessName,
            source.getPropertyValue<double>(EccentricCurvinessName));

        m_curveUpToDate = source.m_curveUpToDate;
        
        m_curve = source.m_curve;
    }
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
    ModelComponent::setup(aModel);
}

void ForceVelocityCurve::initState(SimTK::State& s) const
{
    ModelComponent::initState(s);
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
    return getPropertyValue<double>(ConcentricMinSlopeName);
}

double ForceVelocityCurve::getIsometricMaxSlope()
{
    return getPropertyValue<double>(IsometricMaxSlopeName);
}

double ForceVelocityCurve::getEccentricMinSlope()
{
    return getPropertyValue<double>(EccentricMinSlopeName);
}

double ForceVelocityCurve::getMaxEccentricVelocityForceMultiplier()
{
    return getPropertyValue<double>(MaxEccentricVelocityForceMultiplierName);
}
    
double ForceVelocityCurve::getConcentricCurviness()
{
    return getPropertyValue<double>(ConcentricCurvinessName);
}
    
double ForceVelocityCurve::getEccentricCurviness()
{
    return getPropertyValue<double>(EccentricCurvinessName);
}


/*
ConcentricMinSlopeName 
IsometricMaxSlopeName  
EccentricMinSlopeName  

MaxEccentricVelocityForceMultiplierName
                 
ConcentricCurvinessName 
EccentricCurvinessName  
*/

void ForceVelocityCurve::setConcentricMinSlope(double aConcentricMinSlope)
{
    if(aConcentricMinSlope != getConcentricMinSlope() )
    {
        setPropertyValue(ConcentricMinSlopeName,
                         aConcentricMinSlope);
        m_curveUpToDate = false;
    }

}

void ForceVelocityCurve::setIsometricMaxSlope(double aIsometricMaxSlope)
{
    if(aIsometricMaxSlope != getIsometricMaxSlope() )
    {
        setPropertyValue(IsometricMaxSlopeName,
                         aIsometricMaxSlope);
        m_curveUpToDate = false;
    }
}

void ForceVelocityCurve::setEccentricMinSlope(double aEccentricMinSlope)
{
    if(aEccentricMinSlope != getEccentricMinSlope() )
    {
        setPropertyValue(EccentricMinSlopeName,
                         aEccentricMinSlope);
        m_curveUpToDate = false;
    }
}


void ForceVelocityCurve::
    setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier)
{
    if(aMaxForceMultiplier != getMaxEccentricVelocityForceMultiplier() )
    {
        setPropertyValue(MaxEccentricVelocityForceMultiplierName,
                         aMaxForceMultiplier);
        m_curveUpToDate = false;
    }
}


void ForceVelocityCurve::setConcentricCurviness(double aConcentricCurviness)
{
    if(aConcentricCurviness != getConcentricCurviness() )
    {
        setPropertyValue(ConcentricCurvinessName,
                         aConcentricCurviness);
        m_curveUpToDate = false;
    }
}


void ForceVelocityCurve::setEccentricCurviness(double aEccentricCurviness)
{
    if(aEccentricCurviness != getEccentricCurviness() )
    {
        setPropertyValue(EccentricCurvinessName,
                         aEccentricCurviness);
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