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

ForceVelocityInverseCurve::ForceVelocityInverseCurve():m_curveUpToDate(false)
{
        setNull();
        addProperties();
}

ForceVelocityInverseCurve::~ForceVelocityInverseCurve()
{
}

ForceVelocityInverseCurve::ForceVelocityInverseCurve(
    const ForceVelocityInverseCurve& source)                                   
{
        setNull();
        addProperties();
        copyData(source);   
}

ForceVelocityInverseCurve& ForceVelocityInverseCurve::
                                operator=(const ForceVelocityInverseCurve &source)
{
    if(&source != this){
        ModelComponent::operator=(source);
        copyData(source);        
    }
    return(*this);
}

ForceVelocityInverseCurve::ForceVelocityInverseCurve(double concentricMinSlope, 
                                       double isometricMaxSlope,        
                                       double eccentricMinSlope,
                                     double maxEccentricVelocityForceMultiplier,
                                       double concentricCurviness,
                                       double eccentricCurviness,
                                      const std::string muscleName)
                                      :m_curveUpToDate(false)
{


    std::string curveName = muscleName;
    curveName.append("_ForceVelocityInverseCurve");

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


void ForceVelocityInverseCurve::setNull()
{
    m_curveUpToDate =false;
}

void ForceVelocityInverseCurve::addProperties()
{

    setName("default_ForceVelocityInverseCurve");

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

void ForceVelocityInverseCurve::copyData(const ForceVelocityInverseCurve &source)
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
    return getPropertyValue<double>(ConcentricMinSlopeName);
}

double ForceVelocityInverseCurve::getIsometricMaxSlope()
{
    return getPropertyValue<double>(IsometricMaxSlopeName);
}

double ForceVelocityInverseCurve::getEccentricMinSlope()
{
    return getPropertyValue<double>(EccentricMinSlopeName);
}

double ForceVelocityInverseCurve::getMaxEccentricVelocityForceMultiplier()
{
    return getPropertyValue<double>(MaxEccentricVelocityForceMultiplierName);
}
    
double ForceVelocityInverseCurve::getConcentricCurviness()
{
    return getPropertyValue<double>(ConcentricCurvinessName);
}
    
double ForceVelocityInverseCurve::getEccentricCurviness()
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

void ForceVelocityInverseCurve::setConcentricMinSlope(double aConcentricMinSlope)
{
    if(aConcentricMinSlope != getConcentricMinSlope() )
    {
        setPropertyValue(ConcentricMinSlopeName,
                         aConcentricMinSlope);
        m_curveUpToDate = false;
    }

}

void ForceVelocityInverseCurve::setIsometricMaxSlope(double aIsometricMaxSlope)
{
    if(aIsometricMaxSlope != getIsometricMaxSlope() )
    {
        setPropertyValue(IsometricMaxSlopeName,
                         aIsometricMaxSlope);
        m_curveUpToDate = false;
    }
}

void ForceVelocityInverseCurve::setEccentricMinSlope(double aEccentricMinSlope)
{
    if(aEccentricMinSlope != getEccentricMinSlope() )
    {
        setPropertyValue(EccentricMinSlopeName,
                         aEccentricMinSlope);
        m_curveUpToDate = false;
    }
}


void ForceVelocityInverseCurve::
    setMaxEccentricVelocityForceMultiplier(double aMaxForceMultiplier)
{
    if(aMaxForceMultiplier != getMaxEccentricVelocityForceMultiplier() )
    {
        setPropertyValue(MaxEccentricVelocityForceMultiplierName,
                         aMaxForceMultiplier);
        m_curveUpToDate = false;
    }
}


void ForceVelocityInverseCurve::setConcentricCurviness(double aConcentricCurviness)
{
    if(aConcentricCurviness != getConcentricCurviness() )
    {
        setPropertyValue(ConcentricCurvinessName,
                         aConcentricCurviness);
        m_curveUpToDate = false;
    }
}


void ForceVelocityInverseCurve::setEccentricCurviness(double aEccentricCurviness)
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