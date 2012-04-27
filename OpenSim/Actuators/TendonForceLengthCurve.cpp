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
#include "TendonForceLengthCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static const char* StrainAtOneNormForceName     = "strain_at_one_norm_force";
static const char* StiffnessAtOneNormForceName  = "stiffness_at_one_norm_force";
static const char* CurvinessName                = "curviness";

//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================

TendonForceLengthCurve::TendonForceLengthCurve():m_curveUpToDate(false)
{
        setNull();
        addProperties();
}

TendonForceLengthCurve::~TendonForceLengthCurve()
{
}

TendonForceLengthCurve::TendonForceLengthCurve(
    const TendonForceLengthCurve& source)                                   
{
        setNull();
        addProperties();
        copyData(source);   
}

TendonForceLengthCurve& TendonForceLengthCurve::
                                operator=(const TendonForceLengthCurve &source)
{
    if(&source != this){
        ModelComponent::operator=(source);
        copyData(source);        
    }
    return(*this);
}

TendonForceLengthCurve::TendonForceLengthCurve( double strainAtOneNormForce, 
                                                double stiffnessAtOneNormForce,
                                                double curviness,
                                                const std::string muscleName)
                                                :m_curveUpToDate(false)
{


    std::string curveName = muscleName;
    curveName.append("_TendonForceLengthCurve");

    setNull();
    addProperties();

   setStrainAtOneNormForce(strainAtOneNormForce);
   setStiffnessAtOneNormForce(stiffnessAtOneNormForce);
   setCurviness(curviness);


    buildCurve();
}


void TendonForceLengthCurve::setNull()
{
    m_curveUpToDate =false;
}

void TendonForceLengthCurve::addProperties()
{   
    setName("default_TendonForceLengthCurve");

    addProperty<double>(StrainAtOneNormForceName, 
        "tendon strain at a tension of 1 normalized force",
        0.04);

    addProperty<double>(StiffnessAtOneNormForceName, 
        "tendon stiffness at a tension of 1 normalized force",
        42);

    addProperty<double>(CurvinessName, 
        "tendon curve bend, from linear to maximum bend (0-1)",
        0.75);
}

void TendonForceLengthCurve::copyData(const TendonForceLengthCurve &source)
{
    if(&source != this){
        setPropertyValue(StrainAtOneNormForceName,
            source.getPropertyValue<double>(StrainAtOneNormForceName));

        setPropertyValue(StiffnessAtOneNormForceName,
            source.getPropertyValue<double>(StiffnessAtOneNormForceName));

        setPropertyValue(CurvinessName,
            source.getPropertyValue<double>(CurvinessName));

        m_curveUpToDate = source.m_curveUpToDate;
        
        m_curve = source.m_curve;
    }
}


void TendonForceLengthCurve::buildCurve()
{
    if(m_curveUpToDate == false){
        
        double e0   =  getStrainAtOneNormForce();
        double kiso =  getStiffnessAtOneNormForce();
        double c    =  getCurviness();        

        //Here's where you call the MuscleCurveFunctionFactory
        MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
                                    createTendonForceLengthCurve(   e0,
                                                                    kiso,
                                                                    c,
                                                                    true,
                                                                    getName());            

        this->m_curve = tmp;
          
    }
    m_curveUpToDate = true;
}


//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void TendonForceLengthCurve::setup(Model& aModel)
{
    ModelComponent::setup(aModel);
}

void TendonForceLengthCurve::initState(SimTK::State& s) const
{
    ModelComponent::initState(s);
}

void TendonForceLengthCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double TendonForceLengthCurve::getStrainAtOneNormForce()
{
    return getPropertyValue<double>(StrainAtOneNormForceName);
}

double TendonForceLengthCurve::getStiffnessAtOneNormForce()
{
    return getPropertyValue<double>(StiffnessAtOneNormForceName);
}

double TendonForceLengthCurve::getCurviness()
{
    return getPropertyValue<double>(CurvinessName);
}


void TendonForceLengthCurve::
    setStrainAtOneNormForce(double aStrainAtOneNormForce)
{
    if(aStrainAtOneNormForce != getStrainAtOneNormForce() )
    {
        setPropertyValue(StrainAtOneNormForceName,
                         aStrainAtOneNormForce);
        m_curveUpToDate = false;
    }
}

void TendonForceLengthCurve::
        setStiffnessAtOneNormForce(double aStiffnessAtOneNormForce)
{
    if(aStiffnessAtOneNormForce != getStiffnessAtOneNormForce() )
    {
        setPropertyValue(StiffnessAtOneNormForceName,
                         aStiffnessAtOneNormForce);
        m_curveUpToDate = false;
    }
}

void TendonForceLengthCurve::setCurviness(double aCurviness)
{
    if(aCurviness != getCurviness() )
    {
        setPropertyValue(CurvinessName,
                         aCurviness);
        m_curveUpToDate = false;
    }
}


//=============================================================================
// SERVICES
//=============================================================================

double TendonForceLengthCurve::
    calcValue(double aStrain) const
{
    if(m_curveUpToDate == false){
        TendonForceLengthCurve* mthis = 
            const_cast<TendonForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(aStrain);
}

double TendonForceLengthCurve::
    calcDerivative(double aStrain, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "TendonForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(m_curveUpToDate == false){
        TendonForceLengthCurve* mthis = 
            const_cast<TendonForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(aStrain,order);
}

SimTK::Vec2 TendonForceLengthCurve::getCurveDomain() const
{
    if(m_curveUpToDate == false){
        TendonForceLengthCurve* mthis = 
            const_cast<TendonForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void TendonForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(m_curveUpToDate == false){
        TendonForceLengthCurve* mthis = 
            const_cast<TendonForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}