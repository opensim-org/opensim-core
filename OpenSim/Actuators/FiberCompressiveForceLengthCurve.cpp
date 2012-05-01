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
#include "FiberCompressiveForceLengthCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static const char* NormLengthAtZeroForceName     = "norm_length_at_zero_force";
static const char* StiffnessAtZeroLengthName     = "stiffness_at_zero_length";
static const char* CurvinessName             = "curviness";

//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================

FiberCompressiveForceLengthCurve::
        FiberCompressiveForceLengthCurve():m_curveUpToDate(false)
{
        setNull();
        addProperties();
}

FiberCompressiveForceLengthCurve::~FiberCompressiveForceLengthCurve()
{
}

FiberCompressiveForceLengthCurve::FiberCompressiveForceLengthCurve(
    const FiberCompressiveForceLengthCurve& source)                                   
{
        setNull();
        addProperties();
        copyData(source);   
}

FiberCompressiveForceLengthCurve& FiberCompressiveForceLengthCurve::
                      operator=(const FiberCompressiveForceLengthCurve &source)
{
    if(&source != this){
        ModelComponent::operator=(source);
        copyData(source);        
    }
    return(*this);
}

FiberCompressiveForceLengthCurve::
    FiberCompressiveForceLengthCurve(   double normLengthAtZeroForce, 
                                        double stiffnessAtZeroLength,
                                        double curviness,
                                        const std::string muscleName)
                                        :m_curveUpToDate(false)
{


    std::string curveName = muscleName;
    curveName.append("_FiberCompressiveForceLengthCurve");

    setNull();
    addProperties();

   setNormLengthAtZeroForce(normLengthAtZeroForce);
   setStiffnessAtZeroLength(stiffnessAtZeroLength);
   setCurviness(curviness);


    buildCurve();
}


void FiberCompressiveForceLengthCurve::setNull()
{
    m_curveUpToDate =false;
}

void FiberCompressiveForceLengthCurve::addProperties()
{   
    setName("default_FiberCompressiveForceLengthCurve");

    addProperty<double>(NormLengthAtZeroForceName, 
        "Normalized fiber length at zero force ",
        0.58564173314080115);

    addProperty<double>(StiffnessAtZeroLengthName, 
        "Fiber stiffness at zero length",
        -8.0);

    addProperty<double>(CurvinessName, 
        "Fiber curve bend, from linear to maximum bend (0-1)",
        0.1);
}

void FiberCompressiveForceLengthCurve::
    copyData(const FiberCompressiveForceLengthCurve &source)
{
    if(&source != this){
        setPropertyValue(NormLengthAtZeroForceName,
            source.getPropertyValue<double>(NormLengthAtZeroForceName));

        setPropertyValue(StiffnessAtZeroLengthName,
            source.getPropertyValue<double>(StiffnessAtZeroLengthName));

        setPropertyValue(CurvinessName,
            source.getPropertyValue<double>(CurvinessName));

        m_curveUpToDate = source.m_curveUpToDate;
        
        m_curve = source.m_curve;
    }
}


void FiberCompressiveForceLengthCurve::buildCurve()
{
    if(m_curveUpToDate == false){
        
        double l0   =  getNormLengthAtZeroForce();
        double kiso =  getStiffnessAtZeroLength();
        double c    =  getCurviness();        

        //Here's where you call the MuscleCurveFunctionFactory
        MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
            createFiberCompressiveForceLengthCurve( l0,
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
void FiberCompressiveForceLengthCurve::setup(Model& aModel)
{
    ModelComponent::setup(aModel);
}

void FiberCompressiveForceLengthCurve::initState(SimTK::State& s) const
{
    ModelComponent::initState(s);
}

void FiberCompressiveForceLengthCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    FiberCompressiveForceLengthCurve* mthis = const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberCompressiveForceLengthCurve::getNormLengthAtZeroForce()
{
    return getPropertyValue<double>(NormLengthAtZeroForceName);
}

double FiberCompressiveForceLengthCurve::getStiffnessAtZeroLength()
{
    return getPropertyValue<double>(StiffnessAtZeroLengthName);
}

double FiberCompressiveForceLengthCurve::getCurviness()
{
    return getPropertyValue<double>(CurvinessName);
}


void FiberCompressiveForceLengthCurve::
    setNormLengthAtZeroForce(double aNormLengthAtZeroForce)
{
    if(aNormLengthAtZeroForce != getNormLengthAtZeroForce() )
    {
        setPropertyValue(NormLengthAtZeroForceName,
                         aNormLengthAtZeroForce);
        m_curveUpToDate = false;
    }
}

void FiberCompressiveForceLengthCurve::
        setStiffnessAtZeroLength(double aStiffnessAtZeroLength)
{
    if(aStiffnessAtZeroLength != getStiffnessAtZeroLength() )
    {
        setPropertyValue(StiffnessAtZeroLengthName,
                         aStiffnessAtZeroLength);
        m_curveUpToDate = false;
    }
}

void FiberCompressiveForceLengthCurve::setCurviness(double aCurviness)
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

double FiberCompressiveForceLengthCurve::
    calcValue(double aNormLength) const
{
    if(m_curveUpToDate == false){
        FiberCompressiveForceLengthCurve* mthis = 
            const_cast<FiberCompressiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(aNormLength);
}

double FiberCompressiveForceLengthCurve::
    calcDerivative(double aNormLength, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberCompressiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(m_curveUpToDate == false){
        FiberCompressiveForceLengthCurve* mthis = 
            const_cast<FiberCompressiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(aNormLength,order);
}

SimTK::Vec2 FiberCompressiveForceLengthCurve::getCurveDomain() const
{
    if(m_curveUpToDate == false){
        FiberCompressiveForceLengthCurve* mthis = 
            const_cast<FiberCompressiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void FiberCompressiveForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(m_curveUpToDate == false){
        FiberCompressiveForceLengthCurve* mthis = 
            const_cast<FiberCompressiveForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}