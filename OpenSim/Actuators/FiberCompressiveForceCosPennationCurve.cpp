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
#include "FiberCompressiveForceCosPennationCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static const double DegreesToRadians = SimTK::Pi/180.0;



//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================

FiberCompressiveForceCosPennationCurve::
        FiberCompressiveForceCosPennationCurve()
{
        setNull();
        constructProperties();
        setName("default_FiberCompressiveForceCosPennationCurve");
}

FiberCompressiveForceCosPennationCurve::
    FiberCompressiveForceCosPennationCurve(double engagementAngleInDegrees, 
                                           double stiffnessAtPerpendicular,
                                           double curviness,
                                           const std::string& muscleName)                                           
{
    setNull();
    constructProperties();
    setName(muscleName + "_FiberCompressiveForceCosPennationCurve");

    setEngagementAngleInDegrees(engagementAngleInDegrees);
    setStiffnessAtPerpendicular(stiffnessAtPerpendicular);
    setCurviness(curviness);

    buildCurve();
}

FiberCompressiveForceCosPennationCurve::
    FiberCompressiveForceCosPennationCurve(double engagementAngleInDegrees,                                           
                                           const std::string& muscleName)                                           
{
    setNull();
    constructProperties();
    setName(muscleName + "_FiberCompressiveForceCosPennationCurve");

    setEngagementAngleInDegrees(engagementAngleInDegrees);    
    buildCurve();
}


void FiberCompressiveForceCosPennationCurve::setNull()
{    
}

void FiberCompressiveForceCosPennationCurve::constructProperties()
{   
    
    constructProperty_engagement_angle_in_degrees(80);
    constructProperty_stiffness_at_perpendicular();
    constructProperty_curviness();

}


void FiberCompressiveForceCosPennationCurve::buildCurve()
{
    if(isObjectUpToDateWithProperties() == false){
          
        double angle =  getEngagementAngleInDegrees();
        double kiso  =  getStiffnessAtPerpendicularInUse();
        double c     =  getCurvinessInUse();        

        double cosAngle = cos(angle*DegreesToRadians);

        //Here's where you call the MuscleCurveFunctionFactory
        MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
            createFiberCompressiveForceCosPennationCurve(   cosAngle,
                                                            kiso,
                                                            c,
                                                            true,
                                                            getName());       
        this->m_curve = tmp;          
    }   
    setObjectIsUpToDateWithProperties();
}


//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void FiberCompressiveForceCosPennationCurve::setup(Model& aModel)
{
    ModelComponent::setup(aModel);
}

void FiberCompressiveForceCosPennationCurve::initState(SimTK::State& s) const
{
    ModelComponent::initState(s);
}

void FiberCompressiveForceCosPennationCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    FiberCompressiveForceCosPennationCurve* mthis = const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->buildCurve();
}

//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberCompressiveForceCosPennationCurve::getEngagementAngleInDegrees()
{
    return getProperty_engagement_angle_in_degrees();
}

double FiberCompressiveForceCosPennationCurve::getStiffnessAtPerpendicular()
{
    return getProperty_stiffness_at_perpendicular();
}

double FiberCompressiveForceCosPennationCurve::getCurviness()
{
    return getProperty_curviness();
}


double FiberCompressiveForceCosPennationCurve::
    getStiffnessAtPerpendicularInUse()
{
    double val = 0;
    if(getProperty_stiffness_at_perpendicular().empty()){
        double eAngleRad = getEngagementAngleInDegrees()*DegreesToRadians;
        val = -2.0/cos(eAngleRad);        
    }else{
        val = getProperty_stiffness_at_perpendicular();
    }
    return val;
}

double FiberCompressiveForceCosPennationCurve::getCurvinessInUse()
{
    double val = 0;
    if(getProperty_curviness().empty()){
        val = 0.1;
    }else{
        val = getProperty_curviness();
    }
    return val;
}

void FiberCompressiveForceCosPennationCurve::
    setEngagementAngleInDegrees(double aEngagementAngleInDegrees)
{
    if(aEngagementAngleInDegrees != getEngagementAngleInDegrees() )
    {
        setProperty_engagement_angle_in_degrees(aEngagementAngleInDegrees);
    }
}

void FiberCompressiveForceCosPennationCurve::
        setStiffnessAtPerpendicular(double aStiffnessAtPerpendicular)
{    
        setProperty_stiffness_at_perpendicular(aStiffnessAtPerpendicular);
}

void FiberCompressiveForceCosPennationCurve::setCurviness(double aCurviness)
{    
        setProperty_curviness(aCurviness);
}


//=============================================================================
// SERVICES
//=============================================================================

double FiberCompressiveForceCosPennationCurve::
    calcValue(double cosPennationAngle) const
{    
    if(isObjectUpToDateWithProperties() == false){
        FiberCompressiveForceCosPennationCurve* mthis = 
            const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(cosPennationAngle);
}

double FiberCompressiveForceCosPennationCurve::
    calcDerivative(double cosPennationAngle, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberCompressiveForceCosPennationCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(isObjectUpToDateWithProperties() == false){
        FiberCompressiveForceCosPennationCurve* mthis = 
            const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(cosPennationAngle,order);
}

SimTK::Vec2 FiberCompressiveForceCosPennationCurve::getCurveDomain() const
{
    if(isObjectUpToDateWithProperties() == false){
        FiberCompressiveForceCosPennationCurve* mthis = 
            const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void FiberCompressiveForceCosPennationCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(isObjectUpToDateWithProperties() == false){
        FiberCompressiveForceCosPennationCurve* mthis = 
            const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}