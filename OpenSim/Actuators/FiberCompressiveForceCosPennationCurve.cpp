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

    setProperty_engagement_angle_in_degrees(engagementAngleInDegrees);
    setProperty_stiffness_at_perpendicular(stiffnessAtPerpendicular);
    setProperty_curviness(curviness);

    ensureCurveUpToDate();
}

FiberCompressiveForceCosPennationCurve::
    FiberCompressiveForceCosPennationCurve(double engagementAngleInDegrees,                                           
                                           const std::string& muscleName)                                           
{
    setNull();
    constructProperties();
    setName(muscleName + "_FiberCompressiveForceCosPennationCurve");

    setProperty_engagement_angle_in_degrees(engagementAngleInDegrees);    
    ensureCurveUpToDate();
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
    double angle =  getProperty_engagement_angle_in_degrees();
    double k     =  m_stiffnessAtPerpendicularInUse;
    double c     =  m_curvinessInUse;        

    double cosAngle = cos(angle*DegreesToRadians);

    //Here's where you call the SmoothSegmentedFunctionFactory
    SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
        createFiberCompressiveForceCosPennationCurve(   cosAngle,
                                                        k,
                                                        c,
                                                        true,
                                                        getName());       
    this->m_curve = tmp;          
       
    setObjectIsUpToDateWithProperties();
}

void FiberCompressiveForceCosPennationCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties() == false){

        //=====================================================================
        //Compute the optional properties if they have not been set
        //=====================================================================
        if(getProperty_stiffness_at_perpendicular().empty() == true &&
            getProperty_curviness().empty() == true)
        {
            double eAngleRad = getProperty_engagement_angle_in_degrees()
                              *DegreesToRadians;

            m_stiffnessAtPerpendicularInUse= -2.0/cos(eAngleRad); 
            m_curvinessInUse = 0.1;
            m_isFittedCurveBeingUsed = true;
        }

        //=====================================================================
        //Use the optional properties if they have been set
        //=====================================================================
        if(getProperty_stiffness_at_perpendicular().empty() == false &&
            getProperty_curviness().empty() == false)
        {
            
            m_stiffnessAtPerpendicularInUse = 
                getProperty_stiffness_at_perpendicular();
            m_curvinessInUse = getProperty_curviness();
            m_isFittedCurveBeingUsed = false;
        }

        //=====================================================================
        //Error condition if only one optional parameter is set.
        //=====================================================================
        bool a = getProperty_stiffness_at_perpendicular().empty();
        bool b = getProperty_curviness().empty();

        //This is really a XOR operation ...
        if( ( a && !b ) || ( !a && b ) ){

            //This error condition is checked to make sure that the reference
            //fiber is being used to populate all optional properties or none
            //of them. Anything different would result in an inconsistency in
            //the computed curve - it wouldn't reflect the reference curve.
            SimTK_ERRCHK1_ALWAYS(false,
                "FiberCompressiveForceCosPennationCurve::ensureCurveUpToDate()",
                "%s: Optional parameters stiffness and curviness must both"
                "be set, or both remain empty. You have set one parameter"
                "and left the other blank.",
                getName().c_str());  
        }

        buildCurve();
    }
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

void FiberCompressiveForceCosPennationCurve::
    createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();
}

//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberCompressiveForceCosPennationCurve::
    getEngagementAngleInDegrees() const
{    
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return getProperty_engagement_angle_in_degrees();
}

double FiberCompressiveForceCosPennationCurve::
    getStiffnessAtPerpendicularInUse() const
{
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_stiffnessAtPerpendicularInUse;
}

double FiberCompressiveForceCosPennationCurve::
    getCurvinessInUse() const
{
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curvinessInUse;
}

void FiberCompressiveForceCosPennationCurve::
    setEngagementAngleInDegrees(double aEngagementAngleInDegrees)
{   
   setProperty_engagement_angle_in_degrees(aEngagementAngleInDegrees);  
}

void FiberCompressiveForceCosPennationCurve::
    setOptionalProperties(double aStiffnessAtPerpendicular, double aCurviness)
{    
    setProperty_stiffness_at_perpendicular(aStiffnessAtPerpendicular);    
    setProperty_curviness(aCurviness);
}

bool FiberCompressiveForceCosPennationCurve::isFittedCurveBeingUsed() const
{
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_isFittedCurveBeingUsed;
}

//=============================================================================
// SERVICES
//=============================================================================

double FiberCompressiveForceCosPennationCurve::
    calcValue(double cosPennationAngle) const
{    
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.calcValue(cosPennationAngle);
}

double FiberCompressiveForceCosPennationCurve::
    calcDerivative(double cosPennationAngle, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberCompressiveForceCosPennationCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
       
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.calcDerivative(cosPennationAngle,order);
}

double FiberCompressiveForceCosPennationCurve::
    calcIntegral(double cosPennationAngle) const
{    
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.calcIntegral(cosPennationAngle);
}

SimTK::Vec2 FiberCompressiveForceCosPennationCurve::getCurveDomain() const
{
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.getCurveDomain();
}

void FiberCompressiveForceCosPennationCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    FiberCompressiveForceCosPennationCurve* mthis = 
        const_cast<FiberCompressiveForceCosPennationCurve*>(this);    
    mthis->ensureCurveUpToDate();    

    m_curve.printMuscleCurveToCSVFile(path);
}