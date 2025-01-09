/* -------------------------------------------------------------------------- *
 *            OpenSim:  FiberCompressiveForceCosPennationCurve.cpp            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "FiberCompressiveForceCosPennationCurve.h"
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================

FiberCompressiveForceCosPennationCurve::
        FiberCompressiveForceCosPennationCurve()
{
        setNull();
        constructProperties();
        setName("default_FiberCompressiveForceCosPennationCurve");
        ensureCurveUpToDate();
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

    set_engagement_angle_in_degrees(engagementAngleInDegrees);
    set_stiffness_at_perpendicular(stiffnessAtPerpendicular);
    set_curviness(curviness);

    ensureCurveUpToDate();
}

FiberCompressiveForceCosPennationCurve::
    FiberCompressiveForceCosPennationCurve(double engagementAngleInDegrees,                                           
                                           const std::string& muscleName)                                           
{
    setNull();
    constructProperties();
    setName(muscleName + "_FiberCompressiveForceCosPennationCurve");

    set_engagement_angle_in_degrees(engagementAngleInDegrees);    
    ensureCurveUpToDate();
}


void FiberCompressiveForceCosPennationCurve::setNull()
{    

    setAuthors("Matthew Millard");
}

void FiberCompressiveForceCosPennationCurve::constructProperties()
{   
    
    constructProperty_engagement_angle_in_degrees(85);
    constructProperty_stiffness_at_perpendicular();
    constructProperty_curviness();

}


void FiberCompressiveForceCosPennationCurve::buildCurve( bool computeIntegral )
{
    SmoothSegmentedFunction* f = SmoothSegmentedFunctionFactory::
        createFiberCompressiveForceCosPennationCurve(   
                cos(get_engagement_angle_in_degrees()*Pi/180.0), 
                m_stiffnessAtPerpendicularInUse,
                m_curvinessInUse,
                computeIntegral,
                getName());       

    m_curve = *f; 
    
    delete f;  
       
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
            double eAngleRad = get_engagement_angle_in_degrees()
                              *Pi/180.0;

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
            
            m_stiffnessAtPerpendicularInUse = get_stiffness_at_perpendicular();
            m_curvinessInUse = get_curviness();
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

    //Since the name is not counted as a property, but it can change,
    //and needs to be kept up to date.
    std::string name = getName();
    m_curve.setName(name);
}

SimTK::Function* FiberCompressiveForceCosPennationCurve::createSimTKFunction() const
{
    // back the OpenSim::Function with this SimTK::Function 
    
    return SmoothSegmentedFunctionFactory::
        createFiberCompressiveForceCosPennationCurve(   
                cos(get_engagement_angle_in_degrees()*Pi/180.0), 
                m_stiffnessAtPerpendicularInUse,
                m_curvinessInUse,
                false,
                getName());       
}


//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberCompressiveForceCosPennationCurve::
    getEngagementAngleInDegrees() const
{    
    return get_engagement_angle_in_degrees();
}

double FiberCompressiveForceCosPennationCurve::
    getStiffnessAtPerpendicularInUse() const
{
    return m_stiffnessAtPerpendicularInUse;
}

double FiberCompressiveForceCosPennationCurve::
    getCurvinessInUse() const
{
    return m_curvinessInUse;
}

void FiberCompressiveForceCosPennationCurve::
    setEngagementAngleInDegrees(double aEngagementAngleInDegrees)
{   
   set_engagement_angle_in_degrees(aEngagementAngleInDegrees);  
   ensureCurveUpToDate();
}

void FiberCompressiveForceCosPennationCurve::
    setOptionalProperties(double aStiffnessAtPerpendicular, double aCurviness)
{    
    set_stiffness_at_perpendicular(aStiffnessAtPerpendicular);    
    set_curviness(aCurviness);
    ensureCurveUpToDate();
}

bool FiberCompressiveForceCosPennationCurve::isFittedCurveBeingUsed() const
{
    return m_isFittedCurveBeingUsed;
}

//=============================================================================
// SERVICES
//=============================================================================

double FiberCompressiveForceCosPennationCurve::
    calcValue(double cosPennationAngle) const
{   
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveCosPennationCurve: Curve is not"
        " to date with its properties");

    return m_curve.calcValue(cosPennationAngle);
}

double FiberCompressiveForceCosPennationCurve::
    calcDerivative(double cosPennationAngle, int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveCosPennationCurve: Curve is not"
        " to date with its properties");

    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberCompressiveForceCosPennationCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
           
    return m_curve.calcDerivative(cosPennationAngle,order);
}

double FiberCompressiveForceCosPennationCurve::
    calcDerivative(const std::vector<int>& derivComponents,
                   const SimTK::Vector& x) const
{
    return m_curve.calcDerivative(derivComponents, x);
}

double FiberCompressiveForceCosPennationCurve::
    calcIntegral(double cosPennationAngle) const
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveCosPennationCurve: Curve is not"
        " to date with its properties");
    
    if (!m_curve.isIntegralAvailable()) {
        FiberCompressiveForceCosPennationCurve* mutableThis = 
            const_cast<FiberCompressiveForceCosPennationCurve*>(this); 
        mutableThis->buildCurve(true); 
    }
    
    return m_curve.calcIntegral(cosPennationAngle);
}

SimTK::Vec2 FiberCompressiveForceCosPennationCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveCosPennationCurve: Curve is not"
        " to date with its properties");
    return m_curve.getCurveDomain();
}

void FiberCompressiveForceCosPennationCurve::
    printMuscleCurveToCSVFile(const std::string& path)
{
    ensureCurveUpToDate();

    double xmin = 1.0; //cos(0)
    double xmax = 0.0; //cos(SimTK::Pi/2)


    m_curve.printMuscleCurveToCSVFile(path,xmin,xmax);
}
