/* -------------------------------------------------------------------------- *
 *               OpenSim:  FiberCompressiveForceLengthCurve.cpp               *
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
#include "FiberCompressiveForceLengthCurve.h"
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

//=============================================================================
// CONSTRUCTION
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, 
// copy assignment.

FiberCompressiveForceLengthCurve::
        FiberCompressiveForceLengthCurve()
{
    setNull();
    constructProperties();
    setName("default_FiberCompressiveForceLengthCurve");

    ensureCurveUpToDate();
}

FiberCompressiveForceLengthCurve::FiberCompressiveForceLengthCurve
    (   double normLengthAtZeroForce, 
        double stiffnessAtZeroLength,
        double curviness,
        const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_FiberCompressiveForceLengthCurve");

    set_norm_length_at_zero_force(normLengthAtZeroForce);
    set_stiffness_at_zero_length(stiffnessAtZeroLength);
    set_curviness(curviness);

    ensureCurveUpToDate();
}


void FiberCompressiveForceLengthCurve::setNull()
{

    setAuthors("Matthew Millard");
}

void FiberCompressiveForceLengthCurve::constructProperties()
{   
    constructProperty_norm_length_at_zero_force(0.5);
    constructProperty_stiffness_at_zero_length();
    constructProperty_curviness();
}


void FiberCompressiveForceLengthCurve::buildCurve( bool computeIntegral )
{        
    SmoothSegmentedFunction* f = SmoothSegmentedFunctionFactory::
        createFiberCompressiveForceLengthCurve( 
                get_norm_length_at_zero_force(), 
                m_stiffnessAtZeroLengthInUse,  
                m_curvinessInUse,
                computeIntegral,
                getName());            
    
    m_curve = *f;  

    delete f; 

    setObjectIsUpToDateWithProperties();
}


void FiberCompressiveForceLengthCurve::ensureCurveUpToDate() 
{
    if(isObjectUpToDateWithProperties() == false){

        //=====================================================================
        //Compute the optional properties if they have not been set
        //=====================================================================
        if(getProperty_stiffness_at_zero_length().empty() == true &&
            getProperty_curviness().empty() == true)
        {
            double lengthAtZeroForce = get_norm_length_at_zero_force();

            m_stiffnessAtZeroLengthInUse= -2.0/lengthAtZeroForce; 
            m_curvinessInUse = 0.5;
            m_isFittedCurveBeingUsed = true;
        }

        //=====================================================================
        //Use the optional properties if they have been set
        //=====================================================================
        if(getProperty_stiffness_at_zero_length().empty() == false &&
            getProperty_curviness().empty() == false)
        {
            
            m_stiffnessAtZeroLengthInUse = get_stiffness_at_zero_length();
            m_curvinessInUse = get_curviness();
            m_isFittedCurveBeingUsed = false;
        }

        //=====================================================================
        //Error condition if only one optional parameter is set.
        //=====================================================================
        bool a = getProperty_stiffness_at_zero_length().empty();
        bool b = getProperty_curviness().empty();

        //This is really a XOR operation ...
        if( ( a && !b ) || ( !a && b ) ){

            //This error condition is checked to make sure that the reference
            //fiber is being used to populate all optional properties or none
            //of them. Anything different would result in an inconsistency in
            //the computed curve - it wouldn't reflect the reference curve.
            SimTK_ERRCHK1_ALWAYS(false,
                "FiberCompressiveForceLengthCurve::ensureCurveUpToDate()",
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


//=============================================================================
//  OpenSim::Function Interface
//=============================================================================

SimTK::Function* FiberCompressiveForceLengthCurve::createSimTKFunction() const
{
    // back the OpenSim::Function with this SimTK::Function 

    return SmoothSegmentedFunctionFactory::
        createFiberCompressiveForceLengthCurve( 
                get_norm_length_at_zero_force(), 
                m_stiffnessAtZeroLengthInUse,  
                m_curvinessInUse,
                false,
                getName());            
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberCompressiveForceLengthCurve::getNormLengthAtZeroForce() const
{
    return get_norm_length_at_zero_force();
}

double FiberCompressiveForceLengthCurve::getStiffnessAtZeroLengthInUse() const
{
    return m_stiffnessAtZeroLengthInUse;
}

double FiberCompressiveForceLengthCurve::getCurvinessInUse() const
{
    return m_curvinessInUse;
}

bool FiberCompressiveForceLengthCurve::isFittedCurveBeingUsed() const
{
    return m_isFittedCurveBeingUsed;
}


void FiberCompressiveForceLengthCurve::
    setNormLengthAtZeroForce(double aNormLengthAtZeroForce)
{   
    set_norm_length_at_zero_force(aNormLengthAtZeroForce);
    ensureCurveUpToDate();
}

void FiberCompressiveForceLengthCurve::
        setOptionalProperties(double aStiffnessAtZeroLength, double aCurviness)
{
   set_stiffness_at_zero_length(aStiffnessAtZeroLength);
   set_curviness(aCurviness);
   ensureCurveUpToDate();
}


//=============================================================================
// SERVICES
//=============================================================================

double FiberCompressiveForceLengthCurve::calcValue(double aNormLength) const
{        
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveForceLengthCurve: Curve is not"
        " to date with its properties");
    return m_curve.calcValue(aNormLength);
}

double FiberCompressiveForceLengthCurve::calcIntegral(double aNormLength) const
{    
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveForceLengthCurve: Curve is not"
        " to date with its properties");
    
    if (!m_curve.isIntegralAvailable()) {
        FiberCompressiveForceLengthCurve* mutableThis = 
            const_cast<FiberCompressiveForceLengthCurve*>(this); 
        mutableThis->buildCurve(true); 
    }

    return m_curve.calcIntegral(aNormLength);
}

double FiberCompressiveForceLengthCurve::
    calcDerivative(double aNormLength, int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveForceLengthCurve: Curve is not"
        " to date with its properties");

    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberCompressiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
     
    return m_curve.calcDerivative(aNormLength,order);
}

double FiberCompressiveForceLengthCurve::
    calcDerivative(const std::vector<int>& derivComponents,
                   const SimTK::Vector& x) const
{
    return m_curve.calcDerivative(derivComponents, x);
}

SimTK::Vec2 FiberCompressiveForceLengthCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties()==true,
        "FiberCompressiveForceLengthCurve: Curve is not"
        " to date with its properties");   
    return m_curve.getCurveDomain();
}

void FiberCompressiveForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path)
{    
    ensureCurveUpToDate();

    double xmin = 0;
    double xmax = max(1.0,get_norm_length_at_zero_force());

    m_curve.printMuscleCurveToCSVFile(path,xmin,xmax);
}
