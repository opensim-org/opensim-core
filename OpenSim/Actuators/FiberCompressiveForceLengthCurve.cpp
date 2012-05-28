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
}

void FiberCompressiveForceLengthCurve::constructProperties()
{   
    constructProperty_norm_length_at_zero_force(0.5);
    constructProperty_stiffness_at_zero_length();
    constructProperty_curviness();
}


void FiberCompressiveForceLengthCurve::buildCurve()
{        
        double l0   =  get_norm_length_at_zero_force();
        double k    =  m_stiffnessAtZeroLengthInUse;
        double c    =  m_curvinessInUse;        

        //Here's where you call the SmoothSegmentedFunctionFactory
        SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
            createFiberCompressiveForceLengthCurve( l0,
                                                    k,
                                                    c,
                                                    true,
                                                    getName());            
        this->m_curve = tmp;
          
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
            
            m_stiffnessAtZeroLengthInUse = 
                get_stiffness_at_zero_length();
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
}


//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void FiberCompressiveForceLengthCurve::setup(Model& aModel)
{
    Super::setup(aModel);
}

void FiberCompressiveForceLengthCurve::initState(SimTK::State& s) const
{
    Super::initState(s);
}

void FiberCompressiveForceLengthCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberCompressiveForceLengthCurve::getNormLengthAtZeroForce() const
{
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return get_norm_length_at_zero_force();
}

double FiberCompressiveForceLengthCurve::getStiffnessAtZeroLengthInUse() const
{
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_stiffnessAtZeroLengthInUse;
}

double FiberCompressiveForceLengthCurve::getCurvinessInUse() const
{
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curvinessInUse;
}

bool FiberCompressiveForceLengthCurve::isFittedCurveBeingUsed() const
{
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_isFittedCurveBeingUsed;
}


void FiberCompressiveForceLengthCurve::
    setNormLengthAtZeroForce(double aNormLengthAtZeroForce)
{   
    set_norm_length_at_zero_force(aNormLengthAtZeroForce);
}

void FiberCompressiveForceLengthCurve::
        setOptionalProperties(double aStiffnessAtZeroLength, double aCurviness)
{
   set_stiffness_at_zero_length(aStiffnessAtZeroLength);
   set_curviness(aCurviness);
}


//=============================================================================
// SERVICES
//=============================================================================

double FiberCompressiveForceLengthCurve::calcValue(double aNormLength) const
{    
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.calcValue(aNormLength);
}

double FiberCompressiveForceLengthCurve::calcIntegral(double aNormLength) const
{    
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.calcIntegral(aNormLength);
}

double FiberCompressiveForceLengthCurve::
    calcDerivative(double aNormLength, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberCompressiveForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
     
        FiberCompressiveForceLengthCurve* mthis = 
            const_cast<FiberCompressiveForceLengthCurve*>(this);    
        mthis->ensureCurveUpToDate();    

    return m_curve.calcDerivative(aNormLength,order);
}

SimTK::Vec2 FiberCompressiveForceLengthCurve::getCurveDomain() const
{
   
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    return m_curve.getCurveDomain();
}

void FiberCompressiveForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{    
    FiberCompressiveForceLengthCurve* mthis = 
        const_cast<FiberCompressiveForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    
    m_curve.printMuscleCurveToCSVFile(path);
}