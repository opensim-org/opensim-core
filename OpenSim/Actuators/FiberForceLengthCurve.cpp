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
#include "FiberForceLengthCurve.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static int RefFiber_e0_Idx     = 0;
static int RefFiber_kPE_Idx    = 1;
static int RefFiber_klin_Idx   = 2;
static int RefFiber_normPE_Idx = 3;

//=============================================================================
// CONSTRUCTION
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, 
// copy assignment.

FiberForceLengthCurve::FiberForceLengthCurve()
{
    setNull();
    constructProperties();
    setName("default_FiberForceLengthCurve");
}

FiberForceLengthCurve::FiberForceLengthCurve(   double strainAtOneNormForce, 
                                                double stiffnessAtOneNormForce,
                                                double curviness,
                                                const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_FiberForceLengthCurve");

    setStrainAtOneNormForce(strainAtOneNormForce);
    setStiffnessAtOneNormForce(stiffnessAtOneNormForce);
    setCurviness(curviness);

    buildCurve();
}


void FiberForceLengthCurve::setNull()
{
    //Sherm says this is taken care of automatically
    //setObjectIsUpToDateWithProperties(false);
}

void FiberForceLengthCurve::constructProperties()
{   

    constructProperty_strain_at_one_norm_force(0.6);
    constructProperty_stiffness_at_one_norm_force();
    constructProperty_curviness();
}

void FiberForceLengthCurve::buildCurve()
{
    if(isObjectUpToDateWithProperties() == false){
        
        double e0   =  getStrainAtOneNormForce();
        double kiso =  getStiffnessAtOneNormForceInUse();
        double c    =  getCurvinessInUse();        

        //Here's where you call the MuscleCurveFunctionFactory
        MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                    kiso,
                                                                    c,
                                                                    true,
                                                                    getName());            

        this->m_curve = tmp;
          
    }
    setObjectIsUpToDateWithProperties(true);
}


//=============================================================================
// MODEL COMPPONENT INTERFACE
//=============================================================================
void FiberForceLengthCurve::setup(Model& model)
{
    Super::setup(model);
}

void FiberForceLengthCurve::initState(SimTK::State& s) const
{
    Super::initState(s);
}

void FiberForceLengthCurve::createSystem(SimTK::MultibodySystem& system) const
{
    Super::createSystem(system);

    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->buildCurve();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberForceLengthCurve::getStrainAtOneNormForce()
{
    return getProperty_strain_at_one_norm_force();
}

double FiberForceLengthCurve::getStiffnessAtOneNormForce()
{
    return getProperty_stiffness_at_one_norm_force();
}

double FiberForceLengthCurve::getStiffnessAtOneNormForceInUse()
{    
    if(getProperty_stiffness_at_one_norm_force().empty() == true)
    {
        //This error condition is checked to make sure that the reference
        //fiber is being used to populate all optional properties or none
        //of them. Anything different would result in an inconsistency in
        //the computed curve - it wouldn't reflect the reference curve.

        SimTK_ERRCHK1_ALWAYS(getProperty_curviness().empty(),
            "FiberForceLengthCurve::getCurvinessInUse()",
            "%s: Optional parameters stiffness and curviness must both"
            "be set, or both remain empty. You have set one parameter"
            "and left the other blank.",
            getName().c_str());  
        
        if(isObjectUpToDateWithProperties() == false &&
            m_stiffnessAtOneNormForceInUse[1] != getStrainAtOneNormForce())
        {
            SimTK::Vec4 refFiber = getReferencePassiveFiber();
            m_stiffnessAtOneNormForceInUse[0] = refFiber[RefFiber_klin_Idx];
            m_stiffnessAtOneNormForceInUse[1] = refFiber[RefFiber_e0_Idx];
        }
    }else{
        m_stiffnessAtOneNormForceInUse[0] = getStiffnessAtOneNormForce();
        m_stiffnessAtOneNormForceInUse[1] = SimTK::NaN;
    }

    return m_stiffnessAtOneNormForceInUse[0];
}


double FiberForceLengthCurve::getCurviness()
{
    return getProperty_curviness();
}


double FiberForceLengthCurve::getCurvinessInUse()
{
    if(getProperty_curviness().empty() == true)
    {
        //This error condition is checked to make sure that the reference
        //fiber is being used to populate all optional properties or none
        //of them. Anything different would result in an inconsistency in
        //the computed curve - it wouldn't reflect the reference curve.

        SimTK_ERRCHK1_ALWAYS(getProperty_stiffness_at_one_norm_force().empty(),
            "FiberForceLengthCurve::getCurvinessInUse()",
            "%s: Optional parameters stiffness and curviness must both"
            "be set, or both remain empty. You have set one parameter"
            "and left the other blank.",
            getName().c_str()); 

        if(isObjectUpToDateWithProperties() == false &&
            m_curvinessInUse[1] != getStrainAtOneNormForce()){
    
            SimTK::Vec4 refFiber = getReferencePassiveFiber();

            double e0       = refFiber[RefFiber_e0_Idx];
            double k        = refFiber[RefFiber_klin_Idx];
            double peNorm   = refFiber[RefFiber_normPE_Idx];

            //Use the bisection method and Newton's method to find the
            //value of the curviness parameter that creates a curve with the
            //same area under it between 0 and e0
            //==================================================================

            double tol = 1e-6;
            std::string name = getName();

            double c = 0.5;
            double prevC = 0.5;
            double step = 0.25;
            
            MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                   k,
                                                                   c,
                                                                   true,
                                                                   name);

            double val = tmp.calcIntegral(1+e0);

            double err      = (val-peNorm)/peNorm;
            double prevErr = 0;
            double errStart = err;
            double errMin   = 0;
            
            double solMin           = 0;
            bool flag_improvement   = false;
            bool flag_Newton        = false;

            int maxIter     = 10;
            int iter        = 0;
            int localIter   = 0;
            int evalIter    = 0;


            while(iter < maxIter && abs(err) > tol){ 
                flag_improvement = false;
                localIter        = 0;                

                //Bisection
                while(flag_improvement == false && localIter < 2 
                        && flag_Newton == false){

                    MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                   k,
                                                                   c+step,
                                                                   true,
                                                                   name);                
                    val     = tmp.calcIntegral(1+e0);
                    errMin  = (val-peNorm)/peNorm; 
                                    
                    if(abs(errMin) < abs(err)){
                        prevC = c;
                        c   = c+step;
                        prevErr = err;
                        err = errMin;
                        flag_improvement = true;

                        if(err*prevErr < 0)
                            step = step*-1;
                        
                    }else{
                        step = step*-1;
                    }
                    localIter++;
               }
                //0. Check if we can compute a Newton step
                //1. Compute a Newton step 
                //2. If the Newton step is smaller than the current step, take it
                //3. If the Newton step results in improvement, update.

                if(abs(err) < abs(errStart)){
                    double dErr = err-prevErr;
                    double dC   = c-prevC; 
                    double dErrdC= dErr/dC;
                    double deltaC = -err/dErrdC;
                    double cNewton = c + deltaC;

                    if(abs(deltaC) < abs(step)){
                       MuscleCurveFunction tmp = MuscleCurveFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                   k,
                                                                   cNewton,
                                                                   true,
                                                                   name);                 
                        val     = tmp.calcIntegral(1+e0);
                        errMin  = (val-peNorm)/peNorm; 
                                    
                        if(abs(errMin) < abs(err)){
                            prevC = c;
                            c   = cNewton;
                            prevErr = err;
                            err = errMin;
                            flag_improvement = true;
                            flag_Newton = true;
                            step = deltaC;

                            if(prevErr*err < 0)
                                step = step*-1;
                      }
                        localIter++;
                    }
                }

                evalIter += localIter;
                step = step/2.0;
                iter++;
                //printf("Root Finding Iter %i ,err: %f, eval %i\n",
                //    iter,err,evalIter);
            }

            SimTK_ERRCHK1_ALWAYS(err < tol && errStart > (tol+step),
                "FiberForceLengthCurve::getCurvinessInUse()",
                "%s: Not able to fit a fiber curve to the reference"
                "fiber curve",getName().c_str());


            m_curvinessInUse[0] = c;
            m_curvinessInUse[1] = refFiber[RefFiber_e0_Idx];
            //==================================================================          
        }
    }else{
        m_curvinessInUse[0] = getCurviness();
        m_curvinessInUse[1] = SimTK::NaN;
    }

    return m_curvinessInUse[0];
}


SimTK::Vec4 FiberForceLengthCurve::getReferencePassiveFiber()
{
    //Update the properties if they haven't been computed yet, 
    //or if the strain value they are based on has changed.

    if(m_FiberReference.size()==0){
        double e0 = getStrainAtOneNormForce();
        m_FiberReference = calcReferencePassiveFiber(e0);
    }else if(getStrainAtOneNormForce() != m_FiberReference[RefFiber_e0_Idx]){
        double e0 = getStrainAtOneNormForce();
        m_FiberReference = calcReferencePassiveFiber(e0);    
    }
    return m_FiberReference;
}

SimTK::Vec4 FiberForceLengthCurve::
    calcReferencePassiveFiber(double strainAtOneNormForce)
{
    SimTK::Vec4 properties;
    double e0 = strainAtOneNormForce;
    double kPE = 5;

    //From Thelen 2003, the expression for the fiber force length curve is:
    // F = (e^(kPE*epsilon/e0) - 1) / (e^(kPE) - 1)
    // Thus
    // dF/depsilon = (kPE/e0) e^(kPE*epsilon/e0)  / (e^(kPE) - 1)
    // The stiffness we want is given by this equation evaluated at
    // epsilon = e0:

    double expkPE = exp(kPE);
    double dFdepsilon = (kPE/e0)*expkPE / (expkPE-1);

    //From Thelen 2003, the expression for the fiber force length curve is:
    // F = (e^(kPE*epsilon/e0) - 1) / (e^(kPE) - 1)
    // And the integral is
    // int(F,epsilon) = ((e0/kPE)*e^(kPE*epsilon/e0) - epsilon) / (e^(kPE) - 1)

    double epsilon = e0;
    double intF1 = ((e0/kPE)*exp(kPE*epsilon/e0) - epsilon) / (expkPE - 1);
    epsilon = 0;
    double intF0 = ((e0/kPE)*exp(kPE*epsilon/e0) - epsilon) / (expkPE - 1);

    //Populate the property vector:

    
     properties[RefFiber_e0_Idx]     = e0;
     properties[RefFiber_kPE_Idx]    = kPE;
     properties[RefFiber_klin_Idx]   = dFdepsilon;
     properties[RefFiber_normPE_Idx] = intF1-intF0;
            
    return properties;
}

void FiberForceLengthCurve::
    setStrainAtOneNormForce(double aStrainAtOneNormForce)
{
    if(aStrainAtOneNormForce != getStrainAtOneNormForce() )
    {
        setProperty_strain_at_one_norm_force(aStrainAtOneNormForce);
        //Sherm says this is taken care of automatically
        //setObjectIsUpToDateWithProperties(false);
    }
}

void FiberForceLengthCurve::
        setStiffnessAtOneNormForce(double aStiffnessAtOneNormForce)
{
    //if(aStiffnessAtOneNormForce != getStiffnessAtOneNormForce() )
    //{
        setProperty_stiffness_at_one_norm_force(aStiffnessAtOneNormForce);
        //Sherm says this is taken care of automatically
        //setObjectIsUpToDateWithProperties(false);
    //}
}

void FiberForceLengthCurve::setCurviness(double aCurviness)
{
    //if(aCurviness != getCurviness() )
    //{
        setProperty_curviness(aCurviness);
        //Sherm says this is taken care of automatically
        //setObjectIsUpToDateWithProperties(false);
    //}
}


//=============================================================================
// SERVICES
//=============================================================================

double FiberForceLengthCurve::
    calcValue(double aNormLength) const
{
    if(isObjectUpToDateWithProperties() == false){
        FiberForceLengthCurve* mthis = 
            const_cast<FiberForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcValue(aNormLength);
}

double FiberForceLengthCurve::
    calcDerivative(double aNormLength, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    if(isObjectUpToDateWithProperties() == false){
        FiberForceLengthCurve* mthis = 
            const_cast<FiberForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcDerivative(aNormLength,order);
}

double FiberForceLengthCurve::
    calcIntegral(double aNormLength) const
{
    if(isObjectUpToDateWithProperties() == false){
        FiberForceLengthCurve* mthis = 
            const_cast<FiberForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.calcIntegral(aNormLength);
}

SimTK::Vec2 FiberForceLengthCurve::getCurveDomain() const
{
    if(isObjectUpToDateWithProperties() == false){
        FiberForceLengthCurve* mthis = 
            const_cast<FiberForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    return m_curve.getCurveDomain();
}

void FiberForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    if(isObjectUpToDateWithProperties() == false){
        FiberForceLengthCurve* mthis = 
            const_cast<FiberForceLengthCurve*>(this);    
        mthis->buildCurve();    
    }

    m_curve.printMuscleCurveToCSVFile(path);
}