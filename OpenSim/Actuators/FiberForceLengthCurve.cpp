/* -------------------------------------------------------------------------- *
 *                    OpenSim:  FiberForceLengthCurve.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Matthew Millard                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
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

    set_strain_at_one_norm_force(strainAtOneNormForce);
    setOptionalProperties(stiffnessAtOneNormForce, curviness);

    ensureCurveUpToDate();
}


void FiberForceLengthCurve::setNull()
{

}

void FiberForceLengthCurve::constructProperties()
{   

    constructProperty_strain_at_one_norm_force(0.6);
    constructProperty_stiffness_at_one_norm_force();
    constructProperty_curviness();
}

void FiberForceLengthCurve::buildCurve()
{           
        double e0   =  get_strain_at_one_norm_force();
        double kiso =  m_stiffnessAtOneNormForceInUse;
        double c    =  m_curvinessInUse;        

        SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                    kiso,
                                                                    c,
                                                                    true,
                                                                    getName());            
        this->m_curve = tmp;
       
    setObjectIsUpToDateWithProperties();
}

void FiberForceLengthCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties() == false){
        

        //=====================================================================
        //Compute optional properties if they haven't been provided
        //=====================================================================
        if(    getProperty_stiffness_at_one_norm_force().empty() == true
            && getProperty_curviness().empty() == true)
        {
            //Get properties of the reference curve
            double e0 = get_strain_at_one_norm_force();
            SimTK::Vec4 refFiber = calcReferencePassiveFiber(e0);    

            //Assign the stiffness
            m_stiffnessAtOneNormForceInUse = refFiber[RefFiber_klin_Idx];

            //Fit the curviness parameter            
            m_curvinessInUse = calcCurvinessOfBestFit(refFiber[RefFiber_e0_Idx],
                                                refFiber[RefFiber_klin_Idx],
                                                refFiber[RefFiber_normPE_Idx],
                                                1e-6);
            m_fittedCurveBeingUsed = true;

        }

        //=====================================================================
        //Get optional properties if they've both been provided
        //=====================================================================
        if(    getProperty_stiffness_at_one_norm_force().empty() == false
            && getProperty_curviness().empty() == false){

                
            m_stiffnessAtOneNormForceInUse
                                = get_stiffness_at_one_norm_force();
            m_curvinessInUse    = get_curviness();

            m_fittedCurveBeingUsed = false;
        }

        //=====================================================================
        //Error condition if only one optional parameter is set.
        //=====================================================================
        bool a = getProperty_stiffness_at_one_norm_force().empty();
        bool b = getProperty_curviness().empty();

        //This is really a XOR operation ...
        if( ( a && !b ) || ( !a && b ) ){

            //This error condition is checked to make sure that the reference
            //fiber is being used to populate all optional properties or none
            //of them. Anything different would result in an inconsistency in
            //the computed curve - it wouldn't reflect the reference curve.

            SimTK_ERRCHK1_ALWAYS(false,
                "FiberForceLengthCurve::ensureCurveUpToDate()",
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
void FiberForceLengthCurve::connectToModel(Model& model)
{
    Super::connectToModel(model);
}

void FiberForceLengthCurve::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void FiberForceLengthCurve::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double FiberForceLengthCurve::getStrainAtOneNormForce() const
{    
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return get_strain_at_one_norm_force();
}


double FiberForceLengthCurve::getStiffnessAtOneNormForceInUse() const
{    
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_stiffnessAtOneNormForceInUse;
}

double FiberForceLengthCurve::getCurvinessInUse() const
{
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curvinessInUse;
}

bool FiberForceLengthCurve::isFittedCurveBeingUsed() const
{
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_fittedCurveBeingUsed;
}

void FiberForceLengthCurve::
    setStrainAtOneNormForce(double aStrainAtOneNormForce)
{
        set_strain_at_one_norm_force(aStrainAtOneNormForce);   
}

void FiberForceLengthCurve::
        setOptionalProperties(double aStiffnessAtOneNormForce,double aCurviness)
{
        set_stiffness_at_one_norm_force(aStiffnessAtOneNormForce);
        set_curviness(aCurviness);
}

//=============================================================================
// SERVICES
//=============================================================================

double FiberForceLengthCurve::
    calcValue(double aNormLength) const
{
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curve.calcValue(aNormLength);
}

double FiberForceLengthCurve::
    calcDerivative(double aNormLength, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "FiberForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curve.calcDerivative(aNormLength,order);
}

double FiberForceLengthCurve::
    calcIntegral(double aNormLength) const
{
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curve.calcIntegral(aNormLength);
}

SimTK::Vec2 FiberForceLengthCurve::getCurveDomain() const
{
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    return m_curve.getCurveDomain();
}

void FiberForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    FiberForceLengthCurve* mthis = const_cast<FiberForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();

    m_curve.printMuscleCurveToCSVFile(path);
}

//=============================================================================
// PRIVATE SERVICES
//=============================================================================

double FiberForceLengthCurve::calcCurvinessOfBestFit(double e0, double k, 
                                                   double area, double relTol)
{

            std::string name = getName();

            double c = 0.5;
            double prevC = 0.5;
            double step = 0.25;
            
            SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                   k,
                                                                   c,
                                                                   true,
                                                                   name);

            double val = tmp.calcIntegral(1+e0);

            double err      = (val-area)/area;
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


            while(iter < maxIter && abs(err) > relTol){ 
                flag_improvement = false;
                localIter        = 0;                

                //Bisection
                while(flag_improvement == false && localIter < 2 
                        && flag_Newton == false){

                    SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                   k,
                                                                   c+step,
                                                                   true,
                                                                   name);                
                    val     = tmp.calcIntegral(1+e0);
                    errMin  = (val-area)/area; 
                                    
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
                       SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
                                    createFiberForceLengthCurve(   e0,
                                                                   k,
                                                                   cNewton,
                                                                   true,
                                                                   name);                 
                        val     = tmp.calcIntegral(1+e0);
                        errMin  = (val-area)/area; 
                                    
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

            SimTK_ERRCHK1_ALWAYS(abs(err) < abs(relTol) 
                         && abs(errStart) > abs(relTol+abs(step)),
                "FiberForceLengthCurve::calcCurvinessOfBestFit()",
                "%s: Not able to fit a fiber curve to the reference"
                "fiber curve",getName().c_str());

            return c;
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
