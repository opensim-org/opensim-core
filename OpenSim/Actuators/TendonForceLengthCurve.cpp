/* -------------------------------------------------------------------------- *
 *                    OpenSim:  TendonForceLengthCurve.cpp                    *
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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
#include "TendonForceLengthCurve.h"
#include "SimTKmath.h"

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static int RefTendon_e0_Idx     = 0;
static int RefTendon_etoe_Idx   = 1;
static int RefTendon_Ftoe_Idx   = 2;
static int RefTendon_ktoe_Idx   = 3;
static int RefTendon_klin_Idx   = 4;
static int RefTendon_normPE_Idx = 5;

//=============================================================================
// CONSTRUCTION, COPY CONSTRUCTION, ASSIGNMENT
//=============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

TendonForceLengthCurve::TendonForceLengthCurve()
{
    setNull();
    constructProperties();
    setName("default_TendonForceLengthCurve");
}

TendonForceLengthCurve::TendonForceLengthCurve( double strainAtOneNormForce, 
                                                double stiffnessAtOneNormForce,
                                                double curviness,
                                                const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_TendonForceLengthCurve");

    setStrainAtOneNormForce(strainAtOneNormForce);
    setOptionalProperties(stiffnessAtOneNormForce,curviness);

    ensureCurveUpToDate();
}


TendonForceLengthCurve::TendonForceLengthCurve( double strainAtOneNormForce, 
                                                const std::string& muscleName)
{
    setNull();
    constructProperties();
    setName(muscleName + "_TendonForceLengthCurve");

    setStrainAtOneNormForce(strainAtOneNormForce);
    ensureCurveUpToDate();
}

void TendonForceLengthCurve::setNull()
{
}

void TendonForceLengthCurve::constructProperties()
{   
    constructProperty_strain_at_one_norm_force(0.04);
    constructProperty_stiffness_at_one_norm_force();
    constructProperty_curviness();
}


void TendonForceLengthCurve::buildCurve()
{            
        double e0   =  get_strain_at_one_norm_force();
        double k    =  m_stiffnessAtOneNormForceInUse;                       
        double c    =  m_curvinessInUse;        

        //Here's where you call the SmoothSegmentedFunctionFactory
        SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
                                    createTendonForceLengthCurve(   e0,
                                                                    k,
                                                                    c,
                                                                    true,
                                                                    getName());            
        this->m_curve = tmp;              
        setObjectIsUpToDateWithProperties();
}

void TendonForceLengthCurve::ensureCurveUpToDate()
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
        SimTK::Vector refTendon = calcReferenceTendon(e0);    

        //Assign the stiffness
        m_stiffnessAtOneNormForceInUse = refTendon[RefTendon_klin_Idx];

        //Fit the curviness parameter            
        m_curvinessInUse=calcCurvinessOfBestFit(refTendon[RefTendon_e0_Idx],
                                                refTendon[RefTendon_klin_Idx],
                                                refTendon[RefTendon_normPE_Idx],
                                                1e-6);
        m_isFittedCurveBeingUsed = true;
    }

    //=====================================================================
    //Get optional properties if they've both been provided
    //=====================================================================
    if(    getProperty_stiffness_at_one_norm_force().empty() == false
        && getProperty_curviness().empty() == false){

                
        m_stiffnessAtOneNormForceInUse
                            = get_stiffness_at_one_norm_force();
        m_curvinessInUse    = get_curviness();
        m_isFittedCurveBeingUsed = false;
    }

    //=====================================================================
    //Error condition if only one optional parameter is set.
    //=====================================================================
    bool a = getProperty_stiffness_at_one_norm_force().empty();
    bool b = getProperty_curviness().empty();

    //This is really a XOR operation ...
    if( ( a && !b ) || ( !a && b ) ){

        //This error condition is checked to make sure that the reference
        //tendon is being used to populate all optional properties or none
        //of them. Anything different would result in an inconsistency in
        //the computed curve - it wouldn't reflect the reference curve.

        SimTK_ERRCHK1_ALWAYS(false,
            "TendonForceLengthCurve::ensureCurveUpToDate()",
            "%s: Optional parameters stiffness and curviness must both"
            "be set, or both remain empty. You have set one parameter"
            "and left the other blank.",
            getName().c_str());  
    }

    buildCurve();
}

}


//=============================================================================
// MODEL COMPONENT INTERFACE
//=============================================================================
void TendonForceLengthCurve::connectToModel(Model& model)
{
    Super::connectToModel(model);
}

void TendonForceLengthCurve::initStateFromProperties(SimTK::State& s) const
{
    Super::initStateFromProperties(s);
}

void TendonForceLengthCurve::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);

    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();
}



//=============================================================================
// GET & SET METHODS
//=============================================================================
double TendonForceLengthCurve::getStrainAtOneNormForce() const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return get_strain_at_one_norm_force();
}


double TendonForceLengthCurve::getStiffnessAtOneNormForceInUse() const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return m_stiffnessAtOneNormForceInUse;
}


double TendonForceLengthCurve::getCurvinessInUse() const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return m_curvinessInUse;
}

bool TendonForceLengthCurve::isFittedCurveBeingUsed() const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();
    return m_isFittedCurveBeingUsed;
}

void TendonForceLengthCurve::
    setStrainAtOneNormForce(double aStrainAtOneNormForce)
{         
    set_strain_at_one_norm_force(aStrainAtOneNormForce);  
}

void TendonForceLengthCurve::
    setOptionalProperties(double aStiffnessAtOneNormForce, double aCurviness)
{
    set_stiffness_at_one_norm_force(aStiffnessAtOneNormForce);
    set_curviness(aCurviness);
}




//=============================================================================
// SERVICES
//=============================================================================

double TendonForceLengthCurve::
    calcValue(double aNormLength) const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    return m_curve.calcValue(aNormLength);
}

double TendonForceLengthCurve::
    calcDerivative(double aNormLength, int order) const
{
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2, 
        "TendonForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);
    
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    return m_curve.calcDerivative(aNormLength,order);
}

double TendonForceLengthCurve::calcIntegral(double aNormLength) const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    return m_curve.calcIntegral(aNormLength);
}

SimTK::Vec2 TendonForceLengthCurve::getCurveDomain() const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    return m_curve.getCurveDomain();
}

void TendonForceLengthCurve::
    printMuscleCurveToCSVFile(const std::string& path) const
{
    TendonForceLengthCurve* mthis = const_cast<TendonForceLengthCurve*>(this);    
    mthis->ensureCurveUpToDate();    
    m_curve.printMuscleCurveToCSVFile(path);
}


//==============================================================================
// Private Services
//==============================================================================

double TendonForceLengthCurve::
    calcCurvinessOfBestFit(double e0, double klin, double area, double relTol)
{

    std::string name = getName();
    //double c_opt = fitToReferenceTendon(e0, klin,peNorm,tol,name);

    double c = 0.5;
    double prevC = 0;
    double step = 0.25;
            
    SmoothSegmentedFunction tmp = SmoothSegmentedFunctionFactory::
            createTendonForceLengthCurve(   e0,
                                            klin,
                                            c,
                                            true,
                                            getName());

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
                            createTendonForceLengthCurve(   e0,
                                                            klin,
                                                            c+step,
                                                            true,
                                                            getName());                
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

        //0. Check if we can numerically calculate a Newton step
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
                            createTendonForceLengthCurve(   e0,
                                                            klin,
                                                            cNewton,
                                                            true,
                                                            getName());                
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

    SimTK_ERRCHK1_ALWAYS(   abs(err) < relTol 
                        &&  abs(errStart) > abs(relTol+abs(step)),
        "TendonForceLengthCurve::calcCurvinessOfBestFit()",
        "%s: Not able to fit a tendon curve to the reference"
        "tendon curve",getName().c_str());

    return c;

}

SimTK::Vector TendonForceLengthCurve::
    calcReferenceTendon(double strainAtOneNormForce)
{
    SimTK::Vector tdnProp(6);

    //Calculate the stiffness given the strain at 1 norm force.
    //Use the extrapolated exponential tendon curve documented in
    //Thelen, 2003

        double FToe = 0.33; //Normalized force at which the transition from
                            //the exponential to the linear function is made
        double kToe = 3.0;  //The exponential shape factor
        double e0 = strainAtOneNormForce;

    //Compute the normalized strain at which the transition from an 
    //exponential to a linear function is made.
        double FkToe = FToe*kToe;
        double expkToe = exp(kToe);
        double eToe = FkToe*expkToe*e0 / (-0.1e1+ FkToe*expkToe
                                        + expkToe - FToe*expkToe + FToe);
        double kLin = (1-FToe) / (e0 - eToe);

    //Compute the (normalized) potental energy stored at e0. 
        //For a tendon with a strain of 0.04 under 1 unit load, the area
        //computed using the trapezoidal method in Matlab is 
        //1.266816749781739e-002 
    
        //Compute the potental energy stored in the toe region of the exponental
        double peNormToe = (FToe/(expkToe-1))*( ((eToe/kToe)*exp(kToe) - eToe) 
                                          - ((eToe/kToe)*exp(0.0) -  0.0));
        //Compute the potential energy stored in the linear region of the
        //curve between the toe region and e0
        double peNormLin = (kLin*(0.5*e0*e0       - eToe*e0   ) + FToe*e0)
                      -(kLin*(0.5*eToe*eToe - eToe*eToe) + FToe*eToe); 
        //Sum the total
        double peNormTotal = peNormToe + peNormLin;
         
        tdnProp[RefTendon_e0_Idx]   = strainAtOneNormForce;
        tdnProp[RefTendon_etoe_Idx] = eToe;
        tdnProp[RefTendon_Ftoe_Idx] = FToe; 
        tdnProp[RefTendon_ktoe_Idx] = kToe;
        tdnProp[RefTendon_klin_Idx] = kLin;
        tdnProp[RefTendon_normPE_Idx] = peNormTotal;

    return tdnProp;
}
