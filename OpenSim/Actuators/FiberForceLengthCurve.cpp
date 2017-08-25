/* -------------------------------------------------------------------------- *
 *                    OpenSim:  FiberForceLengthCurve.cpp                     *
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
#include "FiberForceLengthCurve.h"
#include <OpenSim/Common/SmoothSegmentedFunctionFactory.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static int RefFiber_e0_Idx     = 0;
static int RefFiber_e1_Idx     = 1;
static int RefFiber_kPE_Idx    = 2;
static int RefFiber_klin_Idx   = 3;
static int RefFiber_normPE_Idx = 4;

//==============================================================================
// CONSTRUCTION
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment.
FiberForceLengthCurve::FiberForceLengthCurve()
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());
    ensureCurveUpToDate();
}

FiberForceLengthCurve::FiberForceLengthCurve(double strainAtZeroForce,
                                             double strainAtOneNormForce,
                                             double stiffnessAtLowForce,
                                             double stiffnessAtOneNormForce,
                                             double curviness)
{
    setNull();
    constructProperties();
    setName(getConcreteClassName());

    set_strain_at_zero_force(strainAtZeroForce);
    set_strain_at_one_norm_force(strainAtOneNormForce);
    setOptionalProperties(stiffnessAtLowForce,
                          stiffnessAtOneNormForce,
                          curviness);

    ensureCurveUpToDate();
}

void FiberForceLengthCurve::setNull()
{
    setAuthors("Matthew Millard");
}

void FiberForceLengthCurve::constructProperties()
{
    constructProperty_strain_at_zero_force(0.0);
    constructProperty_strain_at_one_norm_force(0.7);
    constructProperty_stiffness_at_low_force();
    constructProperty_stiffness_at_one_norm_force();
    constructProperty_curviness();
}

void FiberForceLengthCurve::buildCurve(bool computeIntegral)
{
    SmoothSegmentedFunction* f = SmoothSegmentedFunctionFactory::
        createFiberForceLengthCurve(
            get_strain_at_zero_force(),
            get_strain_at_one_norm_force(),
            m_stiffnessAtLowForceInUse,
            m_stiffnessAtOneNormForceInUse,
            m_curvinessInUse,
            computeIntegral,
            getName());

    m_curve = *f;
    delete f;

    setObjectIsUpToDateWithProperties();
}

void FiberForceLengthCurve::ensureCurveUpToDate()
{
    if(isObjectUpToDateWithProperties()) {
        return;
    }

    if(!getProperty_stiffness_at_low_force().empty()
        && !getProperty_stiffness_at_one_norm_force().empty()
        && !getProperty_curviness().empty()) {

        m_stiffnessAtOneNormForceInUse = get_stiffness_at_one_norm_force();
        m_stiffnessAtLowForceInUse     = get_stiffness_at_low_force();
        m_curvinessInUse               = get_curviness();
        m_fittedCurveBeingUsed         = false;

    } else if(getProperty_stiffness_at_low_force().empty()
        && getProperty_stiffness_at_one_norm_force().empty()
        && getProperty_curviness().empty()) {

        // Compute optional properties if they haven't been provided.
        double e0 = get_strain_at_zero_force(); //properties of reference curve
        double e1 = get_strain_at_one_norm_force();

        // Assign the stiffnesses. These values are based on the Thelen2003
        // default curve and the EDL passive force-length curve found
        // experimentally by Winters, Takahashi, Ward, and Lieber (2011).

        //SimTK::Vec5 refFiber = calcReferencePassiveFiber(e0,e1);
        //m_stiffnessAtOneNormForceInUse = refFiber[RefFiber_klin_Idx];
        //m_stiffnessAtOneNormForceInUse = (0.6*8.3898637908858689)/(e1-e0);
        m_stiffnessAtOneNormForceInUse = 2.0/(e1-e0);
        m_stiffnessAtLowForceInUse     = 0.2;

        // Fit the curviness parameter to the reference curve, which is the one
        // documented by Thelen. This approach is very slow and Thelen's curve
        // was not based on experimental data.
        //m_curvinessInUse = calcCurvinessOfBestFit(refFiber[RefFiber_e0_Idx],
        //                                        refFiber[RefFiber_e1_Idx],
        //                                        refFiber[RefFiber_klin_Idx],
        //                                        refFiber[RefFiber_normPE_Idx],
        //                                        1.0e-6);
        //m_curvinessInUse = 0.63753341725162227;
        m_curvinessInUse = 0.75;

        m_fittedCurveBeingUsed = true;

    } else {
        // Ensure that the reference curve is being used to populate either all
        // optional properties or none of them; otherwise, there would be an
        // inconsistency in the computed curve and it wouldn't reflect the
        // reference curve.

        SimTK_ERRCHK1_ALWAYS(false,
            "FiberForceLengthCurve::ensureCurveUpToDate()",
            "%s: Optional parameters stiffnessAtLowForce, "
            "stiffnessAtOneNormForce, and curviness must either all be set or "
            "all be empty.", getName().c_str());
    }

    buildCurve();
}

//==============================================================================
// OpenSim::Function Interface
//==============================================================================
SimTK::Function* FiberForceLengthCurve::createSimTKFunction() const
{
    // Back the OpenSim::Function with this SimTK::Function.
    return SmoothSegmentedFunctionFactory::createFiberForceLengthCurve(
                get_strain_at_zero_force(),
                get_strain_at_one_norm_force(),
                m_stiffnessAtLowForceInUse,
                m_stiffnessAtOneNormForceInUse,
                m_curvinessInUse,
                false,
                getName());
}

//==============================================================================
// GET AND SET METHODS
//==============================================================================
double FiberForceLengthCurve::getStrainAtZeroForce() const
{   return get_strain_at_zero_force(); }
double FiberForceLengthCurve::getStrainAtOneNormForce() const
{   return get_strain_at_one_norm_force(); }
double FiberForceLengthCurve::getStiffnessAtLowForceInUse() const
{   return m_stiffnessAtLowForceInUse; }
double FiberForceLengthCurve::getStiffnessAtOneNormForceInUse() const
{   return m_stiffnessAtOneNormForceInUse; }
double FiberForceLengthCurve::getCurvinessInUse() const
{   return m_curvinessInUse; }
bool FiberForceLengthCurve::isFittedCurveBeingUsed() const
{   return m_fittedCurveBeingUsed; }

void FiberForceLengthCurve::setCurveStrains(double aStrainAtZeroForce,
                                            double aStrainAtOneNormForce)
{
    set_strain_at_zero_force(aStrainAtZeroForce);
    set_strain_at_one_norm_force(aStrainAtOneNormForce);
    ensureCurveUpToDate();
}

void FiberForceLengthCurve::setOptionalProperties(double aStiffnessAtLowForce,
                                              double aStiffnessAtOneNormForce,
                                              double aCurviness)
{
    set_stiffness_at_low_force(aStiffnessAtLowForce);
    set_stiffness_at_one_norm_force(aStiffnessAtOneNormForce);
    set_curviness(aCurviness);
    ensureCurveUpToDate();
}

//==============================================================================
// SERVICES
//==============================================================================
double FiberForceLengthCurve::calcValue(double normFiberLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "FiberForceLengthCurve: Curve is not up-to-date with its properties");
    return m_curve.calcValue(normFiberLength);
}

double FiberForceLengthCurve::calcDerivative(double normFiberLength,
                                             int order) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "FiberForceLengthCurve: Curve is not up-to-date with its properties");
    SimTK_ERRCHK1_ALWAYS(order >= 0 && order <= 2,
        "FiberForceLengthCurve::calcDerivative",
        "order must be 0, 1, or 2, but %i was entered", order);

    return m_curve.calcDerivative(normFiberLength,order);
}

double FiberForceLengthCurve::
    calcDerivative(const std::vector<int>& derivComponents,
                   const SimTK::Vector& x) const
{
    return m_curve.calcDerivative(derivComponents, x);
}

double FiberForceLengthCurve::calcIntegral(double normFiberLength) const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "FiberForceLengthCurve: Curve is not up-to-date with its properties");

    if(!m_curve.isIntegralAvailable()) {
        FiberForceLengthCurve* mutableThis =
            const_cast<FiberForceLengthCurve*>(this);
        mutableThis->buildCurve(true);
    }

    return m_curve.calcIntegral(normFiberLength);
}

SimTK::Vec2 FiberForceLengthCurve::getCurveDomain() const
{
    SimTK_ASSERT(isObjectUpToDateWithProperties(),
        "FiberForceLengthCurve: Curve is not up-to-date with its properties");
    return m_curve.getCurveDomain();
}

void FiberForceLengthCurve::printMuscleCurveToCSVFile(const std::string& path)
{
    ensureCurveUpToDate();

    double xmin = 1.0 + get_strain_at_zero_force();
    xmin = xmin*0.9;
    double xmax = 1.0 + get_strain_at_one_norm_force()*1.1;

    m_curve.printMuscleCurveToCSVFile(path, xmin, xmax);
}

//==============================================================================
// PRIVATE SERVICES
//==============================================================================
// NO LONGER IN USE. calcCurvinessOfBestFit fits to Thelen's 2003 paper curve,
// which is not the best source of data (Winter's in-vivo data is). This routine
// is very expensive (takes about 0.5 s to complete in Release mode on a fast
// machine). This computational expense adds up to some very noticeable delays
// when used in a model that has 10-100 muscles.
double FiberForceLengthCurve::calcCurvinessOfBestFit(double e0,   double e1,
                                                     double k,    double ftoe,
                                                     double area, double relTol)
{
    std::string name = getName();
    double c     = 0.5;
    double prevC = 0.5;
    double step  = 0.25;

    SmoothSegmentedFunction* tmp = SmoothSegmentedFunctionFactory::
        createFiberForceLengthCurve(e0, e1, k, ftoe, c, true, name);

    double val = tmp->calcIntegral(1+e1);
    delete tmp;

    double err      = (val-area)/area;
    double prevErr  = 0;
    double errStart = err;
    double errMin   = 0;
    // double solMin   = 0;

    bool flag_improvement = false;
    bool flag_Newton      = false;

    int maxIter   = 20;
    int iter      = 0;
    int localIter = 0;
    int evalIter  = 0;

    while(iter < maxIter && abs(err) > relTol) {
        flag_improvement = false;
        localIter        = 0;

        // Bisection method.
        while(!flag_improvement && localIter < 2 && !flag_Newton) {
            tmp = SmoothSegmentedFunctionFactory::
                createFiberForceLengthCurve(e0, e1, k, ftoe,
                                            c+step, true, name);
            val = tmp->calcIntegral(1+e1);
            delete tmp;

            errMin = (val-area)/area;

            if(abs(errMin) < abs(err)) {
                prevC = c;
                c = c+step;
                if(c > 1.0) {
                    c = 1.0;
                }
                if(c < 0) {
                    c = 0;
                }
                prevErr = err;
                err = errMin;
                flag_improvement = true;

                if(err*prevErr < 0) {
                    step *= -1;
                }

            } else {
                step *= -1;
            }
            localIter++;
        }

        // 1. Check to see whether we can compute a Newton step.
        // 2. Compute a Newton step.
        // 3. If the Newton step is smaller than the current step, take it.
        // 4. If the Newton step results in improvement, update.
        if(abs(err) < abs(errStart)) {
            double dErr    = err-prevErr;
            double dC      = c-prevC;
            double dErrdC  = dErr/dC;
            double deltaC  = -err/dErrdC;
            double cNewton = c + deltaC;

            if(abs(deltaC) < abs(step)) {
                tmp = SmoothSegmentedFunctionFactory::
                    createFiberForceLengthCurve(e0, e1, k, ftoe,
                                                cNewton, true, name);
                val = tmp->calcIntegral(1+e1);
                delete tmp;

                errMin = (val-area)/area;

                if(abs(errMin) < abs(err)) {
                    prevC = c;
                    c = cNewton;
                    if(c > 1.0) {
                        c = 1.0;
                    }
                    if(c < 0) {
                        c = 0;
                    }
                    prevErr = err;
                    err = errMin;
                    flag_improvement = true;
                    flag_Newton = true;
                    step = deltaC;

                    if(prevErr*err < 0) {
                        step *= -1;
                    }
                }
                localIter++;

            } else {
                flag_Newton = false;
            }
        }

        evalIter += localIter;

        step = step/2.0;
        iter++;
    }

    SimTK_ERRCHK1_ALWAYS(abs(err) < abs(relTol)
                         && abs(errStart) > abs(relTol+abs(step)),
        "FiberForceLengthCurve::calcCurvinessOfBestFit()",
        "%s: Not able to fit a fiber curve to the reference fiber curve",
        getName().c_str());

    return c;
}

SimTK::Vec5 FiberForceLengthCurve::calcReferencePassiveFiber(
    double strainAtZeroForce, double strainAtOneNormForce)
{
    SimTK::Vec5 properties;
    double e0  = strainAtZeroForce;
    double e1  = strainAtOneNormForce;
    double eD  = e1-e0;
    double kPE = 5;

    // To expand this equation to make a curve that does not start at a strain
    // of 0, we are changing notation a bit. Now e0 is the strain at zero force
    // and e1 is the strain at 1 unit force.
    //
    // From Thelen 2003, the expression for the fiber force-length curve is
    // F = (e^(kPE*epsilon/e0) - 1) / (e^(kPE) - 1)
    //
    // Replacing e0 with eD = (e1-e0) yields
    // F = (e^(kPE*epsilon/eD) - 1) / (e^(kPE) - 1)
    //
    // Thus, dF/depsilon = (kPE/eD) e^(kPE*epsilon/eD) / (e^(kPE) - 1)
    // The stiffness we want is given by this equation evaluated at epsilon=eD.
    double epsilon = eD;
    double expkPE = exp(kPE);
    double dFdepsilon = (kPE/eD)*exp(kPE*epsilon/eD) / (expkPE-1);

    // The expression for the fiber force-length curve we are using is
    // F = (e^(kPE*epsilon/eD) - 1) / (e^(kPE) - 1)
    // and the integral is
    // int(F,epsilon) = ((eD/kPE)*e^(kPE*epsilon/eD) - epsilon) / (e^(kPE) - 1)
    epsilon = eD;
    double intF1 = ((eD/kPE)*exp(kPE*epsilon/eD) - epsilon)/(expkPE-1);
    epsilon = 0;
    double intF0 = ((eD/kPE)*exp(kPE*epsilon/eD) - epsilon)/(expkPE-1);

    // Populate the property vector.
    properties[RefFiber_e0_Idx]     = e0;
    properties[RefFiber_e1_Idx]     = e1;
    properties[RefFiber_kPE_Idx]    = kPE;
    properties[RefFiber_klin_Idx]   = dFdepsilon;
    properties[RefFiber_normPE_Idx] = intF1-intF0;

    return properties;
}
